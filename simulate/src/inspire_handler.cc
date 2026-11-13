#include "inspire_handler.h"

#include <fcntl.h>
#include <poll.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

namespace grove_g1
{
namespace
{

constexpr std::size_t kActuators = 6;

// Register order: pinky, ring, middle, index, thumb bend, thumb rotation.
constexpr std::array<const char*, kActuators> kJointSuffixes = {
    "pinky_proximal_joint", "ring_proximal_joint",        "middle_proximal_joint",
    "index_proximal_joint", "thumb_proximal_pitch_joint", "thumb_proximal_yaw_joint",
};

// Mirror grove-g1's g1_description/config/inspire_params.yaml (open_rad, closed_rad, lever_arm_m);
// grove-g1's test_inspire_sim holds them equal. ANGLE 1000 is open, 0 closed, linear between.
constexpr std::array<double, kActuators> kOpenRad    = { 0.0, 0.0, 0.0, 0.0, 0.0, -0.1 };
constexpr std::array<double, kActuators> kClosedRad  = { 1.7, 1.7, 1.7, 1.7, 0.5, 1.3 };
constexpr std::array<double, kActuators> kLeverArmM  = { 0.05, 0.05, 0.05, 0.05, 0.07, 0.03 };
// The DFQ's rated force, g: 4 N at a finger, 6 N at the thumb.
constexpr std::array<double, kActuators> kRatedForceG = { 400, 400, 400, 400, 600, 600 };

constexpr double kNewtonsPerGram = 9.80665e-3;
// s. ANGLE 1000 to 0 at SPEED_SET 1000: the DFQ flexes a finger at about 570 deg/s.
constexpr double kFullStrokeS = 0.2;
// The actuated joint's position loop: stiff, like the lead screw it stands for.
constexpr double kKp = 5.0;   // N m/rad
constexpr double kKd = 0.05;  // N m s/rad
// A stopped actuator holds against up to this multiple of its rated force: a lead screw does not
// back-drive.
constexpr double kHoldMargin = 2.0;
// s at rated force before a stalled actuator latches locked rotor, as the real one does.
constexpr double kLockAfterS = 2.0;
// rad/s. Slower than this a closing digit counts as blocked, so its force can stop it: one still
// travelling is only overcoming its own drag.
constexpr double kBlockedRadS = 0.5;

constexpr int    kTickHz          = 1000;
constexpr double kBaud            = 115200.0;
constexpr double kTurnaroundS     = 0.0005;  // between a request's last byte and the reply
constexpr int    kPowerOnSpeed    = 1000;
constexpr int    kPowerOnForceG   = 1000;
constexpr int    kTemperatureDegC = 32;

// Byte addresses.
constexpr std::uint16_t kClearError = 1004;
constexpr std::uint16_t kAngleSet   = 1486;
constexpr std::uint16_t kForceSet   = 1498;
constexpr std::uint16_t kSpeedSet   = 1522;
constexpr std::uint16_t kAngleAct   = 1546;
constexpr std::uint16_t kForceAct   = 1582;
constexpr std::uint16_t kCurrent    = 1594;
constexpr std::uint16_t kError      = 1606;
constexpr std::uint16_t kStatus     = 1612;
constexpr std::uint16_t kTemp       = 1618;

constexpr std::uint8_t kStatusReleasing = 0;
constexpr std::uint8_t kStatusGrasping  = 1;
constexpr std::uint8_t kStatusReached   = 2;
constexpr std::uint8_t kStatusForce     = 3;
constexpr std::uint8_t kStatusLocked    = 6;
constexpr std::uint8_t kErrorLocked     = 0x01;

struct Actuator
{
    int    qpos_adr{ -1 };
    int    dof_adr{ -1 };
    double command{ 0.0 };  // where the screw is driven to, ANGLE units, slewed toward the target
    int    target{ 0 };     // the ANGLE_SET last acted on
    bool   force_stopped{ false };
    bool   locked{ false };
    double stalled_s{ 0.0 };
    double tau{ 0.0 };
};

struct Hand
{
    std::string side;
    char        prefix{ 'L' };
    std::string port;
    int         master_fd{ -1 };
    int         slave_fd{ -1 };  // held open, so the line stays up while the driver is away
    std::array<Actuator, kActuators>       act;
    std::array<std::uint8_t, 2048>         regs{};
    std::vector<std::uint8_t>              pending;
    std::vector<std::uint8_t>              reply;
    std::chrono::steady_clock::time_point  reply_at{};
};

struct State
{
    std::thread         thread;
    std::atomic<bool>   running{ false };
    std::array<Hand, 2> hands;
};

State& state()
{
    // Leaked on purpose, as in dex3_handler: the physics thread ends in exit(0).
    static State* s = new State();
    return *s;
}

int word(const Hand& hand, std::uint16_t address)
{
    return static_cast<std::int16_t>(hand.regs[address] | (hand.regs[address + 1] << 8));
}

void setWord(Hand& hand, std::uint16_t address, int value)
{
    const auto w            = static_cast<std::uint16_t>(static_cast<std::int16_t>(value));
    hand.regs[address]      = static_cast<std::uint8_t>(w & 0xFF);
    hand.regs[address + 1]  = static_cast<std::uint8_t>(w >> 8);
}

double angleOf(std::size_t i, double q)
{
    return 1000.0 * (q - kClosedRad[i]) / (kOpenRad[i] - kClosedRad[i]);
}

double jointOf(std::size_t i, double angle)
{
    return kClosedRad[i] + angle / 1000.0 * (kOpenRad[i] - kClosedRad[i]);
}

std::uint8_t byteSum(const std::vector<std::uint8_t>& frame, std::size_t end)
{
    std::uint32_t sum = 0;
    for (std::size_t i = 2; i < end; ++i)
    {
        sum += frame[i];
    }
    return static_cast<std::uint8_t>(sum & 0xFF);
}

std::uint16_t crc16(const std::uint8_t* bytes, std::size_t size)
{
    std::uint16_t crc = 0xFFFF;
    for (std::size_t i = 0; i < size; ++i)
    {
        crc ^= bytes[i];
        for (int bit = 0; bit < 8; ++bit)
        {
            crc = (crc & 1) ? static_cast<std::uint16_t>((crc >> 1) ^ 0xA001) :
                              static_cast<std::uint16_t>(crc >> 1);
        }
    }
    return crc;
}

bool resolve(const mjModel* model, Hand& hand)
{
    for (std::size_t i = 0; i < kActuators; ++i)
    {
        const std::string name = std::string(1, hand.prefix) + "_" + kJointSuffixes[i];
        const int         id   = mj_name2id(model, mjOBJ_JOINT, name.c_str());
        if (id < 0)
        {
            return false;
        }
        hand.act[i].qpos_adr = model->jnt_qposadr[id];
        hand.act[i].dof_adr  = model->jnt_dofadr[id];
    }
    return true;
}

// The paths from GROVE_G1_INSPIRE_PORTS, "left=<path>,right=<path>".
bool readPorts(State& s)
{
    const char* env = std::getenv("GROVE_G1_INSPIRE_PORTS");
    if (env == nullptr)
    {
        return false;
    }
    std::string spec = env;
    for (auto& hand : s.hands)
    {
        const std::string key = hand.side + "=";
        const auto        at  = spec.find(key);
        if (at == std::string::npos)
        {
            return false;
        }
        const auto end = spec.find(',', at);
        hand.port = spec.substr(at + key.size(), end == std::string::npos ? std::string::npos :
                                                                            end - at - key.size());
    }
    return !s.hands[0].port.empty() && !s.hands[1].port.empty();
}

bool openLine(Hand& hand)
{
    hand.master_fd = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (hand.master_fd < 0 || grantpt(hand.master_fd) != 0 || unlockpt(hand.master_fd) != 0)
    {
        return false;
    }
    char name[128] = {};
    if (ptsname_r(hand.master_fd, name, sizeof(name)) != 0)
    {
        return false;
    }
    hand.slave_fd = open(name, O_RDWR | O_NOCTTY);
    // Replaces a link a crashed run left behind, but never a real file.
    struct stat existing{};
    if (lstat(hand.port.c_str(), &existing) == 0)
    {
        if (!S_ISLNK(existing.st_mode))
        {
            std::fprintf(stderr, "[grove_g1] %s exists and is not a link; not replacing it\n",
                         hand.port.c_str());
            return false;
        }
        unlink(hand.port.c_str());
    }
    return hand.slave_fd >= 0 && symlink(name, hand.port.c_str()) == 0;
}

void closeLine(Hand& hand)
{
    if (!hand.port.empty())
    {
        unlink(hand.port.c_str());
    }
    if (hand.slave_fd >= 0)
    {
        close(hand.slave_fd);
    }
    if (hand.master_fd >= 0)
    {
        close(hand.master_fd);
    }
    hand.slave_fd = hand.master_fd = -1;
}

void clearErrors(Hand& hand)
{
    for (std::size_t i = 0; i < kActuators; ++i)
    {
        hand.act[i].locked    = false;
        hand.act[i].stalled_s = 0.0;
        hand.regs[kError + i] = 0;
    }
}

void store(Hand& hand, std::uint16_t address, const std::uint8_t* bytes, std::size_t size)
{
    if (address + size > hand.regs.size())
    {
        return;
    }
    std::copy(bytes, bytes + size, hand.regs.begin() + address);
    if (address == kClearError && hand.regs[kClearError] == 1)
    {
        clearErrors(hand);
        hand.regs[kClearError] = 0;
    }
}

// Answers the request at the front of hand.pending; returns the bytes it used, or 0 for "more".
std::size_t answer(Hand& hand, std::vector<std::uint8_t>& out)
{
    const std::vector<std::uint8_t>& in = hand.pending;
    if (in.empty())
    {
        return 0;
    }
    if (in[0] == 0xEB)
    {
        if (in.size() < 4 || in.size() < 5U + in[3])
        {
            return 0;
        }
        const std::size_t total = 5U + in[3];
        if (in[1] != 0x90 || in[total - 1] != byteSum(in, total - 1) || in[2] != 1)
        {
            return total;
        }
        const std::uint8_t  command = in[4];
        const std::uint16_t address = static_cast<std::uint16_t>(in[5] | (in[6] << 8));
        out = { 0x90, 0xEB, in[2] };
        if (command == 0x11 && address + in[7] <= hand.regs.size())
        {
            out.insert(out.end(), { static_cast<std::uint8_t>(3 + in[7]), command, in[5], in[6] });
            out.insert(out.end(), hand.regs.begin() + address, hand.regs.begin() + address + in[7]);
        }
        else if (command == 0x12)
        {
            store(hand, address, in.data() + 7, total - 8);
            out.insert(out.end(), { 4, command, in[5], in[6], 1 });
        }
        else
        {
            out.clear();
            return total;
        }
        out.push_back(byteSum(out, out.size()));
        return total;
    }

    if (in.size() < 7)
    {
        return 0;
    }
    const std::uint8_t function = in[1];
    if (function != 0x03 && function != 0x10)
    {
        return 1;  // not a request: resynchronise on the next byte
    }
    const std::size_t total = function == 0x03 ? 8 : 9U + in[6];
    if (in.size() < total)
    {
        return 0;
    }
    const std::uint16_t crc = crc16(in.data(), total - 2);
    if (in[0] != 1 || in[total - 2] != (crc & 0xFF) || in[total - 1] != (crc >> 8))
    {
        return total;
    }
    const std::uint16_t address = static_cast<std::uint16_t>((in[2] << 8) | in[3]);
    const std::size_t   length  = 2U * static_cast<std::size_t>((in[4] << 8) | in[5]);
    if (address + length > hand.regs.size())
    {
        return total;
    }
    out = { in[0], function };
    if (function == 0x03)
    {
        out.push_back(static_cast<std::uint8_t>(length));
        for (std::size_t i = 0; i < length; i += 2)
        {
            // Each register goes high byte first; the hand keeps values low byte first.
            out.push_back(hand.regs[address + i + 1]);
            out.push_back(hand.regs[address + i]);
        }
    }
    else
    {
        std::vector<std::uint8_t> bytes(length);
        for (std::size_t i = 0; i < length; i += 2)
        {
            bytes[i]     = in[7 + i + 1];
            bytes[i + 1] = in[7 + i];
        }
        store(hand, address, bytes.data(), bytes.size());
        out.insert(out.end(), { in[2], in[3], in[4], in[5] });
    }
    const std::uint16_t out_crc = crc16(out.data(), out.size());
    out.push_back(static_cast<std::uint8_t>(out_crc & 0xFF));
    out.push_back(static_cast<std::uint8_t>(out_crc >> 8));
    return total;
}

// Reads what the driver sent and queues the replies, each sent when the real line would have
// carried the request and the reply at 115200 baud.
void serve(Hand& hand)
{
    const auto now = std::chrono::steady_clock::now();
    if (!hand.reply.empty() && now >= hand.reply_at)
    {
        if (write(hand.master_fd, hand.reply.data(), hand.reply.size()) < 0)
        {
            // The driver is away: drop it, as an unread reply on a real line is lost too.
        }
        hand.reply.clear();
    }
    std::uint8_t  chunk[256];
    const ssize_t n = read(hand.master_fd, chunk, sizeof(chunk));
    if (n > 0)
    {
        hand.pending.insert(hand.pending.end(), chunk, chunk + n);
    }
    while (hand.reply.empty() && !hand.pending.empty())
    {
        std::vector<std::uint8_t> out;
        const std::size_t         used = answer(hand, out);
        if (used == 0)
        {
            break;
        }
        const double wire_s = static_cast<double>(used + out.size()) * 10.0 / kBaud;
        hand.pending.erase(hand.pending.begin(), hand.pending.begin() + static_cast<long>(used));
        if (!out.empty())
        {
            hand.reply    = std::move(out);
            hand.reply_at = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                      std::chrono::duration<double>(wire_s + kTurnaroundS));
        }
    }
}

void drive(const mjModel* model, mjData* data, Hand& hand, double dt)
{
    for (std::size_t i = 0; i < kActuators; ++i)
    {
        Actuator&    a         = hand.act[i];
        const double q         = data->qpos[a.qpos_adr];
        const double dq        = data->qvel[a.dof_adr];
        const double angle     = std::clamp(angleOf(i, q), 0.0, 1000.0);
        const double closing   = kClosedRad[i] > kOpenRad[i] ? 1.0 : -1.0;
        const double tau_rated = kRatedForceG[i] * kNewtonsPerGram * kLeverArmM[i];
        // The force sensor feels what the digit presses on, not the joint's own damping and
        // friction, which the model has in place of the screw's drag.
        const double drag = (model->dof_damping[a.dof_adr] * dq) +
                            (model->dof_frictionloss[a.dof_adr] * ((dq > 0.0) - (dq < 0.0)));
        const double pressing_g = closing * (a.tau - drag) / (kNewtonsPerGram * kLeverArmM[i]);

        // A new target releases a force stop; -1 leaves the current one.
        const int written = word(hand, kAngleSet + 2 * i);
        if (written != -1 && written != a.target)
        {
            a.target        = std::clamp(written, 0, 1000);
            a.force_stopped = false;
        }
        const bool closing_now = a.target < a.command;
        const double force_set_g = word(hand, kForceSet + 2 * i);
        if (!a.locked && !a.force_stopped && closing_now && std::abs(dq) < kBlockedRadS &&
            pressing_g >= force_set_g)
        {
            // Stops where the finger is: the screw holds there, so the object stays pressed with
            // the force it was stopped at, which the loop keeps by aiming that far into it.
            const double preload_rad = force_set_g * kNewtonsPerGram * kLeverArmM[i] / kKp;
            a.force_stopped          = true;
            a.command = angle - (preload_rad * 1000.0 / std::abs(kOpenRad[i] - kClosedRad[i]));
        }
        if (!a.locked && !a.force_stopped)
        {
            const double rate = std::clamp(word(hand, kSpeedSet + 2 * i), 0, 1000) / kFullStrokeS;
            const double step = rate * dt;
            a.command += std::clamp(static_cast<double>(a.target) - a.command, -step, step);
            // A blocked screw does not advance: the command leads the finger by no more than the
            // loop's give at rated force.
            const double lead = tau_rated / kKp * 1000.0 / std::abs(kOpenRad[i] - kClosedRad[i]);
            a.command         = std::clamp(a.command, angle - lead, angle + lead);
        }

        const bool   stopped = a.locked || a.force_stopped || std::abs(a.target - a.command) < 0.5;
        const double limit   = tau_rated * (stopped ? kHoldMargin : 1.0);
        a.tau = std::clamp(kKp * (jointOf(i, a.command) - q) - kKd * dq, -limit, limit);
        data->qfrc_applied[a.dof_adr] = a.tau;

        a.stalled_s = std::abs(a.tau) >= 0.99 * tau_rated ? a.stalled_s + dt : 0.0;
        if (!a.locked && a.stalled_s > kLockAfterS)
        {
            a.locked              = true;
            a.command             = angle;
            hand.regs[kError + i] = kErrorLocked;
        }

        setWord(hand, kAngleAct + 2 * i, static_cast<int>(std::lround(angle)));
        setWord(hand, kForceAct + 2 * i, static_cast<int>(std::lround(pressing_g)));
        setWord(hand, kCurrent + 2 * i,
                static_cast<int>(std::lround(100.0 + 900.0 * std::abs(a.tau) / tau_rated)));
        hand.regs[kStatus + i] = a.locked          ? kStatusLocked :
                                 a.force_stopped   ? kStatusForce :
                                 stopped           ? kStatusReached :
                                 closing_now       ? kStatusGrasping :
                                                     kStatusReleasing;
        hand.regs[kTemp + i] = kTemperatureDegC;
    }
}

void run(mjModel** model, mjData** data)
{
    auto& s = state();
    while (s.running && (*model == nullptr || *data == nullptr))
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!s.running)
    {
        return;
    }
    for (auto& hand : s.hands)
    {
        if (!resolve(*model, hand))
        {
            s.running = false;  // a Dex3 or hand-less model: nothing to answer for
            return;
        }
    }
    if (!readPorts(s))
    {
        std::fprintf(stderr,
                     "[grove_g1] Inspire hands in the model but GROVE_G1_INSPIRE_PORTS is unset; "
                     "hand emulation is OFF\n");
        s.running = false;
        return;
    }
    for (auto& hand : s.hands)
    {
        if (!openLine(hand))
        {
            std::fprintf(stderr, "[grove_g1] no pseudo-terminal at %s; Inspire hands are OFF\n",
                         hand.port.c_str());
            for (auto& other : s.hands)
            {
                closeLine(other);
            }
            s.running = false;
            return;
        }
        // Power-on: holding where the model spawned, at full speed, with a 1000 g force limit.
        for (std::size_t i = 0; i < kActuators; ++i)
        {
            const double angle =
                std::clamp(angleOf(i, (*data)->qpos[hand.act[i].qpos_adr]), 0.0, 1000.0);
            hand.act[i].command = angle;
            hand.act[i].target  = static_cast<int>(std::lround(angle));
            setWord(hand, kAngleSet + 2 * i, hand.act[i].target);
            setWord(hand, kSpeedSet + 2 * i, kPowerOnSpeed);
            setWord(hand, kForceSet + 2 * i, kPowerOnForceG);
        }
    }
    std::fprintf(stderr, "[grove_g1] Inspire hands answering on %s and %s\n",
                 s.hands[0].port.c_str(), s.hands[1].port.c_str());

    const auto period = std::chrono::nanoseconds(std::chrono::seconds(1)) / kTickHz;
    auto       next   = std::chrono::steady_clock::now();
    while (s.running)
    {
        // No sim.mtx, as in dex3_handler: independent doubles, and a torn read costs one tick.
        mjModel* m = *model;
        mjData*  d = *data;
        if (m != nullptr && d != nullptr)
        {
            for (auto& hand : s.hands)
            {
                serve(hand);
                drive(m, d, hand, 1.0 / kTickHz);
            }
        }
        next += period;
        std::this_thread::sleep_until(next);
    }

    // qfrc_applied persists between steps, so a torque left here would act forever.
    if (*data != nullptr)
    {
        for (const auto& hand : s.hands)
        {
            for (const auto& a : hand.act)
            {
                (*data)->qfrc_applied[a.dof_adr] = 0.0;
            }
        }
    }
    for (auto& hand : s.hands)
    {
        closeLine(hand);
    }
}

}  // namespace

void StartInspireHandler(mjModel** model, mjData** data)
{
    auto& s = state();
    if (s.running)
    {
        return;
    }
    s.hands[0].side   = "left";
    s.hands[0].prefix = 'L';
    s.hands[1].side   = "right";
    s.hands[1].prefix = 'R';
    s.running         = true;
    s.thread          = std::thread([model, data] { run(model, data); });
}

void StopInspireHandler()
{
    auto& s   = state();
    s.running = false;
    if (s.thread.joinable())
    {
        s.thread.join();
    }
}

}  // namespace grove_g1
