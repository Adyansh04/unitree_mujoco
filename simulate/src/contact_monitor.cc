#include "contact_monitor.h"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace grove_g1
{
namespace
{

enum class Kind
{
    kOther,
    kHand,
    kEnvironment,
    kProp,
    kRobot,
};

const char* kindName(Kind kind)
{
    switch (kind)
    {
        case Kind::kHand:
            return "hand";
        case Kind::kEnvironment:
            return "environment";
        case Kind::kProp:
            return "prop";
        case Kind::kRobot:
            return "robot";
        default:
            return "other";
    }
}

// N. Lighter contacts are not episodes: resting contact noise.
constexpr double kRecordN = 0.5;
// N. The DFQ finger is rated 4 N: past this the hand is pushing on something hard.
constexpr double kHardN = 5.0;
// s apart before an episode closes, so a contact chattering at the step rate is one episode.
constexpr double kGapS = 0.05;
// s without touching before a hand's grasp of a prop can be logged again.
constexpr double kRegraspS = 0.5;

constexpr std::array<const char*, 5> kDigits = { "thumb", "index", "middle", "ring", "pinky" };

struct Geom
{
    Kind kind{ Kind::kOther };
    int  side{ -1 };  // 0 left, 1 right, for a hand geom
};

struct Episode
{
    double t0{ 0.0 };
    double last{ 0.0 };
    double peak_n{ 0.0 };
    // Wall-clock seconds at its start and at its peak, to line it up with the stack's own logs.
    double wall0{ 0.0 };
    double wall_peak{ 0.0 };
    int    side{ -1 };
    Kind   other_kind{ Kind::kOther };
};

struct Touch
{
    std::set<std::string> digits;
    double                sum[3]{ 0.0, 0.0, 0.0 };
    int                   count{ 0 };
};

struct State
{
    const mjModel*        model{ nullptr };
    std::vector<Geom>     geoms;
    std::array<int, 2>    palm{ -1, -1 };
    std::map<std::pair<int, int>, Episode> episodes;     // (hand body, other body)
    std::map<std::pair<int, int>, double>  grasp_seen;   // (side, prop body) -> last touch
    FILE*                 log{ nullptr };
    bool                  log_tried{ false };
};

State& state()
{
    // Leaked on purpose, as in dex3_handler: the physics thread ends in exit(0).
    static State* s = new State();
    return *s;
}

double wallNow()
{
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch())
        .count();
}

bool inSubtree(const mjModel* m, int body, int root)
{
    while (body > 0)
    {
        if (body == root)
        {
            return true;
        }
        body = m->body_parentid[body];
    }
    return false;
}

int topAncestor(const mjModel* m, int body)
{
    while (body > 0 && m->body_parentid[body] > 0)
    {
        body = m->body_parentid[body];
    }
    return body;
}

bool hasFreeJoint(const mjModel* m, int body)
{
    return m->body_jntnum[body] > 0 && m->jnt_type[m->body_jntadr[body]] == mjJNT_FREE;
}

const char* bodyName(const mjModel* m, int body)
{
    const char* name = mj_id2name(m, mjOBJ_BODY, body);
    return name != nullptr ? name : "unnamed";
}

std::string digitOf(const mjModel* m, int body)
{
    const std::string name = bodyName(m, body);
    for (const char* digit : kDigits)
    {
        if (name.find(digit) != std::string::npos)
        {
            return digit;
        }
    }
    return name.find("palm") != std::string::npos || name.find("hand_base") != std::string::npos ?
               "palm" :
               "wrist";
}

void classify(const mjModel* m, State& s)
{
    s.model = m;
    s.episodes.clear();
    s.grasp_seen.clear();
    const std::array<const char*, 2> wrists = { "left_wrist_roll_link", "right_wrist_roll_link" };
    std::array<int, 2>               wrist{};
    for (int side = 0; side < 2; ++side)
    {
        wrist[side]  = mj_name2id(m, mjOBJ_BODY, wrists[side]);
        s.palm[side] = -1;
        for (int b = 1; b < m->nbody && wrist[side] > 0; ++b)
        {
            const std::string name = bodyName(m, b);
            if (inSubtree(m, b, wrist[side]) &&
                (name.find("hand_palm_link") != std::string::npos ||
                 name.find("hand_base_link") != std::string::npos))
            {
                s.palm[side] = b;
            }
        }
    }
    const int robot = mj_name2id(m, mjOBJ_BODY, "pelvis");
    s.geoms.assign(m->ngeom, Geom{});
    for (int g = 0; g < m->ngeom; ++g)
    {
        const int body = m->geom_bodyid[g];
        Geom&     geom = s.geoms[g];
        for (int side = 0; side < 2; ++side)
        {
            if (wrist[side] > 0 && inSubtree(m, body, wrist[side]))
            {
                geom.kind = Kind::kHand;
                geom.side = side;
            }
        }
        if (geom.kind == Kind::kHand)
        {
            continue;
        }
        if (robot > 0 && inSubtree(m, body, robot))
        {
            geom.kind = Kind::kRobot;
        }
        else if (body == 0 || !hasFreeJoint(m, topAncestor(m, body)))
        {
            geom.kind = Kind::kEnvironment;
        }
        else
        {
            geom.kind = Kind::kProp;
        }
    }
}

void openLog(State& s)
{
    s.log_tried      = true;
    const char* path = std::getenv("GROVE_G1_CONTACT_LOG");
    if (path == nullptr || *path == '\0')
    {
        return;
    }
    s.log = std::fopen(path, "a");
    std::fprintf(stderr, "[grove_g1] contact monitor %s %s\n",
                 s.log != nullptr ? "logging to" : "could not open", path);
}

void closeEpisode(const mjModel* m, State& s, const std::pair<int, int>& key, const Episode& e)
{
    const char* side = e.side == 0 ? "left" : "right";
    // A grip presses on its prop by design; only the rest is a strike.
    if (e.peak_n >= kHardN && e.other_kind != Kind::kProp)
    {
        std::fprintf(stderr,
                     "[grove_g1] HARD contact: %s hand %s on %s (%s), %.1f N peak over %.2f s\n",
                     side, bodyName(m, key.first), bodyName(m, key.second),
                     kindName(e.other_kind), e.peak_n, e.last - e.t0);
    }
    if (s.log != nullptr)
    {
        std::fprintf(s.log,
                     "{\"event\":\"contact\",\"t0\":%.4f,\"t1\":%.4f,\"side\":\"%s\","
                     "\"link\":\"%s\",\"digit\":\"%s\",\"other\":\"%s\",\"other_kind\":\"%s\","
                     "\"peak_n\":%.3f,\"wall0\":%.3f,\"wall_peak\":%.3f}\n",
                     e.t0, e.last, side, bodyName(m, key.first), digitOf(m, key.first).c_str(),
                     bodyName(m, key.second), kindName(e.other_kind), e.peak_n, e.wall0,
                     e.wall_peak);
        std::fflush(s.log);
    }
}

}  // namespace

void MonitorContacts(const mjModel* model, const mjData* data)
{
    State& s = state();
    if (!s.log_tried)
    {
        openLog(s);
    }
    if (model != s.model)
    {
        classify(model, s);
    }
    const double now = data->time;
    std::map<std::pair<int, int>, Touch> touches;  // (side, prop body)

    for (int i = 0; i < data->ncon; ++i)
    {
        const mjContact& contact = data->contact[i];
        if (contact.exclude != 0)
        {
            continue;
        }
        for (int k = 0; k < 2; ++k)
        {
            const int mine  = contact.geom[k];
            const int other = contact.geom[1 - k];
            if (mine < 0 || other < 0 || s.geoms[mine].kind != Kind::kHand)
            {
                continue;
            }
            const Kind other_kind =
                s.geoms[other].kind == Kind::kHand ? Kind::kRobot : s.geoms[other].kind;
            mjtNum force[6];
            mj_contactForce(model, data, i, force);
            const double normal_n   = std::abs(force[0]);
            const int    hand_body  = model->geom_bodyid[mine];
            const int    other_body = model->geom_bodyid[other];
            const int    side       = s.geoms[mine].side;

            if (other_kind == Kind::kProp)
            {
                Touch& touch = touches[{ side, other_body }];
                touch.digits.insert(digitOf(model, hand_body));
                for (int a = 0; a < 3; ++a)
                {
                    touch.sum[a] += contact.pos[a];
                }
                ++touch.count;
            }
            if (normal_n < kRecordN)
            {
                continue;
            }
            Episode& episode = s.episodes[{ hand_body, other_body }];
            if (episode.side < 0)
            {
                episode.t0         = now;
                episode.wall0      = wallNow();
                episode.side       = side;
                episode.other_kind = other_kind;
            }
            episode.last = now;
            if (normal_n > episode.peak_n)
            {
                episode.peak_n    = normal_n;
                episode.wall_peak = wallNow();
            }
        }
    }

    for (auto it = s.episodes.begin(); it != s.episodes.end();)
    {
        if (now - it->second.last > kGapS || now < it->second.last)
        {
            closeEpisode(model, s, it->first, it->second);
            it = s.episodes.erase(it);
        }
        else
        {
            ++it;
        }
    }

    for (const auto& [key, touch] : touches)
    {
        if (touch.digits.count("thumb") == 0 || touch.digits.size() < 2)
        {
            continue;  // not opposed: not a grasp yet
        }
        // Refreshed every step the grasp lasts, so one grasp logs once.
        const auto seen   = s.grasp_seen.find(key);
        const bool fresh  = seen == s.grasp_seen.end() || now - seen->second > kRegraspS;
        s.grasp_seen[key] = now;
        if (!fresh || s.log == nullptr)
        {
            continue;
        }
        // The centroid of the hand's contacts against the prop's centre of mass, in the palm's
        // frame: a grasp centred between the digits puts them on opposite sides of it.
        const int     palm = s.palm[key.first];
        const double* com  = data->xipos + 3 * key.second;
        double        world[3];
        for (int a = 0; a < 3; ++a)
        {
            world[a] = touch.sum[a] / touch.count - com[a];
        }
        double local[3] = { world[0], world[1], world[2] };
        if (palm > 0)
        {
            mju_mulMatTVec3(local, data->xmat + 9 * palm, world);
        }
        std::string digits;
        for (const std::string& digit : touch.digits)
        {
            digits += (digits.empty() ? "\"" : ",\"") + digit + "\"";
        }
        std::fprintf(s.log,
                     "{\"event\":\"grasp\",\"t\":%.4f,\"side\":\"%s\",\"object\":\"%s\","
                     "\"digits\":[%s],\"contacts\":%d,\"centroid_offset_mm\":[%.1f,%.1f,%.1f],"
                     "\"offset_mm\":%.1f,\"wall\":%.3f}\n",
                     now, key.first == 0 ? "left" : "right", bodyName(model, key.second),
                     digits.c_str(), touch.count, 1000.0 * local[0], 1000.0 * local[1],
                     1000.0 * local[2], 1000.0 * mju_norm3(world), wallNow());
        std::fflush(s.log);
    }
}

}  // namespace grove_g1
