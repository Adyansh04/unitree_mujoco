#ifndef GROVE_G1_INSPIRE_HANDLER_H_
#define GROVE_G1_INSPIRE_HANDLER_H_

// Answers each Inspire RH56DFQ hand's RS485 line inside unitree_mujoco, so grove-g1's
// ros2_control component (g1_inspire_hand) drives the simulated fingers through the same frames,
// registers and serial code it uses on the robot. Each hand is a pseudo-terminal, linked at the path
// GROVE_G1_INSPIRE_PORTS names ("left=<path>,right=<path>"), and answers both the hand's own EB 90
// frame and Modbus RTU, told apart by their first byte.
//
// The fingers move as the hand's firmware moves them: each actuator runs toward its ANGLE_SET at its
// SPEED_SET, stops closing once its finger presses at FORCE_SET, and holds where it stopped, as the
// lead screws do. A finger pushing at its rated force for two seconds latches a locked-rotor error
// until CLEAR_ERROR. The six coupled joints follow through the model's joint equalities.
//
// Driven through qfrc_applied, like the Dex3: the SDK bridge sizes itself from nu. A no-op on models
// without the Inspire joints, or without GROVE_G1_INSPIRE_PORTS.
//
// Lives in its own file so the edits to upstream's sources stay a few lines; see GROVE.md.

#include <mujoco/mujoco.h>

namespace grove_g1
{

// Starts the hand thread. Non-blocking. model/data are the addresses of unitree_mujoco's globals,
// which a model dropped into the viewer reassigns.
void StartInspireHandler(mjModel** model, mjData** data);

// Stops the hand thread, blocking until it exits, zeroes its torques and removes the port links.
// Safe when idle. Call before mj_deleteModel on any path that replaces the model: the cached joint
// addresses outlive it. One-way, like StopDex3Handler.
void StopInspireHandler();

}  // namespace grove_g1

#endif  // GROVE_G1_INSPIRE_HANDLER_H_
