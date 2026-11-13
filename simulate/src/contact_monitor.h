#ifndef GROVE_G1_CONTACT_MONITOR_H_
#define GROVE_G1_CONTACT_MONITOR_H_

// Watches what the hands touch, every physics step, so grasps can be tuned until the fingers never
// strike the table: SIMULATION ONLY, it changes nothing in the simulation.
//
// A hand is everything from each wrist_roll_link out, so it covers the Dex3 and the Inspire hands
// alike. Each contact a hand makes is classed by what it meets: the environment (the world body
// and every body without a free joint, such as tables), a prop (a free body), or the robot itself.
//
// Every step's contact forces are folded into episodes, one per hand link and the body it touches,
// closed once they have been apart for 50 ms. An episode against the environment or the robot that
// peaked above 5 N, where the DFQ finger is rated 4 N, is printed to stderr. With
// GROVE_G1_CONTACT_LOG naming a file, each episode is appended there as a JSON line, with the
// wall-clock time of its start and its peak to line it up with the stack's logs, and so is
// each grasp: the first step a hand's thumb and another digit both touch a prop, with how far the
// centroid of the hand's contacts sits from the prop's centre.
//
// Lives in its own file so the edits to upstream's sources stay a few lines; see GROVE.md.

#include <mujoco/mujoco.h>

namespace grove_g1
{

// Folds this step's contacts in. Call right after each mj_step, under the physics lock, so every
// step's peak is seen. Cheap: a lookup per contact. A new model is picked up by its address.
void MonitorContacts(const mjModel* model, const mjData* data);

}  // namespace grove_g1

#endif  // GROVE_G1_CONTACT_MONITOR_H_
