# The grove branch

[grove-g1](https://github.com/Adyansh04/grove-g1)'s changes to unitree_mujoco, on top of upstream
`ae6a840`. grove-g1 pins this branch as the submodule `workspace/vendor/unitree_mujoco` and builds
the `simulate` app from it into its development image.

Sensors, the hands and the grasp weld need the live `mjModel`/`mjData`, which are local to
the simulator's process and on no DDS topic, so they run inside it. A second process mirroring
joint state would be a second physics instance, and the real G1 has no odometry topic to drive
one.

| Commit | Does |
|---|---|
| Run grove-g1's sensor sampler in the simulator | `sensor_publisher.cc` samples the LiDAR, the depth cameras, the Mid360 IMU, the base state and the object poses for grove-g1's relay. |
| Let the SDK honour CYCLONEDDS_URI | unitree_sdk2 reads the container's DDS config instead of building its own. |
| Give the G1 its Dex3-1 hands | The hand bodies, unactuated, at the stock model's hand mass. |
| Run grove-g1's Dex3 hand handler | `dex3_handler.cc` drives the fingers over DDS, as the real hands are. |
| Weld grasped objects to the palm in simulation | `grasp_weld.cc`, for the scenes that carry objects while walking. |
| Give the Mid360 its IMU | The IMU inside the LiDAR's housing, which FAST-LIO needs. |
| Hold the robot until the control stack drives the motors | Each motor holds its spawn angle until its first command. |
| Give the Dex3 palm and fingers contact geometry | Grasps by contact rather than by a weld. |
| Apply a scene's startup hold angles at load | `startup_pose.cc` spawns a scene's postures before the first step. |
| Stamp each camera frame with its own instant | A depth frame carries the sim time its camera pose was copied at, not the earlier LiDAR sweep's. |
| Refuse cameras the relay cannot use | A camera name too long for the frame header is disabled loudly, and an MJCF camera with no resolution renders at 848x480 rather than MuJoCo's 1x1. |
| Give the G1 Inspire RH56DFQ hands | `g1_29dof_inspire.xml`, the G1 with Unitree's model of the Inspire DFQ hands instead of the Dex3, and its meshes. Generated from grove-g1's URDF by its `g1_bringup/tools/inspire_hand_mjcf.py`; rerun that rather than editing the file. |
| Answer the Inspire hands' RS485 lines | `inspire_handler.cc` serves each hand's serial line on a pseudo-terminal, in the hand's own frame and in Modbus RTU, and moves the fingers as the hand's firmware does. |
| Count a capsule's end caps in its height | A standing capsule prop is reported at its full height, not its straight part's. |
| Monitor what the hands touch | `contact_monitor.cc`, called after every physics step: each hand contact's peak force by what it met, and how centred each grasp sits, to stderr above 5 N and to `GROVE_G1_CONTACT_LOG`. Changes nothing in the simulation. |

## Rules

- **All real logic goes in its own files under `simulate/src/`.** Upstream's files get the smallest
  edit that registers them, so a rebase onto upstream touches a few lines.
- **`simulate/src/unitree_sdk2_bridge.h` is edited only by the startup hold.** Its `run()` is a
  1 kHz callback that writes `mj_data_->ctrl[]`, the actuator command path, so nothing in it may
  block. The startup hold is the exception because it has to replace that command until a
  controller drives the motor.
- **Nothing writes `mjData` state from the bridge thread except `ctrl`.** Other state is changed
  under `sim.mtx` or from `mjcb_control`; flipping `eq_active` mid-step exits the simulator.

## The Inspire hands

`g1_29dof_inspire.xml` is the same robot with each Dex3 subtree replaced by an Inspire hand: six
actuated joints and six coupled ones, which follow through joint equalities as the URDF's
`<mimic>` tags say. grove-g1 stages it in place of `g1_29dof.xml` with `hand_model:=inspire`.

`inspire_handler.cc` links a pseudo-terminal per hand at the paths `GROVE_G1_INSPIRE_PORTS` names
(`left=<path>,right=<path>`) and answers there as the hand does on RS485, so grove-g1's driver
runs unchanged. Each actuator moves toward its `ANGLE_SET` at its `SPEED_SET`, stops closing once
its finger presses at `FORCE_SET`, and holds where it stopped, as the lead screws do; one pushing
at its rated force for 2 s latches a locked-rotor error until `CLEAR_ERROR`. Replies are delayed by
the time the line would take at 115200 baud. The angle mapping mirrors grove-g1's
`inspire_params.yaml`, and grove-g1's `test_inspire_sim` holds the two equal.

## The contact firewall

The Dex3 and Inspire collision capsules are `contype="2" conaffinity="2"` and every other geom in the robot and
the scenes is `1/1`. `(2 & 1) || (1 & 2)` is zero, so the hand collides with nothing until a scene
opts a prop in by setting bit 1. Only grove-g1's two manipulation scenes set their props and table
to `3/3`; every other world keeps the contact count the walking policy was trained with.

The capsules are `group="3"`, outside the groups 0-2 that the viewer and the rendered camera
draw, so perception still sees only the meshes. `priority="1"` makes the finger friction and
solver parameters win outright instead of averaging with the object's, so no prop needs contact
tuning to be pickable.

## Scene hooks

- `pelvis_startup_hold`: a weld released once every motor has been driven for half a second.
- `startup_hold_<joint>`: a single-joint equality holding a spawn angle. The angle is written into
  `qpos` at load, becomes the motor's hold target, and the equality is released on its first
  tick. The manipulation scenes use these to spawn with the hands clear of the table; without them
  the right hand spawns inside a prop and throws it across the room.
- `grasp_*`: body welds, palm first, that `grasp_weld.cc` engages when the thumb and one other
  digit touch the object within `capture_radius_m` of the palm, and releases after
  `release_after_s` without that contact.

The viewer's reset restores the startup holds, and nothing releases them again.

## Taking upstream changes

```bash
git remote add upstream https://github.com/unitreerobotics/unitree_mujoco.git
git fetch upstream && git rebase upstream/main
```

Build the simulator and run grove-g1's simulator tests before pushing, then move grove-g1's
submodule to the new commit.
