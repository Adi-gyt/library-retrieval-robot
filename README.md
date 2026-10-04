# Library Retrieval Robot (Simulation Track)

Simulation of the arm and gripper subsystem of an autonomous library book-retrieval robot. A 6-DOF arm with a wrist-mounted depth camera finds boxes by ArUco marker, picks them up, and places them on a pedestal.

Stack: ROS 2 Humble, Gazebo Classic 11, MoveIt 2 (OMPL, KDL), `gazebo_ros2_control`, OpenCV ArUco.

## Current status

The full pick-and-place demo runs end to end from a cold start, in freshly opened terminals.

- Three marker-tagged boxes (IDs 1, 2, 3) spawn on table A.
- The default demo moves boxes 2 and 3 to the two pedestal slots, in that order. Box 1 is spawned but not moved by default.
- Two boxes take about 100 seconds.

### Last verified result

Final box poses read from Gazebo after the run (world frame, metres):

| Box | Target slot (x, y, z) | Final position (x, y, z) | Error |
|-----|-----------------------|--------------------------|-------|
| 2 | 0.600, 0.450, 0.490 | 0.5995, 0.4503, 0.4895 | under 1 mm |
| 3 | 0.500, 0.450, 0.490 | 0.4995, 0.4502, 0.4895 | under 1 mm |

Both detach calls reported `ok`. The target z is the pedestal top (0.45) plus half a box (0.04).

## How it works

One Python client node (`scripts/step11_demo.py`, built on `step7_marker_reach.py`, `step8_grasp.py` and `step10_stack.py`) drives the whole task. It talks to MoveIt, the ros2_control controllers and Gazebo.

**Scene.** Table A at x = 1.14 holds three 8 cm boxes at x = 1.00, y = +0.20 / 0.00 / -0.20. A pedestal (25 x 25 x 45 cm) at (0.55, 0.45) has slots at box-centre x = 0.60 and 0.50.

**Per box:**

1. **Scan.** MoveIt (OMPL) moves the hand to a scan pose in front of the box.
2. **Detect and localise.** OpenCV detects the box's ArUco marker (4x4_50) in the camera image. Depth is the median of an 11 x 11 patch around the marker centre, back-projected with the camera intrinsics, then transformed to `base_footprint` with tf2. The result is the median of several frames and is rejected if it falls outside the expected zone.
3. **Approach.** The gripper opens and the hand moves to a pre-grasp pose 10 cm behind the box, then pushes in along a straight Cartesian line (`/compute_cartesian_path`). Paths are refused if the planner jumps between IK branches.
4. **Grasp.** The fingers close under effort control. The script checks that the fingers stopped on an object and that the box is still upright and aligned, then attaches the box to the hand.
5. **Carry and place.** The hand lifts to carry height, moves along waypoints above the pedestal slot, descends, waits, detaches, opens the gripper and lifts clear.

**Attachment.** The attach and detach services come from the Gazebo model attachment plugin (`boeing_gazebo_model_attachment_plugin_msgs`). It creates a fixed joint between the hand and the box. A local patch (`ros2_ws/src/gazebo_attach_plugin_local_changes.patch`) disables the plugin's snap-to-pose teleport, so the box keeps its position in the hand. Service calls are retried up to three times.

## First-time setup

```bash
cd ~/projects/library-retrieval-robot/ros2_ws
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-gazebo-ros2-control
colcon build --symlink-install
cp ../tools/gz_env.sh ~/gz_env.sh
```

The tools scripts expect the repo at `~/projects/library-retrieval-robot` and the environment file at `~/gz_env.sh`.

## Running the demo

Each step works in a freshly opened terminal. The scripts source ROS 2 Humble, the workspace overlay and `~/gz_env.sh` themselves.

1. Start the simulation (leave this terminal open):

   ```bash
   tools/start_sim.sh
   ```

2. In a second terminal, wait for MoveIt and the gripper controller:

   ```bash
   tools/ready.sh
   ```

   Wait for the line starting with `READY`.

3. Run the demo:

   ```bash
   tools/run_demo.sh
   ```

   The box order defaults to `2,3` and can be changed, for example `ORDER=3,1 tools/run_demo.sh`. The pedestal has two slots, so the order can hold at most two boxes unless `SLOTS_X` in `step11_demo.py` is extended.

4. To reset the boxes for another run without restarting the sim:

   ```bash
   tools/reset_demo.sh
   ```

`start_sim.sh` kills any earlier Gazebo and MoveIt processes before launching.

## Repository layout

- `tools/` : run scripts (`start_sim.sh`, `ready.sh`, `run_demo.sh`, `reset_demo.sh`, `reset_stack.sh`) and `gz_env.sh`
- `ros2_ws/src/library_bot_description/` : URDF (arm, gripper, D405 camera), world, box, table and pedestal models, and the demo scripts
- `ros2_ws/src/library_bot_moveit_config/` : SRDF, controller and kinematics config, and the Gazebo + MoveIt launch file
- `ros2_ws/src/gazebo_model_attachment_plugin/` : attachment plugin and its service definitions
- `docs/`, `PROJECT_HANDOFF.md` : project notes (the handoff predates steps 7 to 11)

## Troubleshooting

- **"The passed service type is invalid" or "Unknown package 'boeing_gazebo_model_attachment_plugin_msgs'"**: the workspace overlay was not sourced in that terminal. The tools scripts handle this; if you run Python scripts by hand, run `source ros2_ws/install/setup.bash` first.
- **Demo starts before the sim is ready**: wait for `READY` from `tools/ready.sh`.
- **A new launch fails or hangs**: an orphaned `gzserver` may still be running. `tools/start_sim.sh` clears it; by hand use `pkill -9 gzserver gzclient`.

## Known limitations

- The grasp is held by a fixed joint created after a finger and box check, not by friction alone.
- The grasp check and result logging read box poses from Gazebo directly (`gz model`), which would not exist on real hardware.
- Joint limits are estimates, and link geometry is primitive cylinders.
- The camera link has no collision geometry, so MoveIt cannot see it.
- Camera clip distances (0.07 to 0.5 m) are marked temporary in `d405_camera.xacro`.

## Next steps

- Add a third pedestal slot so all three boxes can be moved.
- Base-to-arm handoff signal (see `PROJECT_HANDOFF.md`).
- FPGA CORDIC IK solver, separate from this repo's KDL-based IK.
