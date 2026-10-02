# PROJECT HANDOFF: Library Retrieval Robot (Sim Track)

**Last updated:** October 2, 2026
**Reflects:** repo state at the last push, Sep 29, 2026 22:43 IST (commit `0bc7873`).
**Supersedes:** the Aug 8 handoff and the Aug 15 "Step 6 consolidated" handoff. Both are out of date.

---

## Read this first

- Sim Steps 1 to 6 are done.
- **Step 6.5** (Gazebo physics + MoveIt2 + D405 depth running together) was committed on Sep 29 as *"Working baseline: Gazebo + MoveIt2 + D405 depth verified"*.
  - Evidence in the repo: `launch.log` shows MoveIt2 planning with OMPL, then `arm_controller` inside Gazebo reporting `Goal reached, success!` and `Completed trajectory execution with status SUCCEEDED`. Two executions are in that log.
  - **Not recorded anywhere:** the actual depth readings from that check. Re-run `check_raw_value.py` once and write the numbers into the diary so the claim has evidence behind it.
- **Next task: Step 7.** Wire it together: detect marker, localize, transform, IK, move. Details in the "Next" section.
- In any new chat, upload `Library-Robot-Project-Plan_UPDATED.docx` first. It is the source of truth for objectives.

---

## Project context

Final-year autonomous library book-retrieval robot, 4-person team. Two decoupled subsystems: base navigation (nav2) and arm/gripper (D405 depth + ArUco + 6-DOF arm). My solo deliverables: the Gazebo/ROS2 simulation (this track) and a CORDIC-based IK solver on an FPGA (`fpga_ik_solver/`, separate, empty as of the last check). Timeline per plan doc: Aug 3 to end Nov 2026.

Machine: HP Victus 15 (Ryzen 7 5800H, 16GB RAM, RTX 3050 4GB), Ubuntu 22.04, ROS2 Humble, Gazebo Classic 11 (not the newer Gazebo/Ignition), MoveIt2, Nav2.

---

## Repo

`github.com/Adi-gyt/library-retrieval-robot`, local path `~/projects/library-retrieval-robot/`. Diary repo: `github.com/Adi-gyt/bot-sim-log` (narrative logs, not code).

Commits (newest first):

| Commit | Date | Message |
|---|---|---|
| `0bc7873` | 2026-09-29 | Ignore build, install and log dirs |
| `f7919bd` | 2026-09-29 | Working baseline: Gazebo + MoveIt2 + D405 depth verified |
| `4dd2796` | 2026-08-15 | Step 6 complete: DH table, joint limits, MoveIt2/KDL working (3 poses) |
| `87d10ad` | 2026-08-11 | Sim Step 5: hand-eye transform via tf2 |
| `1ab52d8` | 2026-08-10 | Sim Step 4: back-projection verified; depth camera bug fixed |
| `6f6877a` | 2026-08-08 | Add project handoff doc |
| `31c6580` | 2026-08-08 | Sim Steps 2-3: depth verification, ArUco marker + live detection |
| `d902ef5` | 2026-08-08 | Sim Step 1: D405 camera in Gazebo |

Layout:
```
docs/                              (project plan docx)
fpga_ik_solver/                    (empty)
ros2_ws/src/
  library_bot_description/         (URDF, camera xacro, world, marker model, scripts)
  library_bot_moveit_config/       (SRDF, controllers, MoveIt config, launch files)
```

---

## What changed on Sep 29 (the Step 6.5 session)

Committed in `f7919bd`:

- **`config/library_bot.ros2_control.xacro`**: now takes `use_sim`. `false` (default) uses `mock_components/GenericSystem`, `true` uses `gazebo_ros2_control/GazeboSystem`.
- **`config/library_bot_with_ros2_control.xacro`**: takes the `use_sim` arg. When true it also adds the `libgazebo_ros2_control.so` plugin block that loads `ros2_controllers.yaml` inside Gazebo.
- **`launch/gazebo_moveit.launch.py`** (new): one launch for everything. Order: Gazebo + world, robot_state_publisher, spawn_entity, joint_state_broadcaster, arm_controller, move_group. Each step waits for the previous. Everything runs with `use_sim_time`. The robot description has XML comments stripped by regex before use.
- **`config/library_bot.srdf`**: the end effector now has its own `ee_group` (containing `ee_link`) with `parent_group="arm"`, instead of reusing the arm group.
- **`urdf/d405_camera.xacro`**: the five topic remappings were removed from the camera plugin. Topics should be back to the default names, e.g. `/d405/d405/image_raw` and `/d405/d405/depth/image_raw`. Confirm with `ros2 topic list | grep d405`; scripts that used the remapped names (`depth/image_rect_raw`, `color/...`) would need updating.
- **`launch/rviz.launch.py`** and **`config/moveit.rviz`** (new): RViz setup for MoveIt. I have not read their contents.
- `.gitignore`: build, install and log dirs excluded.

---

## How to run

First time, or after config edits:
```bash
cd ~/projects/library-retrieval-robot/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
`gazebo_ros2_control` is required: `sudo apt install ros-humble-gazebo-ros2-control` (run `sudo apt update` first if you get a 404).

Launch the full sim (Gazebo + MoveIt2):
```bash
pkill -9 gzserver gzclient
cd ~/projects/library-retrieval-robot/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export GAZEBO_MODEL_PATH=$HOME/projects/library-retrieval-robot/ros2_ws/src/library_bot_description/models:$GAZEBO_MODEL_PATH
ros2 launch library_bot_moveit_config gazebo_moveit.launch.py
```
Expected: Gazebo opens with the marker and robot, controllers activate, `move_group` prints that you can start planning.

Planning-only test without Gazebo (mock hardware): `ros2 launch library_bot_moveit_config move_group.launch.py`.

Do not run `sim.launch.py` and `gazebo_moveit.launch.py` together. `gazebo_moveit.launch.py` replaces both `sim.launch.py` and the old mock `move_group.launch.py` for Gazebo runs.

Send a test pose with `send_hardcoded_pose.py` (find it with `find ~/projects/library-retrieval-robot -name send_hardcoded_pose.py`). Confirmed reachable poses: `(0.7, 0, 0.57)`, `(0.5, 0.3, 0.57)`, `(0.9, 0, 0.57)`. Use orientation `(1, 0, 0, 0)`: identity `(0, 0, 0, 1)` was unreachable at `(0.7, 0, 0.57)`.

---

## Sim pipeline status (plan doc Section 7.1)

- [x] Step 1: D405 camera in Gazebo
- [x] Step 2: depth verified
- [x] Step 3: ArUco marker model + live detection (marker ID 0, DICT_4X4_50)
- [x] Step 4: back-projection to 3D, checked against ground truth
- [x] Step 5: hand-eye transform via tf2
- [x] Step 6: real 6-DOF URDF, DH table, sim joint limits, MoveIt2/KDL (3 poses on mock hardware)
- [x] Step 6.5: Gazebo + MoveIt2 + D405 together (commit message says verified; depth numbers not recorded, see top)
- [ ] **Step 7**: wire detect, localize, transform, IK, move
- [ ] Step 8: gripper close + grasp verification
- [ ] Step 9: base-to-arm handoff signal

---

## Next: Step 7

Goal: the arm moves to wherever the marker is actually seen, instead of a hardcoded pose.

Rough shape of the node:
1. Subscribe to the marker detection (see `aruco_live_detector.py` in `library_bot_description/scripts/`).
2. Read depth at the marker's pixel and the intrinsics from `camera_info`, then back-project to a 3D point in the camera frame.
3. Transform that point into the base frame with tf2.
4. Check `/compute_ik` for the target first, since it is cheaper than a failed plan.
5. Send the MoveIt2 goal.

Pitfalls already known:
- Decode ROS `Image` messages manually (`np.frombuffer`), never `cv_bridge`.
- Target orientation matters: some orientations are unreachable even when the position is fine.
- Expect the planner to ignore the camera: `d405_link` has visual geometry but no collision geometry, so MoveIt2 can't see it. Fine in open space, matters near the shelf.

---

## Open items and known issues

- **Depth evidence missing:** re-run `check_raw_value.py` with the arm at a known pose and log the readings (center pixel near the marker's distance, background at the far clip).
- **`d405_link` has no collision geometry** (MoveIt2 warning, non-blocking for now).
- **Joint limits are estimates**, not measured hardstops. Only `joint3_elbow` was tightened (to ±2.094 rad). Revisit once the arm is printed.
- **Link geometry is primitive cylinders**, not real link shapes.
- **`cv_bridge` vs NumPy 2.x:** silently wrong output (`-inf`). Not fixed at the root; `pip install "numpy<2"` was blocked by an old pip. Worked around by not using `cv_bridge`.
- Camera clip values in `d405_camera.xacro` are marked TEMP in comments (near 0.07, far 0.5); check they are intended before real-hardware comparisons.
- `fpga_ik_solver/` was empty at last check, and the interface contract (plan doc Section 6.2) was not written.

---

## Gotchas

- `GAZEBO_MODEL_PATH` must be exported in the same terminal you launch from, every time. The ArUco marker may also need symlinking into `~/.gazebo/models/` or its texture won't render.
- Orphaned `gzserver` blocks new launches: `pkill -9 gzserver gzclient` first.
- Never spawn two D405 robots in one Gazebo instance (segfault).
- URDF/xacro edits need a full Gazebo relaunch; there is no hot-reload.
- After xacro or launch edits, rebuild with `colcon build --symlink-install` and re-source `install/setup.bash`.
- `colcon` needs `python3-colcon-common-extensions`; run `rqt_image_view` with `ros2 run rqt_image_view rqt_image_view`.
- Repeated `gzserver` restarts in one session may lose the GPU render context (suspected, never confirmed).

---

## Workflow preference

Exact copy-pasteable commands with expected output described. Raw terminal output pasted back, not summarized.

---

## Opening message for a new chat

> "Continuing my library retrieval robot sim (ROS2 Humble, Gazebo Classic, MoveIt2). Attached: this handoff and the project plan docx. Sim Steps 1 to 6.5 are done; I want to start Step 7 (detect marker, localize, transform, IK, move). First I want to re-run the D405 depth check to record real numbers for Step 6.5. Here is my current terminal output: [paste]."
