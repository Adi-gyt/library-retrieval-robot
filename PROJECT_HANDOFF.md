# Project Handoff — Autonomous Library Retrieval Robot (Sim Track)

**Purpose of this document:** paste this entire file as your first message in a new Claude chat if the current session ends, so the new chat has full context without re-explaining everything.

**Last updated:** August 8, 2026, end of Day 2 session

---

## Who I am / project context

Final-year ECE student (Sahrdaya College, Thrissur), S7, building an Autonomous Library Retrieval Robot as FYP with a 4-person team. My personal solo deliverables: the Gazebo/ROS2 simulation (this track) and a custom CORDIC-based 6-DOF IK solver on an FPGA (separate, not started yet). Machine: HP Victus 15 (Ryzen 7 5800H, 16GB RAM, RTX 3050 4GB VRAM), dual-boot Ubuntu 22.04 LTS + Windows.

Full project plan lives in a docx (`Library-Robot-Project-Plan.docx`) — single source of truth for architecture/decisions. It's checked into the repo at `docs/`. If you're picking this up in a new chat, **upload that docx first** so the new Claude has the full plan, not just this summary.

---

## What's installed on this machine (Ubuntu side)

- ROS2 Humble Desktop
- Gazebo 11.10.2 (Gazebo Classic — correct pairing with Humble, NOT the newer Gazebo/Ignition line)
- MoveIt2 (full package set)
- Nav2 + slam_toolbox (full package set)
- `opencv-contrib-python` 5.0.0 + NumPy, via pip3
- `python3-colcon-common-extensions` (colcon wasn't bundled with ros-humble-desktop, had to install separately)
- `ros-humble-rqt-image-view`
- `ros-humble-xacro`

**Known unresolved environment issue:** `cv_bridge` (installed via apt, built against NumPy 1.x) conflicts with NumPy 2.x (pulled in by the pip-installed opencv). Produces silently wrong output (`-inf`) rather than a clean crash. Workaround in use: avoid `cv_bridge` entirely, decode ROS `Image` messages manually via `np.frombuffer(msg.data, ...).reshape(...)`. Not fixed at the root — `pip install "numpy<2"` was the fix but blocked by an outdated pip not supporting `--break-system-packages`. Not worth chasing further unless it blocks something new.

---

## Repo structure — TWO separate repos, different purposes

### 1. `bot-sim-log` — github.com/Adi-gyt/bot-sim-log
Personal narrative diary, day-by-day. Same role as the `cadence-diaries` repo played for the VLSI internship. NOT the working codebase.
```
bot-sim-log/
  README.md
  logs/
    day-01.md   (Aug 4 — Ubuntu dual-boot + ROS2 stack install)
    day-02.md   (Aug 8 — camera decision, Sim Steps 1-3)
    images/     (screenshots referenced in day-02.md)
```
Commits so far: `e2d632e` (Day 1), `0c2bef4` (Day 2 initial), `d7a4775` (Day 2 addendum, Steps 2-3).

### 2. `library-retrieval-robot` — github.com/Adi-gyt/library-retrieval-robot
The actual working codebase. This is what teammates will eventually clone.
```
library-retrieval-robot/
  .gitignore          (excludes ros2_ws/build, install, log — colcon output, never commit)
  docs/
    Library-Robot-Project-Plan.docx   (single source of truth, updated Aug 8 for D405 switch)
  ros2_ws/
    src/
      library_bot_description/
        package.xml
        CMakeLists.txt
        urdf/
          d405_camera.xacro          (D405 sensor macro — reusable)
          library_bot.urdf.xacro     (top-level robot: base_footprint -> base_link -> ee_placeholder_link -> D405 mounted here; also has test_target_link, a box 0.3m in front of camera for depth testing)
        launch/
          sim.launch.py              (starts Gazebo + spawns the robot)
        models/
          aruco_marker_0/            (Gazebo model: textured plane showing ArUco marker ID 0, DICT_4X4_50)
            model.config
            model.sdf
            materials/scripts/marker.material
            materials/textures/aruco_marker_0.png
        scripts/
          generate_marker.py         (creates the marker PNG, needs 50px white border or detection fails)
          test_aruco_detection.py    (Phase A test — static image, no ROS)
          aruco_live_detector.py     (Phase B — live detection on /d405/d405/image_raw, avoids cv_bridge)
          check_depth.py             (numeric depth check — currently broken by the cv_bridge/NumPy issue above, not fixed)
  fpga_ik_solver/        (empty so far — FPGA IK work hasn't started)
```
Commits so far: `d902ef5` (Sim Step 1 — camera working), `31c6580` (Sim Steps 2-3 — depth + ArUco detection working).

**Local paths:** the git repo lives at `~/projects/library-retrieval-robot/`. There was an EARLIER, now-abandoned workspace at `~/ros2_ws/` from before this structure was set up — that one is not used, everything real is under `~/projects/library-retrieval-robot/ros2_ws/`.

---

## Key architecture decisions made so far

- **Camera: Intel RealSense D405**, not the originally-planned Waveshare IMX219-83. D405 has onboard depth ASIC (no manual stereo calibration needed), 7-50cm ideal range matching grasp-approach distance, global shutter (matters for eye-in-hand motion). Logged as Decision D8 in the plan doc.
- **Sim depth sensor:** single Gazebo `type="depth"` sensor (RGBD), not two raw cameras — mirrors what the D405's ASIC does on real hardware.
- **Sim architecture:** placeholder arm geometry used throughout Steps 1-9 (a fixed `ee_placeholder_link`, no real joints yet) — real DH parameters are blocked on a library visit (rained out so far), but this does NOT block any of Steps 1-9 per the plan doc's explicit note.
- **FPGA IK never touches camera data directly** — by the time a pose reaches the IK solver, vision pipeline (Steps 1-6) has already reduced it to one target pose. Vision and FPGA tracks are independent, connected only by an interface contract (plan doc Section 6.2, not written yet).

---

## Sim pipeline status (plan doc Section 7.1, 9 steps total)

- [x] **Step 1** — D405 camera sensor set up matching real specs, verified publishing in Gazebo (topics: `/d405/d405/image_raw`, `depth/image_raw`, `camera_info`, `depth/camera_info`, `points`, all confirmed live at ~15Hz)
- [x] **Step 2** — Depth verified sane via a test box placed 0.3m in front of camera; visually confirmed distinct depth region in rqt_image_view. Exact numeric confirmation blocked by the cv_bridge/NumPy issue (not resolved, deemed not worth chasing further)
- [x] **Step 3** — ArUco detection working end-to-end: static-image test passed (Phase A), then a real Gazebo model with marker texture built and detected live via camera topic (Phase B). Marker ID 0 correctly detected with accurate corners.
- [ ] **Step 4** — NEXT UP. Back-project the detected marker's 2D pixel position into a real 3D point using the depth value at that pixel + camera intrinsics from `camera_info`.
- [ ] **Step 5** — Hand-eye transform (camera-to-end-effector fixed transform + tf2 for end-effector-to-base). Simplified for now since placeholder arm has no moving joints.
- [ ] **Step 6** — Get MoveIt2/KDL moving the arm to a hardcoded target pose. Will need: adding at least one real movable (revolute) joint to the URDF (currently everything is fixed joints), and generating a MoveIt2 config via MoveIt Setup Assistant (not used yet). Expected to be the heaviest of steps 4-6.
- [ ] **Step 7** — Wire together: detect → localize → transform → IK → move
- [ ] **Step 8** — Integrate gripper close + grasp verification check
- [ ] **Step 9** — Wire in base-to-arm handoff signal last

None of Steps 4-9 require the library visit.

---

## Recurring debugging gotchas hit so far (useful if they recur)

- `colcon: command not found` → needs `python3-colcon-common-extensions` installed separately from ros-humble-desktop
- `rqt_image_view: command not found` even after confirmed-installed → wasn't on PATH directly; use `ros2 run rqt_image_view rqt_image_view` instead
- Downloaded `.gitignore` loses its leading dot via browser save → check with `find ~/Downloads -iname "*gitignore*"`, rename with `mv`
- ArUco markers need a blank white "quiet zone" border around them or detection fails completely, even on a perfectly valid marker
- `cv_bridge` + pip-installed NumPy 2.x = silent wrong output, not a crash — avoid `cv_bridge`, decode ROS Image messages manually
- Stuck/orphaned `gzserver` processes can silently prevent Gazebo from ever fully starting (spawn_entity service unavailable) — fix is closing all terminals and relaunching clean, not debugging the literal error
- Any URDF/xacro edit requires killing the running Gazebo instance and relaunching — changes don't hot-reload
- `GAZEBO_MODEL_PATH` must be exported in the SAME terminal you launch Gazebo from, every time (doesn't persist across terminals)

---

## Learning plan (separate track, not urgent)

Decided to prioritize getting the sim demo-ready over deep ROS2 syntax study — teacher demo is the near-term deadline, actual understanding is being deferred to a 10-day holiday coming up in late August. A separate chat was set up with a structured, hands-on learning plan following docs.ros.org (Humble version): CLI tools → URDF from scratch → movable URDF (joints) → physical/collision properties → Xacro → Gazebo integration (`<gazebo>` tags, sensor plugins). That chat is independent of this one.

---

## If resuming in a new chat, say something like:

"Here's my project handoff doc [paste this file]. I want to continue with Sim Step 4 — back-projecting the ArUco detection into a 3D point using depth data. Here's terminal output / a screenshot of [whatever the current state is]."
