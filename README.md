# Library Retrieval Robot

A simulated library-retrieval manipulator built on ROS 2 Humble, Gazebo and MoveIt. The robot locates boxes by marker, picks them up with a gripper, and places them into slots on a pedestal.

## Current status

The full pick-and-place demo runs end to end from a cold start, in fresh terminals, with no manual environment setup.

- All three boxes spawn in the world at startup.
- The demo moves boxes 2 and 3 from table A to their pedestal slots (box 1 is spawned but not moved by default).
- Each box is found by its marker, grasped, carried, then released with a detach call.
- Boxes are attached to and detached from the gripper through the Gazebo model attachment plugin (`boeing_gazebo_model_attachment_plugin_msgs`).
- Attach and detach service calls are retried, so a slow-to-start plugin no longer causes a failed run.

### Last verified result

| Box | Source (table A) | Target slot | Final position (x, y, z) |
|-----|------------------|-------------|--------------------------|
| 2 | y = 0.00 | x = 0.60 | 0.599, 0.450, 0.490 |
| 3 | y = -0.20 | x = 0.50 | 0.499, 0.450, 0.490 |

Both placements land within about 1 cm of the target, and both detach calls report `ok`.

## Running the demo

Each step works in a freshly opened terminal. The scripts source ROS 2 Humble, the workspace overlay (`ros2_ws/install/setup.bash`) and `~/gz_env.sh` themselves.

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

   The box order defaults to `2,3` and can be changed, for example `ORDER=1,2,3 tools/run_demo.sh`.

4. To reset the boxes for another run without restarting the sim:

   ```bash
   tools/reset_demo.sh
   ```

If a previous sim is still running, stop it first with `pkill -f gz; pkill -f ruby`.

## Repository layout

- `tools/` : `start_sim.sh`, `ready.sh`, `run_demo.sh`, `reset_demo.sh`
- `ros2_ws/src/library_bot_description/` : robot description, world, and the demo scripts (`scripts/step11_demo.py`)

## Troubleshooting

- **"The passed service type is invalid" or "Unknown package 'boeing_gazebo_model_attachment_plugin_msgs'"**: the workspace overlay was not sourced in that terminal. The tools scripts handle this; if you run Python scripts by hand, run `source ros2_ws/install/setup.bash` first.
- **Demo starts before the sim is ready**: wait for `READY` from `tools/ready.sh` before running the demo.

## Next steps

- Add box 1 to the default demo order.
- Retrieval from the pedestal back to the table.
- Add more details here as the project grows (hardware, perception notes, planned shelves).
