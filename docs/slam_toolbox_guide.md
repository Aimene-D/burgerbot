# SLAM Toolbox Guide — Mapping, Saving, and Reusing Maps

This guide covers how to use `slam_toolbox` in the burgerbot project: running SLAM to build a map, saving it to disk, loading it back for pure localization, and tuning every parameter to get the best results in the field.

---

## Concepts

SLAM Toolbox has two distinct operating modes:

| Mode | Config file | Launch file | What it does |
|------|-------------|-------------|--------------|
| **Mapping** | `slam_toolbox.yaml` (`mode: mapping`) | `slam.launch.py` | Builds a map in real time from `/scan` + odometry |
| **Localization** | `slam_toolbox_localization.yaml` (`mode: localization`) | `localization.launch.py` | Loads a saved map and localizes the robot within it |

A saved map consists of **two files** produced by slam_toolbox's own serializer:
- `<name>.posegraph` — the pose graph (nodes + constraints)
- `<name>.data` — the raw scan data

> **Note:** `nav2_map_server` produces `.pgm` + `.yaml` files. Those work for nav2 but cannot be reloaded by slam_toolbox for continued mapping or graph-based localization. Always save with the slam_toolbox service below.

---

## Phase 1 — Build a Map

### 1.1 Real robot

```bash
# Terminal 1 — bringup + SLAM
ros2 launch burgerbot slam_nav.launch.py

# Terminal 2 — optional: open RViz to watch the map grow
rviz2 -d $(ros2 pkg prefix burgerbot)/share/burgerbot/rviz/slam.rviz
```

`slam_nav.launch.py` starts: bringup → slam → navigation, so the robot is already teleoperable or goal-able while mapping.

If you only want SLAM without Nav2:

```bash
ros2 launch burgerbot slam.launch.py
```

### 1.2 Simulation (Gazebo)

```bash
# Headless — pair with `rviz` service to watch the map
docker compose -f docker/docker-compose.yml run --rm sim-slam

# With Gazebo window open
docker compose -f docker/docker-compose.yml run --rm sim-slam-gui

# Full stack: Gazebo + SLAM + Nav2, with Gazebo window
docker compose -f docker/docker-compose.yml run --rm sim-nav-gui
```

### 1.3 Drive the robot around

Use teleop, a joystick, or Nav2 goals to explore the whole space you want mapped. Watch the `/map` topic in RViz until coverage looks complete.

---

## Using SLAM Toolbox in RViz

RViz is where you watch the map build and verify the robot is localizing correctly. The burgerbot RViz config (`config/rviz/burgerbot.rviz`) already includes the displays you need for mapping.

### Launching RViz

The `rviz` service loads the pre-built burgerbot layout and is meant to run **alongside** a mapping session:

```bash
# Terminal 1 — start mapping (headless)
docker compose -f docker/docker-compose.yml run --rm sim-slam

# Terminal 2 — open RViz to watch the map grow
xhost +local:docker
docker compose -f docker/docker-compose.yml run --rm rviz
```

Or use `sim-slam-gui` / `sim-nav-gui` which open Gazebo's window, then run `rviz` separately to watch the map.

### Displays you'll use for mapping

The config already has these enabled:

| Display | Topic | What it shows | Color |
|---------|-------|---------------|-------|
| **Map** | `/map` | The occupancy grid being built by slam_toolbox. Black = occupied (walls), white = free, grey = unknown | grayscale |
| **LaserScan** | `/scan` | The live LiDAR returns. These should land exactly on the black walls of the map | red |
| **RobotModel** | `/robot_description` | The 3D robot mesh at its estimated pose | — |
| **TF** | — | Coordinate frames with axes and labels. Confirms `map → odom → base_footprint` exists | — |

### The one setting you must change: Fixed Frame

The config ships with **Fixed Frame: `odom`**. That's fine for the first few seconds before a map exists, but for mapping you want to see the world in the map frame:

1. In the left **Displays** panel, expand **Global Options** at the top
2. Click the **Fixed Frame** value (`odom`) and change it to **`map`**

With Fixed Frame = `map`, the map stays still and the robot moves through it. With `odom`, the map jumps every time loop closure corrects drift — confusing during mapping.

### Reading the map while you drive

A good map looks like this in RViz:

- **Red laser points sit exactly on the black walls.** If the scan "smears" or sits beside the walls, your odometry or EKF is off — the SLAM scan matcher is struggling.
- **Walls are crisp single lines**, not double/ghosted walls. Ghosting means loop closure hasn't fired yet — drive back to a previously-mapped area to trigger it.
- **The map only redraws every few seconds.** That's the `map_update_interval: 5.0` parameter (see [Map Update and Publishing](#map-update-and-publishing)). Lower it to `1.0` for a more responsive display.

### Verifying localization quality

Watch the **TF** display while driving:

- The `base_footprint` frame should move smoothly with the robot
- When you return to a known area, you may see the whole map "snap" — that's a loop closure correcting accumulated drift. This is normal and good.
- If the robot's TF position drifts far from where the laser says it is, mapping quality is degrading.

### What's *not* in the default config (and how to add it)

The shipped layout is minimal. To add more displays, click **Add** at the bottom of the Displays panel:

- **Pose Graph** — `Add → By topic → /slam_toolbox/graph_visualization` (MarkerArray). Shows the nodes and constraints of the pose graph; useful for seeing where loop closures connected.
- **Scan match poses** — `/slam_toolbox/scan_visualization` shows individual scan placements.

To make these the default, save the layout: **File → Save Config** (overwrites `burgerbot.rviz`).

---

## Phase 2 — Save the Map

While the mapping node is still running, call its `serialize_map` service:

```bash
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/$USER/maps/my_map'}"
```

This writes:
```
/home/<user>/maps/my_map.posegraph
/home/<user>/maps/my_map.data
```

**Do not include the file extension in the path — slam_toolbox appends it automatically.**

Create the destination directory first if it doesn't exist:

```bash
mkdir -p ~/maps
```

### Verify the save

```bash
ls -lh ~/maps/my_map.*
# Expected:
# my_map.posegraph   (a few KB to MB depending on environment size)
# my_map.data        (typically larger — raw scan storage)
```

---

## Phase 3 — Localize on the Saved Map

Once you have a map, stop the mapping session and use the localization pipeline.

### 3.1 Real robot — localization + navigation

```bash
ros2 launch burgerbot nav_saved_map.launch.py \
  map:=/home/$USER/maps/my_map
```

This single launch file starts: bringup → localization (slam_toolbox in localization mode) → Nav2 navigation stack.

### 3.2 Localization only (without Nav2)

```bash
ros2 launch burgerbot localization.launch.py \
  map:=/home/$USER/maps/my_map
```

### What happens at startup

The `localization_slam_toolbox_node` loads the `.posegraph` + `.data` files and, because `map_start_at_dock: true` is set in `slam_toolbox_localization.yaml`, it assumes the robot starts at the origin of the saved map. The node publishes `/map` and the `map → odom` transform immediately.

---

## Parameters Reference

### Core Mode and Frame Parameters

| Parameter | Current value | What it does | Field impact |
|-----------|--------------|--------------|-------------|
| `mode` | `mapping` | `mapping` builds the map continuously; `localization` loads a fixed map and only localizes | Must match your use case — mapping if exploring, localization if navigating a known space |
| `odom_frame` | `odom` | The name of the odometry frame published by robot_localization | Must match the TF tree exactly |
| `map_frame` | `map` | The name of the map frame slam_toolbox publishes | Must match what Nav2 expects |
| `base_frame` | `base_footprint` | The robot's base frame (slam_toolbox will look for `base_footprint → odom` TF) | Must exist in TF tree before slam_toolbox starts |
| `scan_topic` | `/scan` | Topic name of the LaserScan input | Must be the actual topic your LiDAR publishes to |

---

### Scan Processing Parameters

| Parameter | Current value | What it does | Field impact |
|-----------|--------------|--------------|-------------|
| `throttle_scans` | `1` | Only process 1 out of every N scans (1 = process all) | Increasing this reduces CPU load and map update rate; useful on slow hardware. At 1, every scan contributes to the map |
| `minimum_time_interval` | `0.5` s | Minimum time between processed scans, regardless of `throttle_scans` | A hard lower bound: even if the LiDAR runs at 10 Hz, slam_toolbox will only use a scan every 0.5 s. Raising this reduces CPU cost but coarsens the map in fast-moving scenarios |
| `min_laser_range` | `0.12` m | Scans closer than this are ignored | Filters out the robot's own body. Too low = body reflections corrupt the map; too high = nearby walls are invisible |
| `max_laser_range` | `3.5` m | Scans farther than this are ignored | Set to your LiDAR's reliable range (3.5 m matches the TurtleBot3 LiDAR). Too high = noisy far returns corrupt the map; too low = you miss distant walls |

---

### Map Update and Publishing

| Parameter | Current value | What it does | Field impact |
|-----------|--------------|--------------|-------------|
| `map_update_interval` | `5.0` s | **How often the `/map` topic is published (in seconds)** | **This is why you see the map refresh slowly.** At 5.0 s the map only updates once every 5 seconds in RViz. Reduce to `1.0` for ~1 Hz refresh or `0.5` for 2 Hz — but more frequent updates use more CPU. In simulation where CPU is cheap, `1.0` is a good default |
| `resolution` | `0.05` m | Size of one grid cell in the occupancy map | 5 cm is standard for indoor robots. Smaller (e.g. 0.025) = sharper map but 4× more memory. Larger (e.g. 0.1) = coarser map, lighter memory. Do not change between mapping and localization sessions |
| `transform_publish_period` | `0.02` s | How often the `map → odom` transform is broadcast (50 Hz) | This is separate from `map_update_interval`. The TF broadcast must be fast enough for Nav2 (≥10 Hz). 0.02 s = 50 Hz which is fine. Do not raise this above 0.1 |

> **Quick fix for slow map refresh:** In `config/slam_toolbox.yaml` change `map_update_interval` from `5.0` to `1.0`. The map will now refresh at 1 Hz in RViz, which is much more responsive during exploration.

---

### Loop Closure Parameters

Loop closure detects when the robot returns to a previously visited place and corrects accumulated drift. This is what makes SLAM dramatically more accurate than dead reckoning.

| Parameter | Current value | What it does | Field impact |
|-----------|--------------|--------------|-------------|
| `do_loop_closing` | `true` | Enable/disable loop closure entirely | Leave `true` for mapping. The map snaps walls into alignment when you revisit an area. Set `false` only for tiny spaces or when debugging drift |
| `loop_search_maximum_distance` | `3.0` m | Maximum distance from the current pose to search for loop closure candidates | Too small = misses valid closures when revisiting from far away. Too large = expensive search and false positives. 3 m is fine for indoor rooms; raise to 5–8 m for large warehouses |
| `loop_match_minimum_response_coarse` | `0.35` | Minimum scan-match score for a coarse loop closure candidate to proceed to fine matching (0–1) | Lower = accepts weaker matches (more corrections, risk of false closures). Higher = only very confident matches close loops. 0.35 is conservative; if your map looks correct you can try 0.3 |
| `loop_match_minimum_response_fine` | `0.45` | Minimum score required after fine matching to accept a loop closure | This is the final gate. Raise to 0.5–0.6 in noisy environments to avoid bad corrections. Lower to 0.4 if you're missing valid closures |
| `loop_match_distance_threshold` | `0.5` m | Maximum translational error between the predicted and matched pose for a loop closure to be accepted | Tight threshold prevents bad jumps. If you have good odometry, 0.3 m works. In poor-odometry environments, 0.5–1.0 m allows more corrections |
| `loop_match_maximum_variance_coarse` | `3.0` | Maximum covariance of a loop closure candidate at the coarse stage | Filters out geometrically ambiguous candidates (e.g. long symmetric corridors) |
| `loop_search_space_dimension` | `8.0` m | Side length of the angular search space during loop closure | Covers ±4 m around the candidate pose. Raise if you have large open spaces |

---

### Scan Buffer and Link Parameters

These control how scan history is maintained and how new scans link into the pose graph.

| Parameter | Current value | What it does | Field impact |
|-----------|--------------|--------------|-------------|
| `scan_buffer_size` | `10` | Number of recent scans kept in the running buffer for scan-to-scan matching | Larger buffer = more robust matching when the robot turns fast but uses more memory. 10 is fine for a small LiDAR |
| `scan_buffer_maximum_scan_distance` | `10.0` m | Maximum distance a buffered scan can be from the current pose before it's discarded | Prevents old far-away scans from interfering with current matching |
| `link_match_minimum_response_fine` | `0.1` | Minimum score for linking a new scan to the pose graph (very low) | This is for sequential scan matching (not loop closure). Very low because sequential scans almost always match well. Raise only if the map accumulates random noise |
| `link_scan_maximum_distance` | `1.5` m | Maximum distance the robot can move between two linked scans | If the robot teleports or odometry fails, scans further apart than this won't be linked, preventing garbage in the pose graph |
| `distance_decomposition_radius` | `1.0` m | Radius used to decompose the distance for scan correlation | Affects how overlapping scan regions are weighted during matching |

---

### Solver Parameters (Ceres)

slam_toolbox uses Google's Ceres Solver for pose graph optimization. These rarely need changing.

| Parameter | Current value | What it does | Field impact |
|-----------|--------------|--------------|-------------|
| `solver_plugin` | `CeresSolver` | Which optimizer to use | Ceres is the standard and well-tested choice |
| `ceres_linear_solver` | `SPARSE_NORMAL_CHOLESKY` | Linear algebra backend for Ceres | `SPARSE_NORMAL_CHOLESKY` is fast for sparse pose graphs (typical). `DENSE_NORMAL_CHOLESKY` is slower but more stable if you get solver failures |
| `ceres_preconditioner` | `SCHUR_JACOBI` | Preconditioner for the iterative solver | `SCHUR_JACOBI` works well for SLAM-scale problems |
| `ceres_trust_strategy` | `LEVENBERG_MARQUARDT` | Non-linear optimization strategy | Levenberg-Marquardt is robust and is the default for most SLAM systems. `DOGLEG` can be faster in some cases |
| `ceres_loss_function` | `None` | Robust loss function to reduce the effect of outliers | `None` is fine when loop closures are reliable. Set to `HuberLoss` if you're getting bad loop closures that corrupt the map |

---

### TF and Timing Parameters

| Parameter | Current value | What it does | Field impact |
|-----------|--------------|--------------|-------------|
| `transform_timeout` | `0.2` s | How long to wait for a TF lookup before giving up | If slam_toolbox logs `"Could not get transform"` errors, increase this. On a loaded system or over network, 0.5 s may be needed |
| `tf_buffer_duration` | `30.0` s | How much TF history to keep in the buffer | Keep at 30 s or higher. A short buffer causes failures when the pose graph optimizer looks up old transforms during loop closure |
| `debug_logging` | `false` | Enable verbose debug output | Set `true` only when debugging — floods the terminal and slows the node |

---

### Localization-only Parameters (`slam_toolbox_localization.yaml`)

| Parameter | Value | What it does | Field impact |
|-----------|-------|--------------|-------------|
| `mode` | `localization` | Disables all map growth; map is frozen | The robot cannot see new obstacles in the map, but it does track its pose |
| `map_start_at_dock` | `true` | Robot starts at the map origin pose | If `true`, physically place the robot at the same spot where mapping started. If the start is different, publish on `/initialpose` to set the initial pose |
| `do_loop_closing` | `false` | No loop closure needed on a fixed map | Saves CPU. Keep `false` in localization mode |

---

## Recommended Tuning Workflow

1. **Start with defaults** — drive the robot slowly, build a map
2. **If map refreshes too slowly** → lower `map_update_interval` (try `1.0`)
3. **If map is noisy** → raise `link_match_minimum_response_fine` (try `0.3`) and `loop_match_minimum_response_fine` (try `0.55`)
4. **If loop closures are missing** → lower `loop_match_minimum_response_coarse` (try `0.3`) and raise `loop_search_maximum_distance` (try `5.0`)
5. **If loop closures cause jumps** → raise `loop_match_minimum_response_fine` (try `0.6`) and add `ceres_loss_function: HuberLoss`
6. **On a slow machine** → raise `throttle_scans` to `2` or `3`, raise `minimum_time_interval` to `1.0`

---

## Troubleshooting

### Map refreshes too slowly in RViz

The `/map` topic only publishes once per `map_update_interval` seconds. The default is `5.0` s — that's why you see the map update infrequently. Set `map_update_interval: 1.0` in `slam_toolbox.yaml` for approximately 1 Hz map updates.

Note that `map_update_interval` only controls the occupancy grid publish rate. The pose graph and TF are updated continuously at scan rate.

### Map is empty / no `/map` topic

The slam_toolbox node is a **lifecycle node**. The launch files auto-drive `configure → activate`, but if you see it stuck in `unconfigured` state:

```bash
# Check the node state
ros2 lifecycle get /slam_toolbox

# Manually configure and activate if needed
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
```

### Localization drifts immediately after startup

- Make sure `map_start_at_dock: true` is set and the robot is physically at the same pose where the map origin was (typically where the robot was when SLAM started).
- If the robot's start pose differs, you can provide an initial pose via the `/initialpose` topic (same topic Nav2's AMCL uses).

### "File not found" error when loading map

Verify the path **without extension** is correct:

```bash
# Wrong
map:=/home/user/maps/my_map.posegraph

# Correct
map:=/home/user/maps/my_map
```

### Map save service not available

The node must be in **active** state and in **mapping** mode. The `serialize_map` service does not exist on `localization_slam_toolbox_node`.

---

## Quick Reference

```bash
# --- MAPPING ---
# Start mapping (real robot)
ros2 launch burgerbot slam.launch.py

# Start mapping + navigation simultaneously
ros2 launch burgerbot slam_nav.launch.py

# Start mapping in simulation (headless — pair with rviz service)
docker compose -f docker/docker-compose.yml run --rm sim-slam

# Start mapping in simulation with Gazebo window
docker compose -f docker/docker-compose.yml run --rm sim-slam-gui

# Start mapping + Nav2 in simulation with Gazebo window
docker compose -f docker/docker-compose.yml run --rm sim-nav-gui

# Save map (while mapping node is running)
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/$USER/maps/my_map'}"

# --- LOCALIZATION ---
# Navigate on a saved map (real robot)
ros2 launch burgerbot nav_saved_map.launch.py \
  map:=/home/$USER/maps/my_map

# Localization only
ros2 launch burgerbot localization.launch.py \
  map:=/home/$USER/maps/my_map
```
