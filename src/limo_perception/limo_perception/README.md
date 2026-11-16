# Perception L3.1

Phase 1 reliability notes: parameters live in `config/*.yaml`, sensor topics use BEST_EFFORT QoS, and confidence values are tiered so control can trust them.

## Nodes

| Node | Input | Output |
| --- | --- | --- |
| `lane_detector` | `/rgb/image_raw`, `/odom`, track map | `/perception/lane`, `/perception/lane/debug` |
| `obstacle_detector` | `/scan` | `/perception/obstacles`, `/perception/obstacle_stop` |
| `sign_detector` | `/rgb/image_raw` | `/perception/signs` (`STOP`, `FEU_ROUGE`, `FEU_ORANGE`, `FEU_VERT`), `/perception/sign_stop` |
| `localization_node` | `/odom` | `/localization/pose` |

## Lane detection (actual algorithm)

The detector follows the **black road**, not a yellow centre line:

1. Lower-image ROI, Gaussian blur, black threshold, morphological open/close.
2. Connected component under the robot (ignore far black islands).
3. Walk the black strip, keep clearance from white edges.
4. Publish `lateral_offset`, `heading_error`, `curvature`, `confidence`.
5. Optional map assist (`qcar_large_track` share texture) for wheel-clearance / return-to-road.

### Confidence tiers

| Source | Meaning | Typical confidence |
| --- | --- | --- |
| `black` | Vision found the road | 0.70–0.95 |
| `wheels` | Map says a wheel is on white (no vision road) | 0.65, soft ±0.32 nudge |
| `black` + map | Vision road + soft wheel bias | keep camera centre, confidence ≥ 0.75 |
| `return` | Map recovery toward road | 0.55 |
| `forbidden` | No black road in view | 0.40 |
| `none` | Empty | 0.0 |

`lane_planner` still applies **lateral recovery** when `|lateral_offset| >= 0.55` even if confidence is below 0.5.

Map path: resolved from package `qcar_large_track` share (override with `map_path`).

## Obstacle detection

- Front-sector LiDAR points → adaptive clustering: `gap(r) = d0 + k r Δθ`.
- `min_cluster_points` rejects isolated noise.
- Watchdog: if `/scan` is older than `scan_timeout`, publish `obstacle_stop=true`.
- Narrow corridor (`block_half_width`) so side curbs on a turn do not stop the robot.
- Large clusters (radius > `max_obstacle_radius`) are treated as walls: only stop closer than `wall_stop_distance`.
- `stop_confirm_scans` requires several blocked scans before latching the stop.

## Sign detection

- **STOP**: mid-frame ROI, roughly square aspect, temporal confirm (`stop_confirm_frames`).
- **Lights**: upper ROI, compact/near-circular blobs; red lights are not re-labeled as STOP.
- Confidence mixes blob area, aspect score, and STOP streak.

## Launch

```bash
ros2 launch limo_bringup perception.launch.py
# or full J1 stack
ros2 launch limo_bringup j1_lane_follow.launch.py
```

## Unit tests (no Gazebo)

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/limo_perception
python3 -m unittest discover -s test -v
```

## Limitations

- Black-road thresholds are tuned for Room 207 / QCar track lighting in Gazebo.
- Sign distance is a rough pinhole estimate.
- Map assist needs the `qcar_large_track` texture installed in the package share.
