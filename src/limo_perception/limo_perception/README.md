# Perception L3.1

YAML parameters, BEST_EFFORT sensor QoS, tiered confidence, and **map-metric footprint clearance** so the whole LIMO stays on the black road.

## Nodes

| Node | Input | Output |
| --- | --- | --- |
| `lane_detector` | `/rgb/image_raw`, `/odom`, track map | `/perception/lane`, `/perception/lane/debug` |
| `obstacle_detector` | `/scan` | `/perception/obstacles`, `/perception/obstacle_stop` |
| `sign_detector` | `/rgb/image_raw` | `/perception/signs`, `/perception/sign_stop` |
| `localization_node` | `/odom` | `/localization/pose` |

## Lane + footprint safety

Black-road vision (ROI, blur, morph, connected component, centreline) plus map assist.

**`LaneInfo` fields:**

- `lateral_offset`, `heading_error`, `curvature`, `confidence`
- `left_clearance`, `right_clearance` (metres; negative = past margin)
- `footprint_safe` (rectangle + wheels + short swept check on the track map)

Official dims (defaults in `config/lane_detector.yaml`):

- `track=0.172`, `wheel_length=0.045` → half-width **0.1085 m**
- `wheelbase=0.20`, `wheel_radius=0.045` → length **0.29 m**
- `safety_margin=0.05 m`, `luma_max_road=90` (grey + white = non-drivable)

**Planner / behavior:**

- `lane_planner` holds the goal if `footprint_safe` is false; nudges away from the tighter side.
- `behavior_manager` → `STOP_BOUNDARY` after a few unsafe frames.

## Obstacle / signs

See prior Phase-1 notes: adaptive LiDAR clustering, wall vs object stop, STOP vs light ROI + temporal confirm.

## Launch / tests

```bash
ros2 launch limo_bringup j1_lane_follow.launch.py
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/limo_perception && python3 -m unittest discover -s test -v
```

## Limitations

- Metric safety uses the track **map** (camera has no calibrated BEV metres).
- Map plane size is fixed; Room 207 lighting still affects vision thresholds.
