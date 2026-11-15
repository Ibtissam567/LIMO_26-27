# Perception L3.1

Nœuds :

| Nœud | Entrée | Sortie |
| --- | --- | --- |
| `lane_detector` | `/rgb/image_raw` | `/perception/lane` |
| `obstacle_detector` | `/scan` | `/perception/obstacles`, `/perception/obstacle_stop` |
| `sign_detector` | `/rgb/image_raw` | `/perception/signs` (`STOP`, `FEU_ROUGE`, `FEU_ORANGE`, `FEU_VERT`), `/perception/sign_stop` |
| `localization_node` | `/odom` | `/localization/pose` |

Lancement :

```bash
ros2 launch limo_bringup perception.launch.py
```
