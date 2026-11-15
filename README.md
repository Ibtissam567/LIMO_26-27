# LIMO 2026-27 — BE Véhicule Autonome (ENSEM)

Dépôt d’équipe pour partager le travail ROS 2 du projet **LIMO / Room 207**.

**Équipe (GitHub) :** [Hafsa-MHd](https://github.com/Hafsa-MHd) · [Ibtissam567](https://github.com/Ibtissam567)  
**Groupe école :** BE 2026 Groupe 4 (GitLab Univ. Lorraine)  
**LIMO :** n° 204 — IP `100.75.155.204`

---

## Où en est-on ? (pour Ibtissam & l’équipe)

| Sujet | État | Où le trouver |
| --- | --- | --- |
| Livrables **L0.1** (planning) + **L1.1** (architecture ROS2) | Fait, rendu prévu 23/10 | `docs/deliverables/` |
| Sujet officiel du BE | PDF | `docs/deliverables/` |
| Fiche groupe GitLab | À compléter (rôles) | `docs/group-informations/` |
| Packages ROS2 de l’équipe (`limo_*`) | Squelette + premiers nœuds | `src/` |
| Simulation Room 207 | OK sous **Ubuntu 22.04 + ROS 2 Humble** (WSL) | dépendances école (voir plus bas) |
| Suivi de voie / polar control | En cours (J1) | `limo_control`, `limo_bringup` |
| MATLAB du prof + nos réponses (loi polaire) | Fait | `matlab/` (voir `matlab/REPONSES_GROUPE.md`) |

### Ce qui a été fait jusqu’ici (résumé)

1. **Environnement** : WSL Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic (la stack Room207 n’est pas sur Jazzy).
2. **Prise en main** : simulation `room207_limo`, téléopération, topics LIMO.
3. **Architecture L1.1** : packages prévus `limo_interfaces`, `limo_perception`, `limo_planning`, `limo_control`, `limo_supervision`, `limo_bringup`.
4. **Code démarré** :
   - messages custom (voies, panneaux, obstacles, mission…) ;
   - nœuds perception (lane / obstacle / sign — squelettes) ;
   - planning (road / lane / trajectory tester) ;
   - contrôle polaire (loi du cours CMD NUM / Simulink de l’an dernier) + lane follower ;
   - launch J1 : `j1_lane_follow.launch.py`.

### Ce qui n’est **pas** dans ce dépôt (volontairement)

Les dépôts **école** (lourds, lecture seule) restent sur GitLab :

- `limo`, `environment`, `teleoperation`, `recording`, `room207-limo`

→ Il faut les cloner à côté dans `~/ros2_ws/src` (voir installation).

---

## Structure du dépôt

```text
LIMO_2026-27/
├── README.md                 ← ce fichier
├── matlab/                   ← MATLAB/Simulink du prof + nos réponses
│   ├── README.md             ← notice enseignant
│   ├── REPONSES_GROUPE.md    ← ce que nous avons rempli
│   ├── run_polar.m           ← test A→Z (notre commande)
│   └── +limo/polar_command.m ← loi polaire
├── docs/
│   ├── deliverables/         ← L0.1 / L1.1 + sujet
│   ├── PHASE2_...md          ← doc phase perception/contrôle
│   └── group-informations/   ← fiche membres
└── src/
    ├── limo_interfaces/      ← messages .msg partagés
    ├── limo_perception/      ← détection voies / obstacles / panneaux
    ├── limo_planning/        ← planificateurs
    ├── limo_control/         ← polar_controller, lane_follower
    ├── limo_supervision/     ← (squelette) MQTT / KPI plus tard
    └── limo_bringup/         ← launch + params + modèles SDF tests
```

### MATLAB (demande des enseignants)

Le dossier `matlab/` reprend le package officiel `limo/matlab` du site école.  
Nous y avons **complété** la commande polaire (`polar_command.m`, `run_polar.m`).  
Explications : [`matlab/REPONSES_GROUPE.md`](matlab/REPONSES_GROUPE.md).

---

## Installation rapide (Humble)

```bash
# 1) WSL Ubuntu 22.04 + ROS 2 Humble déjà installés
source /opt/ros/humble/setup.bash

# 2) Workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# 3) Ce dépôt (packages équipe)
git clone https://github.com/Ibtissam567/LIMO_26-27.git
# ou : git pull si déjà cloné
# Première installation, pas à pas : docs/environnement/README.md

# 4) Packages école (GitLab) — à faire une fois
# limo, environment, teleoperation, recording, room207-limo
# + symlink : limo_ros2 -> limo/ros2_humble/limo_ros2

# 5) Build (packages équipe)
cd ~/ros2_ws
colcon build --symlink-install --packages-select \
  limo_interfaces limo_perception limo_planning \
  limo_control limo_supervision limo_bringup
source install/setup.bash
```

### Lancer la simu école

```bash
ros2 launch room207_limo room207_limo.launch.py
```

### Lancer le test J1 (suivi de voie + simu)

```bash
ros2 launch limo_bringup j1_lane_follow.launch.py
```

---

## Packages — rôle de chacun

| Package | Rôle |
| --- | --- |
| `limo_interfaces` | Messages partagés entre WP (`LaneInfo`, `Obstacle*`, `TrafficSign*`, `Mission*`) |
| `limo_perception` | `lane_detector`, `obstacle_detector`, `sign_detector` |
| `limo_planning` | `road_planner`, `lane_planner`, `trajectory_tester` |
| `limo_control` | `polar_controller` (ρ, α, β du cours), `lane_follower` |
| `limo_supervision` | Réservé WP4 (MQTT / InTouch) — squelette |
| `limo_bringup` | Launch files + `config/j1_params.yaml` + SDF but/obstacle |

---

## Comment collaborer sur GitHub

1. Cloner / pull avant de travailler.
2. Travailler sur une branche : `git checkout -b wp3/lane-detector` (exemple).
3. Commit messages clairs (en français ou anglais).
4. Push + Pull Request vers `main` (ou push direct si vous êtes d’accord en binôme).

```bash
git pull origin main
git checkout -b mon-travail
# ... modifications ...
git add -A
git commit -m "Ajoute squelette lane_detector"
git push -u origin mon-travail
```

---

## Document de phase (16/11 — L2.1 / L3.1)

Voir le détail de ce qui a été fait en perception / planification / contrôle :

- [docs/PHASE2_Perception_Planification_Controle.md](docs/PHASE2_Perception_Planification_Controle.md)

## Liens utiles

- Repo GitHub (ce dépôt) : https://github.com/Ibtissam567/LIMO_26-27
- Préparer son PC (première fois) : [docs/environnement/README.md](docs/environnement/README.md)
- Groupe GitLab école : `coll/l-inp/ensem/robotic_lab/students/be-2026-groupe-4`

---

*Dernière mise à jour partage : novembre 2026 — Hafsa (travail local ROS2 + livrables L0/L1).*

