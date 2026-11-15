# Document de phase — Perception, détection, planification et contrôle

**Projet :** BE Véhicule Autonome — LIMO / Room 207 (ENSEM)  
**Équipe :** Groupe 4 (dont Hafsa Mehdaoui, Ibtissam Eddadsi, …)  
**Période concernée :** après les livrables L0.1 / L1.1 → jusqu’au rendu **16/11/2026**  
**Livrables visés :** **L2.1** (planification / contrôle en simulation) et **L3.1** (détection / perception en simulation) — dépôt GitLab / GitHub  

> **Statut de ce fichier :** BROUILLON pour relecture (Hafsa).  
> **Ne pas considérer comme poussé sur GitHub** tant que tu n’as pas donné ton OK.

---

## 1. Contexte : où on en est dans le BE

### 1.1 Phase précédente (terminée)

| Livrable | Contenu | Date |
| --- | --- | --- |
| **L0.1** | Planning prévisionnel + répartition des tâches (WP0–WP4) | 23/10/2026 |
| **L1.1** | Diagramme d’architecture ROS 2 (packages, nœuds, topics, messages) | 23/10/2026 |

Ces documents sont dans `docs/deliverables/`  
(`BE_Vehicule_Autonome_L0.1_L1.1.pdf` / `.docx`).

### 1.2 Phase actuelle (celle de ce document)

D’après le sujet du BE, le **16/11/2026** correspondent surtout à :

- **L2.1** — fonctions de **planification et contrôle** validées en **simulation**  
- **L3.1** — fonctions de **détection / perception** validées en **simulation**

C’est donc la phase où l’on **met en œuvre dans ROS 2** ce qui était seulement dessiné dans L1.1, et où l’on montre en Gazebo (Room 207) que :

1. le véhicule **perçoit** (voie, obstacles, panneau Stop approximatif) ;
2. le véhicule **se commande** pour aller vers un but / rester sur la voie ;
3. une **planification** sur la carte de la piste peut proposer un chemin.

### 1.3 Ce que ce document n’est pas

- Ce n’est **pas** encore le rapport final (L0.2, janvier).  
- Ce n’est **pas** la supervision MQTT / InTouch (WP4, plus tard).  
- Ce n’est **pas** la validation complète sur le **LIMO réel** (prévue plus tard dans le planning).

---

## 2. Objectifs techniques de la phase

Alignés sur le sujet :

### WP2 — Planification et contrôle (vers L2.1)

- Définir une stratégie pour rejoindre une destination.  
- Générer / suivre une trajectoire réalisable.  
- Développer une loi de commande.  
- **Valider le déplacement en simulation.**

### WP3 — Détection et perception (vers L3.1)

- Détecter les **voies** (ligne jaune / route).  
- Détecter les **obstacles** (LiDAR).  
- Détecter un indice de **Stop** (couleur rouge caméra — version simplifiée).  
- Publier ces informations sur des topics ROS 2 utilisables par le contrôle.

---

## 3. Environnement de travail utilisé

| Élément | Choix | Pourquoi |
| --- | --- | --- |
| OS | WSL **Ubuntu 22.04** | Compatible Room207 |
| ROS | **ROS 2 Humble** | Stack école Room207 / Gazebo Classic |
| Simu | `room207_limo` + Gazebo 11 | Circuit salle 207 |
| Dépôt partage équipe | GitHub `Ibtissam567/LIMO_2026-27` | Travail commun Hafsa / Ibtissam |
| Ressources école | GitLab `robotic_lab` (`limo`, `environment`, `room207-limo`, …) | Non recopiées en entier dans GitHub |

**Important :** Ubuntu 24.04 + ROS 2 Jazzy (utilisé pour les TP intro) **ne lance pas** correctement Room207 (pas de Gazebo Classic). Toute la phase simu LIMO se fait donc sous **Humble**.

---

## 4. Architecture logicielle mise en place

On a créé les packages prévus dans L1.1 :

```text
src/
├── limo_interfaces/     messages partagés (.msg)
├── limo_perception/     détection voie / obstacle / panneau
├── limo_planning/       planification sur carte + tests trajectoire
├── limo_control/        polar_controller + lane_follower
├── limo_supervision/    squelette (WP4 plus tard)
└── limo_bringup/        launch + paramètres + modèles SDF de test
```

### 4.1 Chaîne de données (vue simple)

```text
Caméra /rgb/image_raw ──► lane_detector ──► /perception/lane
                       └─► sign_detector ──► /perception/sign_stop
                                            /perception/signs

LiDAR /scan ──────────► obstacle_detector ─► /perception/obstacle_stop
                                            /perception/obstacles

Odométrie /odom ──┐
But /mission/goal ┼─► polar_controller ──► /cmd_vel ──► LIMO (Gazebo)
Stops perception ─┘

Carte piste ──► road_planner ──► /mission/goal (vers polar_controller)
Voie ─────────► lane_follower ──► /cmd_vel   (mode suivi de voie)
```

Deux modes de commande coexistent dans le code :

1. **Suivi de voie** (`lane_follower`) : réactif, à partir de `LaneInfo`.  
2. **Contrôle polaire vers un but** (`polar_controller`) : loi du cours CMD NUM (ρ, α, β), avec freinage obstacle / Stop.

Le launch principal de démonstration J1 utilise surtout **perception + polar_controller** + spawn d’un but dans Gazebo.

---

## 5. Détail de ce qui a été codé

### 5.1 `limo_interfaces` — contrats entre packages

Messages créés (comme dans le document L1.1) :

| Message | Rôle |
| --- | --- |
| `LaneInfo` | `lateral_offset`, `heading_error`, `curvature`, `confidence` |
| `Obstacle` / `ObstacleArray` | position, distance, flag `blocks_current_segment` |
| `TrafficSign` / `TrafficSignArray` | type (ex. `STOP`), confiance |
| `MissionState`, `MissionKpi`, `MissionEvent` | prévus pour supervision / machine à états (pas encore pleinement utilisés) |

**Pourquoi c’est important :** chaque WP peut développer son nœud sans casser les autres, tant que ces messages restent stables.

---

### 5.2 `limo_perception` — L3.1 (détection)

#### A) `lane_detector.py`

- **Entrée :** `/rgb/image_raw` (image caméra simu).  
- **Sortie :** `/perception/lane` (`LaneInfo`).  
- **Méthode :**
  1. Cherche une **ligne jaune** en HSV dans la partie basse de l’image.  
  2. Si échec, estime le centre de la **route sombre**.  
  3. Calcule un écart latéral normalisé (`lateral_offset`) et une approximation de `heading_error`.  
  4. Publie une `confidence` (0 si rien trouvé).

C’est une détection **vision classique OpenCV**, adaptée à la piste jaune de la simu Room207 — pas encore un réseau de neurones.

#### B) `obstacle_detector.py`

- **Entrée :** `/scan` (LiDAR).  
- **Sorties :**
  - `/perception/obstacle_stop` (`Bool`) — vrai si un obstacle est trop près devant ;
  - `/perception/obstacles` (`ObstacleArray`).  
- **Méthode :** parcours des faisceaux LiDAR dans un cône avant (`front_angle`), distance d’arrêt paramétrable (`stop_distance`, défaut ~0,55 m).

#### C) `sign_detector.py`

- **Entrée :** `/rgb/image_raw`.  
- **Sorties :** `/perception/sign_stop`, `/perception/signs`.  
- **Méthode :** détection d’une **tache rouge** (HSV) dans une fenêtre centrale de l’image → considéré comme Stop (version simplifiée pour la simu).

**Limite honnête :** ce n’est pas une reconnaissance robuste de panneau (pas de YOLO / template matching avancé). C’est un **prototype fonctionnel** pour L3.1 en simulation.

---

### 5.3 `limo_control` — L2.1 (commande)

#### A) `polar_controller.py` (cœur contrôle)

Reprend la **loi polaire du cours Commande Numérique** / Simulink année dernière :

\[
v = s \cdot k_\rho \cdot \rho,\quad
\omega = s \cdot (k_\alpha \alpha + k_\beta \beta)
\]

avec \(s = \pm 1\) selon que la cible est devant ou derrière (`alpha`).

Gains par défaut (comme `fichierlimo.m`) :

- `k_rho = 4`  
- `k_alpha = 40`  
- `k_beta = -10`

**Entrées :**

- `/odom` — pose actuelle ;  
- `/mission/goal` — but optionnel (sinon paramètres `x_star`, `y_star`, `theta_star`) ;  
- `/perception/obstacle_stop`, `/perception/sign_stop` — arrêt ;  
- `/scan` — évitement latéral simple.

**Sortie :** `/cmd_vel`  
En simu Ackermann Gazebo, `angular.z` est converti en **angle de braquage** `γ ≈ arctan(L·ω/|v|)` (+ terme d’évitement).

**Comportements ajoutés pour la simu :**

- saturation `v_max` / `w_max` (vitesse basse pour Gazebo) ;  
- ralentissement près du but (`slow_distance`) ;  
- stop net si `rho` petit ;  
- freinage / contournement léger si obstacle LiDAR proche.

#### B) `lane_follower.py`

- Lit `/perception/lane`.  
- Commande proportionnelle :
  - avance à vitesse constante `speed` ;
  - tourne selon `-kp_lateral * lateral_offset - kp_heading * heading_error`.  
- Utile pour un **suivi de voie pur** (complément au contrôle polaire).

---

### 5.4 `limo_planning` — L2.1 (planification)

#### `road_planner.py`

- Charge la **carte texture** de la piste (`SDCS_MapLayout.png` du package `environment`).  
- Construit une grille de cellules « route libre » (zones non blanches).  
- Planifie un chemin type **A\*** / graphe local.  
- Publie des buts `PoseStamped` vers le contrôleur polaire.

Autres fichiers :

- `lane_planner.py` — planification liée à la voie ;  
- `trajectory_tester.py` — outil de test de trajectoire.

**État :** code présent et branchable ; à présenter clairement en démo (chemin planifié visible / buts successifs).

---

### 5.5 `limo_bringup` — intégration

Launch utiles :

| Fichier | Rôle |
| --- | --- |
| `j1_lane_follow.launch.py` | Simu Room207 + 3 détecteurs + `lane_planner` + `polar_controller` + spawn but SDF |
| `perception.launch.py` | Lancer la perception seule |
| `trajectory_test.launch.py` | Tests trajectoire |

Paramètres : `config/j1_params.yaml` (gains + coordonnées du but).  
Modèles SDF de test : `goal.sdf`, `obstacle.sdf`, `stop_sign.sdf`, `point_a.sdf`.

---

## 6. Comment lancer une démo (simulation)

Sous WSL Ubuntu 22.04 :

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Démo phase actuelle (perception + contrôle polaire + simu)
ros2 launch limo_bringup j1_lane_follow.launch.py
```

Vérifications utiles dans un autre terminal :

```bash
ros2 topic list
ros2 topic echo /perception/lane --once
ros2 topic echo /perception/obstacle_stop
ros2 topic echo /cmd_vel
```

---

## 7. Ce qui est **fait** vs **pas encore fait**

### Fait (pour le rendu du 16/11)

- Architecture ROS 2 réelle (packages, messages, nœuds).  
- Perception caméra (voie + Stop simplifié) et LiDAR (obstacle).  
- Contrôle polaire basé sur le cours, branché sur odom + but.  
- Évitement / arrêt d’urgence basique.  
- Launch d’intégration simu Room207.  
- Première planification sur carte (`road_planner`).  
- Partage GitHub pour l’équipe + livrables L0/L1 archivés.

### Pas encore (à dire clairement au prof / à compléter si le temps le permet)

- Détection panneaux **robuste** (feux, Stop réel, classification).  
- Localisation fine sur circuit (fusion odom + IMU + recalage).  
- Machine à états complète (Stop / feux / replanification) type `behavior_manager`.  
- Replanification A* quand un segment est bloqué (boucle fermée planning↔obstacles).  
- Supervision MQTT / InTouch (WP4).  
- Validation systématique sur **véhicule réel** LIMO 204.  
- Jeux de tests / rosbags documentés pour chaque scénario du sujet.

---

## 8. Correspondance avec les consignes du sujet

| Exigence sujet (résumé) | Réponse de cette phase |
| --- | --- |
| Architecture ROS2 (L1.1) | Déjà livrée ; maintenant **implémentée** en packages |
| Identifier capteurs | Caméra RGB + LiDAR utilisés dans les nœuds |
| Prise en main simu | Room207 sous Humble opérationnelle |
| Planification d’itinéraire | `road_planner` sur carte piste |
| Trajectoire / commande | Contrôle polaire + option lane following |
| Validation simu | Launch J1 + topics perception/cmd_vel |
| Détection voies / obstacles / panneaux | 3 nœuds perception (panneaux = version simplifiée) |

---

## 9. Fichiers à connaître pour le rendu / la soutenance courte

1. `docs/deliverables/BE_Vehicule_Autonome_L0.1_L1.1.pdf` — phase 1  
2. **Ce document** — phase 2 (perception / planning / contrôle)  
3. Code : `src/limo_perception/`, `src/limo_control/`, `src/limo_planning/`, `src/limo_bringup/`  
4. README racine du dépôt — installation et vue d’ensemble  

---

## 10. Message clair pour le professeur (version courte)

> Après L0.1/L1.1, nous avons implémenté sous ROS 2 Humble / Gazebo Room207 une chaîne perception → planification/contrôle : détection de voie (OpenCV), obstacles (LiDAR), Stop simplifié (couleur), commande polaire (ρ, α, β) vers un but en simulation, avec arrêt sur obstacle/panneau et un planificateur sur carte. Le code est organisé selon l’architecture L1.1 (`limo_*`). Les prochaines étapes sont la robustesse de la détection, la machine à états et les essais réels.

---

## 11. Décision demandée à Hafsa

Merci de relire ce document et de répondre :

1. **OK pour GitHub** tel quel, ou  
2. **Modifications** (noms, ce qu’on affirme « validé », captures d’écran à ajouter, etc.)

Dès ton OK, on pourra :
- le renommer (enlever `_DRAFT`) ;
- l’ajouter au dépôt `Ibtissam567/LIMO_2026-27` ;
- éventuellement générer un PDF pour Arche / le prof.

---

*Rédigé pour relecture interne — novembre 2026.*
