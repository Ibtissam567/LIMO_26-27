# MATLAB — réponses du groupe (sur la base fournie par les enseignants)

Ce dossier est le **MATLAB / Simulink du dépôt école `limo`**, que le professeur nous a demandé d’utiliser et de **compléter**.

Nous n’avons **pas** rendu le vieux projet Simulink de l’année dernière (`fichierlimo.m` / `limo.slx` d’un autre groupe) comme livrable : ce n’était qu’une **référence de cours** (loi polaire).  
Le travail attendu est bien **celui du site / dépôt `robotic_lab/limo/matlab`**, rempli avec notre commande.

---

## 1. Ce que le professeur fournit (ne pas réinventer)

| Élément | Rôle |
| --- | --- |
| `limo_lib.slx` / `limo_gazebo.slx` | Modèle LIMO proche de Gazebo |
| `+limo/parameters.m`, `plant_dynamics.m`, … | Paramètres et dynamique |
| `+limo/simulate_gazebo.m` | Simulation MATLAB sans ROS |
| `run_limo_gazebo.m` | Démo scénario ouvert |
| `compare_with_gazebo.m` | Comparaison bag Gazebo ↔ MATLAB |
| `README.md` | Notice officielle |

Voir aussi le README officiel dans ce même dossier.

---

## 2. Ce que notre groupe a ajouté / rempli

### A) Loi de commande polaire — `+limo/polar_command.m`

Implémentation de la loi du cours CMD NUM :

```text
v     = sens * k_rho   * rho
omega = sens * (k_alpha * alpha + k_beta * beta)
```

- `sens = +1` si la cible est devant (`|alpha| ≤ π/2`), sinon `-1` (recul).  
- Arrêt si `rho < stop`.  
- Saturation `v_max` / `w_max`.  
- Ralentissement près du but (`rho < 0.5`).

Sortie : `u = [v; omega]` en mode **`cmd_vel`** pour `limo.simulate_gazebo`.

### B) Script de test groupe — `run_polar.m`

- Point **A** = pose spawn Room 207 (`p.initial.room207_pose`).  
- Point **Z** = 2,5 m devant, même cap.  
- Gains retenus (stables) :

| Gain | Valeur | Commentaire |
| --- | --- | --- |
| `k_rho` | `4` | comme le cours / fichierlimo de référence |
| `k_alpha` | `40` | `k_alpha > k_rho` |
| `k_beta` | **`-10`** | valeur stable ; un `k_beta` trop négatif (ex. −1e5) fait tourner le robot en rond |
| `v_max` | `0.30` | |
| `w_max` | `0.80` | |
| `stop` | `0.10` m | critère d’arrivée |

Le script lance `limo.simulate_gazebo`, affiche la trajectoire A→Z et l’erreur finale.

### C) Lien avec ROS 2 (même loi)

La même commande est dans :

`src/limo_control/limo_control/polar_controller.py`

→ MATLAB sert à **valider / régler** la loi sur le modèle du prof ; ROS sert à la **démo Gazebo Room207** et aux livrables L2.1 / L3.1.

---

## 3. Comment lancer (côté MATLAB)

```matlab
cd('.../LIMO_2026-27/matlab')   % ce dossier
addpath(pwd)

% 1) Démo modèle enseignant (scénario ouvert)
run_limo_gazebo

% 2) Notre réponse : commande polaire A → Z
run_polar
```

Besoin : MATLAB + Simulink (idéalement R2026a/b selon le README enseignant).

---

## 4. Message clair pour le professeur

> Nous sommes partis du package MATLAB fourni dans `limo/matlab`.  
> Nous y avons ajouté `polar_command.m` et le script `run_polar.m` (gains stables, trajet A→Z depuis le spawn Room 207).  
> La même loi est utilisée en ROS 2 (`polar_controller`).  
> Nous n’avons pas rendu le Simulink d’un autre groupe d’une année précédente comme solution ; il n’a servi que de référence pédagogique pour la loi polaire.

---

*Groupe BE 2026 — LIMO / Room 207*
