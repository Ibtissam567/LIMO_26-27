# Avancement détaillé (notes pour l’équipe)

## Environnement machine (Hafsa)

- **Ubuntu 24.04 / ROS 2 Jazzy** : OK pour les TP intro ROS, **pas** adapté à Room207 (Gazebo Classic manquant).
- **Ubuntu 22.04 / ROS 2 Humble** (WSL `Ubuntu-22.04`) : environnement utilisé pour la simu LIMO Room 207.
- Commande d’ouverture : `wsl -d Ubuntu-22.04`

## Livrables

- **L0.1** : planning prévisionnel + répartition WP0–WP4.
- **L1.1** : architecture ROS2 (SysML bdd/ibd/stm + packages/nœuds/topics).
- Fichiers dans `docs/deliverables/`.

## Contrôle (lien avec le cours)

Le nœud `polar_controller` reprend la loi du polycopié **CMD NUM** / Simulink année dernière :

- `v = sens * k_rho * rho`
- `omega = sens * (k_alpha * alpha + k_beta * beta)`
- Gains par défaut : `k_rho=4`, `k_alpha=40`, `k_beta=-10` (comme `fichierlimo.m`)
- `sens = ±1` selon que la cible est devant / derrière

## Prochaines étapes suggérées

1. Compléter `docs/group-informations/README.md` (rôles de chacun).
2. Brancher réellement `lane_detector` sur `/camera/...` et publier `/perception/lane`.
3. Brancher `lane_follower` / `polar_controller` sur odom + perception.
4. Valider J1 en simu : véhicule suit une voie.
5. Pousser régulièrement sur ce dépôt pour qu’Ibtissam voie l’avancement.
