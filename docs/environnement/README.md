# Préparer son PC — première fois

Ce guide est pour quelqu’un qui n’a jamais utilisé ROS 2 ni Gazebo.  
Le but est d’avoir, sur Windows, la même simulation que l’équipe : la petite voiture LIMO dans la salle 207, qui roule sur la route noire jusqu’au point vert.

Dépôt à utiliser : https://github.com/Ibtissam567/LIMO_26-27

Ne clonez pas l’ancien dépôt `LIMO_2026-27`.

---

## Les mots qu’on va voir

| Mot | Ce que c’est, concrètement |
| --- | --- |
| **WSL** | Un Ubuntu qui tourne à l’intérieur de Windows. Les commandes Linux se tapent là, pas dans PowerShell. |
| **ROS 2 Humble** | Le logiciel qui fait parler les programmes entre eux (caméra, calcul, moteurs). |
| **Gazebo Classic** | La fenêtre 3D. C’est le monde virtuel de la salle 207. |
| **Nœud** | Un programme ROS. Exemple : le détecteur de route, le contrôleur. |
| **Topic** | Le tuyau entre deux nœuds. Exemple : `/cmd_vel` est l’ordre de vitesse envoyé à la voiture. |
| **Workspace** | Le dossier `~/ros2_ws` qui contient le code et le résultat de la compilation. |
| **colcon** | La commande qui compile le workspace. |

On utilise **Ubuntu 22.04** et **ROS 2 Humble**.  
Ubuntu 24.04 et ROS 2 Jazzy ne lancent pas cette simulation. Si Windows a déjà installé un Ubuntu « tout court », ce n’est en général pas le bon.

---

## 1. Installer Ubuntu 22.04 dans Windows

Ouvrir **PowerShell** et taper :

```powershell
wsl --install -d Ubuntu-22.04
```

Redémarrer l’ordinateur si Windows le demande. Au premier lancement, Ubuntu demande un nom d’utilisateur et un mot de passe Linux. Retenir ce mot de passe : il sera redemandé par `sudo`.

Vérifier ensuite :

```powershell
wsl -d Ubuntu-22.04
```

Dans Ubuntu :

```bash
lsb_release -a
```

La ligne `Release` doit afficher `22.04`. Si vous voyez `24.04`, vous êtes dans le mauvais Ubuntu. Fermer et relancer avec `wsl -d Ubuntu-22.04`.

---

## 2. Installer ROS 2 Humble et Gazebo

Toujours dans Ubuntu 22.04. Copier le bloc en entier.

```bash
sudo apt update
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo apt update

export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $UBUNTU_CODENAME)_all.deb"
sudo apt install -y /tmp/ros2-apt-source.deb

sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools ros-humble-gazebo-ros-pkgs
```

À la fin de chaque ouverture d’Ubuntu, ROS doit être chargé. Ajouter ces lignes à la fin de `~/.bashrc` :

```bash
source /opt/ros/humble/setup.bash
export DISPLAY=:0
export GAZEBO_MODEL_DATABASE_URI=
```

Puis :

```bash
source ~/.bashrc
echo $ROS_DISTRO
```

La réponse doit être `humble`.

`DISPLAY=:0` sert à afficher la fenêtre Gazebo sur Windows.  
`GAZEBO_MODEL_DATABASE_URI` vide évite que Gazebo essaie de télécharger des modèles sur Internet.

---

## 3. Récupérer le code

### Le code de l’équipe (GitHub)

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Ibtissam567/LIMO_26-27.git
```

### Les paquets de l’école (GitLab)

Ces dossiers sont lourds. Ils ne sont pas dans GitHub. Il faut un compte sur https://gitlab.univ-lorraine.fr et l’accès au groupe `robotic_lab`.

```bash
cd ~/ros2_ws/src
git clone https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/limo.git
git clone https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/environment.git
git clone https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/teleoperation.git
git clone https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/recording.git
git clone https://gitlab.univ-lorraine.fr/coll/l-inp/ensem/robotic_lab/students/be-2026-groupe-4/room207-limo.git
```

Le modèle du robot est rangé dans le dépôt `limo`. Créer le lien que les fichiers de lancement attendent :

```bash
ln -s ~/ros2_ws/src/limo/ros2_humble/limo_ros2 ~/ros2_ws/src/limo_ros2
```

---

## 4. Compiler

`setup.bash` n’est **pas** dans GitHub et **pas** dans les dépôts des enseignants.  
Ces dépôts contiennent le code source. Le fichier `setup.bash` est fabriqué par la compilation, sur votre PC, dans `~/ros2_ws/install/`.

Si ce dossier `install` ne contient que `.colcon_install_layout` et `COLCON_IGNORE`, la compilation n’a trouvé aucun programme. Elle a été lancée dans le mauvais dossier. On efface ce résultat vide et on recommence **depuis** `~/ros2_ws`.

Dans Ubuntu, pas dans PowerShell :

```bash
rm -rf ~/ros2_ws/install ~/ros2_ws/build ~/ros2_ws/log
rm -rf ~/ros2_ws/src/install ~/ros2_ws/src/build ~/ros2_ws/src/log
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
pwd
```

`pwd` doit afficher un chemin qui finit par `/ros2_ws`. Pas par `/ros2_ws/src`.

Vérifier que le code est bien là :

```bash
ls src/LIMO_26-27/src/limo_bringup/package.xml
ls src/room207-limo
ls src/limo
```

Chaque `ls` doit afficher un fichier ou un dossier, pas `No such file or directory`.  
Si un dépôt école manque, refaire son `git clone` du paragraphe 3. GitLab demande le compte universitaire.

Puis :

```bash
cd ~/ros2_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon list
colcon build --symlink-install
ls ~/ros2_ws/install/setup.bash
```

Si `sudo rosdep init` répond que rosdep est déjà initialisé, ce n’est pas une erreur. Continuer.

`colcon list` doit afficher des lignes comme `limo_bringup` et `room207_limo`.  
S’il n’affiche rien, ne pas continuer : vous n’êtes pas dans `~/ros2_ws`, ou les clones sont vides.

`ls ~/ros2_ws/install/setup.bash` doit afficher le chemin du fichier. **Seulement après ça**, ajouter cette ligne dans `~/.bashrc`, sous la ligne Humble :

```bash
source ~/ros2_ws/install/setup.bash
```

Fermer Ubuntu et le rouvrir, ou taper `source ~/.bashrc`.

---

## 5. Lancer la simulation

`ros2: command not found` veut dire que la commande a été tapée au mauvais endroit.  
`ros2` existe seulement **dans Ubuntu 22.04**, après `source /opt/ros/humble/setup.bash`.  
PowerShell ne connaît pas `ros2`. On ne colle pas `wsl` et `ros2` dans le même bloc.

S’il y a déjà une fenêtre Gazebo, cliquer dans son terminal et faire **Ctrl+C**. Attendre qu’il s’arrête. Deux Gazebo en même temps se bloquent.

**Étape A — PowerShell**, une seule ligne. Elle ouvre Ubuntu et s’arrête. Le prompt doit ressembler à `nom@PC:~$`.

```powershell
wsl -d Ubuntu-22.04
```

**Étape B — dans cet Ubuntu**, coller ceci. Ne pas le coller dans PowerShell.

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export DISPLAY=:0
export GAZEBO_MODEL_DATABASE_URI=
which ros2
ros2 launch limo_bringup j1_lane_follow.launch.py
```

`which ros2` doit afficher `/opt/ros/humble/bin/ros2`.  
S’il n’affiche rien, ROS 2 Humble n’est pas installé dans cet Ubuntu : refaire le paragraphe 2, puis `echo $ROS_DISTRO` doit répondre `humble`.

Une fenêtre 3D s’ouvre sur **toute la salle 207**. C’est le nouveau monde du professeur (`ensem_room207`) : la piste noire est la même qu’avant, posée sur une plateforme de 20 cm, et la salle est autour. La voiture est le petit objet noir sur la piste. Les traits rouge, vert et bleu sont les axes de Gazebo.

Pour voir la voiture : placer la souris sur la piste, tourner la **molette** pour zoomer, et garder le **clic gauche** enfoncé pour tourner la vue. La voiture part du bas de la piste (point A) et va vers le marqueur vert (point B) en restant sur la route **noire**. Le blanc et le gris sont interdits.

La voiture avance lentement, environ 5 cm par seconde, pour qu’on puisse suivre le trajet. Regarder la fenêtre dès qu’elle apparaît : si on attend trop, la voiture est déjà arrivée au point vert.

Pour arrêter : **Ctrl+C** dans le terminal qui a lancé la commande.

---

## 6. Changer le point A et le point B

Les coordonnées sont en mètres. Les deux points doivent être sur la route noire. Un point sur le blanc ou sur le gris fait échouer le calcul du chemin.

**Point A**, l’endroit où la voiture apparaît. Depuis la mise à jour du professeur, le fichier est ici :

`~/ros2_ws/src/room207-limo/ros2_foxy/room207_limo/launch/room207_limo.launch.py`

```python
'-x', '-0.725347',
'-y', '-0.086613',
'-z', LaunchConfiguration('spawn_z'),
'-Y', '1.5708',
```

La hauteur par défaut `spawn_z` est `0.35` m : la piste est sur une plateforme de 20 cm. Ne pas mettre `0.15` dans cette salle, la voiture apparaîtrait dans la plateforme.

`-Y` est la direction du regard. `1.5708` regarde vers le haut de la carte, `0` vers la droite, `3.1416` vers la gauche, `-1.5708` vers le bas.

**Point B**, la destination. Fichier du dépôt équipe :

`~/ros2_ws/src/LIMO_26-27/src/limo_bringup/launch/j1_lane_follow.launch.py`

Changer les **trois** endroits avec les mêmes `x` et `y`, sinon la voiture et le marqueur vert ne visent pas le même point.

```python
"goal_x": -0.725, "goal_y": 2.413
```

Cette ligne est sur `lane_planner` et sur `road_planner`. Le marqueur vert est juste en dessous :

```python
"-x", "-0.725",
"-y", "2.41",
```

Enregistrer, **Ctrl+C**, puis relancer la commande du paragraphe 5. Pas besoin de recompiler pour un fichier `.launch.py` si le workspace a été compilé avec `--symlink-install`.

---

## 7. Si l’installation date d’avant la salle 207

Le professeur a ajouté le monde `ensem_room207` et a déplacé le paquet `room207_limo` dans `ros2_foxy`. Quelqu’un qui a cloné avant cette mise à jour voit encore l’ancienne piste seule, ou un `colcon build` qui échoue parce que l’ancien dossier de compilation pointe au mauvais endroit.

Dans Ubuntu :

```bash
cd ~/ros2_ws/src/environment
git pull
cd ~/ros2_ws/src/room207-limo
git pull
cd ~/ros2_ws
rm -rf build/room207_limo install/room207_limo
colcon build --symlink-install --packages-select ensem_room207 room207_limo
source install/setup.bash
```

`git pull` demande l’identifiant GitLab de l’école. Ne pas pousser ces dépôts école : on ne fait que télécharger.

---

## 8. Travailler avec les autres

```bash
cd ~/ros2_ws/src/LIMO_26-27
git pull
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Faire ses modifications sur une branche, pas directement sur `main` :

```bash
git checkout -b mon-essai
git add -A
git commit -m "Décris ce que tu as changé"
git push -u origin mon-essai
```

Puis ouvrir une Pull Request sur GitHub.

---

## Si quelque chose ne marche pas

| Ce qu’on voit | Quoi vérifier |
| --- | --- |
| `ros2: command not found` | La commande a été tapée dans PowerShell, ou le `source` de Humble n’a pas été fait dans Ubuntu. Refaire l’étape A puis l’étape B du paragraphe 5. |
| `No such file or directory` sur `install/setup.bash` | Ce fichier n’est pas téléchargé. Il apparaît seulement après un `colcon build` lancé dans `~/ros2_ws`. Refaire le paragraphe 4. |
| `Package 'limo_bringup' not found` | Le `colcon build` n’a pas fini, ou `source ~/ros2_ws/install/setup.bash` manque. |
| La fenêtre Gazebo ne s’ouvre pas | `echo $DISPLAY` doit afficher `:0`. Relancer avec `export DISPLAY=:0`. |
| Gazebo reste sur « loading » | Un autre Gazebo tourne encore. **Ctrl+C**, attendre, relancer une seule fois. |
| `no on-road path` | Le point A ou B n’est pas sur la route noire. |
| La voiture est déjà au point vert | Le trajet a fini pendant l’ouverture de la fenêtre. **Ctrl+C**, relancer, et regarder tout de suite. |
