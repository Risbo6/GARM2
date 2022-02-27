# GARM 2

Projet de master 2022

## Installation

Cloner le contenu du dossier ROS dans :

```bash
catkin_ws/src
```

Construire le [catkin](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) workspace en faisant :

```bash
cd ~/catkin_ws/
catkin_make
```

## Utilisation - GUI

Depuis le robot, lancer :

```bash
roslaunch garm2 server.launch
```

Depuis la station, lancer :

```bash
roslaunch gui client.launch ip:=ip_du_robot
```

Le robot et la station doivent être sur le même réseau, soit local, soit un VPN. Remplacer ip_du_robot par l'ip du robot.


## Utilisation - Path planning

Depuis une station connectée au ROS network, lancer :

```bash
roslaunch garm2 odom_filter.launch
```

Pour naviguer sans constuire de map, lancer :

```bash
roslaunch garm2 autonomous_blind.launch
```

Pour naviguer avec une map pré-construite au préalable, lancer :

```bash
roslaunch garm2 autonomous_map_predefined.launch
```

Pour créer une map qui sera sauvegardable tout en naviguant, lancer :

```bash
roslaunch garm2 autonomous_gmapping.launch
```

## Webots

Il est possible de simuler le robot en utilisant Webots. Les fichiers sont trop gros pour être upload sur github, merci de me contacter.

