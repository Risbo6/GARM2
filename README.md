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

