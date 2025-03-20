# **TIAGO 2D Navigation & ArUco Detection**

## 👨‍🎓 **Étudiants**
LAPORTE Maxime, DELLAU Landry

---

## 📄 **Résumé du projet**
Ce projet implémente un **système de navigation 2D pour le robot Tiago** en utilisant **ROS 2** et un détecteur de **tags ArUco**.  
L'objectif est d'intégrer un contrôle de navigation basé sur la vision, permettant au robot de se déplacer et d'interagir avec des objets marqués.

---

### 🛠 **Installation**
Remplace le dossier `tiago_2dnav` dans :  
```bash
cd ros2_ws/src/tiago_navigation
```
Vérifie que ton environnement ROS 2 est bien configuré avec `tiago_gazebo` et `aruco_detector`.

---

### ▶️ **Lancer le projet**

#### 1️⃣ **Configurer l'environnement**
Dans chaque terminal, source l'environnement ROS 2 :  
```bash
source ~/ros2_ws/install/setup.bash
```

#### 2️⃣ **Démarrer la simulation Tiago dans Gazebo**
Dans le **premier terminal** :  
```bash
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=pick_and_place
```

#### 3️⃣ **Lancer le détecteur de tags ArUco**
Dans le **deuxième terminal** :  
```bash
ros2 launch aruco_detector aruco_launch.py
```

---

## 📚 **Références et bibliographie**
🔗 **Dépôt GitLab du projet** :  
[ROS2 Jazzy - Robots DevContainer](https://gitlab.com/f2m2robserv/jazzy-ros-ynov/-/tree/main?ref_type=heads#ros2-jazzy--robots-devcontainer)

📄 **Documentation ROS 2 & Tiago** :  
- [Tiago Tutorials](https://github.com/pal-robotics/tiago_tutorials)  
- [ArUco Marker Detection](https://github.com/ros-perception/vision_opencv)

---
