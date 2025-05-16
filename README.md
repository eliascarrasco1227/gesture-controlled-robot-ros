# Gesture-based Teleoperation System with Obstacle Avoidance

[![ROS](https://img.shields.io/badge/ROS-Noetic-brightgreen)](https://www.ros.org/)
[![MediaPipe](https://img.shields.io/badge/MediaPipe-v0.10.3-blue)](https://mediapipe.dev/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

**Práctica**: Sistema de teleoperación mediante gestos para controlar un robot-coche en ROS (Robot Operating System). El sistema permite:
- Controlar el movimiento del robot (avance, giros izquierda/derecha) mediante gestos con la mano (dedo índice y pulgar extendidos)
- Integra un sistema de seguridad con evitación de obstáculos usando LIDAR
- Arquitectura modular basada en ROS para procesamiento en tiempo real

![System Overview](picture.png)

## Características Principales
- **Control Intuitivo**:
  - 👆 Señalar hacia arriba → Avance
  - 👈 Señalar izquierda → Giro izquierda
  - 👉 Señalar derecha → Giro derecha
- **Seguridad**:
  - Detección de obstáculos con LIDAR
  - Sistema de parada de emergencia
- **Arquitectura**:
  - 4 nodos ROS independientes
  - Procesamiento a 30 FPS

## Instalación
```bash
git clone https://github.com/username/gesture-teleop-ros.git
cd gesture-teleop-ros
sudo apt install ros-noetic-cv-bridge ros-noetic-ackermann-msgs
pip install mediapipe opencv-python
```
## uso
```bash
roslaunch gesture_control teleop.launch
```
En otra consola
```bash
roslaunch robotica_inteligente load_scene.launch
```
