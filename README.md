# Mobile Robots for Critical Missions - TurtleBot4 Navigation System

## Project Overview

This project focuses on developing an autonomous navigation system for the **TurtleBot4** robot. The goal is to enable the robot to navigate through a known map, follow road signs indicated by QR codes at each intersection, and avoid both static and dynamic obstacles during navigation. The project leverages **ROS2** for robot control and **OpenCV** with DynamSoft's BarcodeReader for QR code detection.

### Main Objectives:
1. Autonomous navigation through predefined intersections.
2. QR code-based command interpretation for path selection.
3. Obstacle detection and avoidance.
4. Modular architecture for easy debugging and extension.

## Tools and Technologies

- **ROS2**: An open-source framework for robot development. It provides the infrastructure for communication between various nodes using the publisher-subscriber pattern.
  
- **TurtleBot4**: A mobile robot equipped with sensors like **LIDAR**, distance sensors, an **IMU**, and a high-resolution camera. The base offers a carrying capacity of up to 9 kg and can travel at speeds of up to 0.46 m/s.

- **OpenCV**: Used for image processing, especially QR code detection in this project.

- **DynamSoft BarcodeReader**: A library used for decoding QR codes under challenging conditions like low contrast or partial occlusion.

## System Architecture

The system is built in a modular fashion, where each functionality (e.g., image capture, navigation, discovery) is separated into different ROS2 nodes.

### 1. **Camera Node**
   - Captures and processes images from the TurtleBot4’s camera.
   - Detects and decodes QR codes via the **DynamSoft BarcodeReader**.
   - Publishes decoded commands on the `command` topic.

### 2. **Navigation Node**
   - Responsible for the robot’s movement decisions based on the QR code commands.
   - Subscribes to the `command` topic to receive movement instructions.
   - Handles the robot’s position and orientation on the map, using path planning and obstacle avoidance algorithms.

### 3. **Discovery Node**
   - Manages QR code detection when the camera’s field of view may be obstructed.
   - Rotates the robot left and right to ensure that the camera can scan for QR codes in hard-to-reach areas.
   - This operation is triggered when no signal is detected, and the navigation node requests the robot to perform a discovery operation.

## How It Works

### 1. **QR Code Detection and Command Execution**
   The TurtleBot4 reads QR codes placed at intersections. Each QR code contains instructions such as:
   - **STRAIGHTON**: Proceed straight.
   - **RIGHT**: Turn right.
   - **LEFT**: Turn left.
   - **GOBACK**: Perform a U-turn.
   - **STOP**: Halt the robot.

   When a QR code is detected, the corresponding command is published and interpreted by the navigation node, which then moves the robot accordingly.

### 2. **Graph-Based Path Planning**
   The environment’s intersections are modeled as a **directed graph**, where each node represents an intersection, and each edge represents a path between intersections. The robot uses this graph to navigate efficiently.
   
   - The graph is direction-sensitive, with different graphs generated for each of the four cardinal orientations (NORTH, SOUTH, EAST, WEST).
   - For example, when the robot is oriented NORTH, the possible directions and their respective destinations are determined by the graph corresponding to that orientation.

### 3. **Discovery Operation**
   When the robot arrives at an intersection but does not detect a QR code (e.g., due to obstruction), the system initiates a **discovery operation**:
   - The robot rotates by 45 degrees to the left, checks for a QR code, and then repeats the process on the right.
   - If a code is detected, the command is executed. If no code is detected, the robot returns to its original position.

### 4. **Obstacle Avoidance**
   The system integrates ROS2 navigation packages for obstacle avoidance. Using LIDAR data, the robot detects and navigates around obstacles in real-time while still following the QR code commands.

