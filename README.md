# TurtleBot4 ROS2 - setup and navigation

## Warning

About the QR Code detection and scan with BarCode library, you need a license key.
We already requested and inserted inside the code a valid key on 9th of July, it should expire after 30 days.
In case you could have problems with the validation key, we suggest to make a new request to Dynamsoft website.
Moreover this repository contains only the code developed integrated with the code available on the Turtlebot4 documentation , in order to 

## Introduction

This folder contains shell scripts to set up and run the TurtleBot4 environment and nodes using ROS2.
The scripts are divided into three main sections:
1. **Setup script**
2. **Navigation script**
3. **Nodes execution script**

## 1. Setup script (`setup.sh`)

This bash script runs a series of commands to set up the ROS2 environment and the TurtleBot4, making sure everything is set up correctly for development and execution.

First, the script prints a message to the terminal to indicate that the installation process has begun. Then, generate the ROS2 installation script, which sets the environment variables and paths needed for ROS2. This step ensures that ROS2 commands and functionality are available in your terminal session.

Finally, the TurtleBot4 workspace configuration script is generated. This action configures the environment to recognize and use custom packages and workspace-specific dependencies. It ensures that all TurtleBot4-specific nodes and startup files can run without any problems.

### Usage

To use the setup script, run the following command:

```
"bash."
./setup.sh
```

## 2. Navigation script (`navigation.sh`)

This bash script automates the process of starting the TurtleBot4 navigation system using ROS2. Opens multiple terminal windows, each dedicated to launching a specific component required for navigation and viewing.

### Terminal 1: Setting the localization
The first terminal focuses on loading a map and starting the location process. The script generates the necessary ROS2 installation files and runs `localization.launch.py` with a specified map (`diem_map.yaml`). This ensures that the robot can locate itself precisely.

### Terminal 2: Initializing the navigation stack
The second terminal is responsible for launching the navigation stack. By running `nav2.launch.py`, the script sets up global and local planners, cost maps, and other navigation-related components, allowing the robot to navigate autonomously.

### Terminal 3: Visualization with RViz
The third terminal opens RViz. The script runs `view_robot.launch.py` to launch RViz, giving users a way to view and debug the robot's operations.
Each terminal includes a `sleep 5` command to introduce a delay, allowing sufficient time for each process to initialize before moving on to the next. This prevents racing conditions and ensures that all components start correctly. Additionally, the script prints status messages to the terminal, providing feedback on the progress of each step.

### Usage

To use the navigation script, run the following command:

```
"bash."
./navigazione.sh
```

## 3. Nodes Script (`nodes.sh`)
This bash script is designed to simplify the TurtleBot4 robot setup process by automating the compilation and running of two key nodes: `camera_node` and `navigation_node`.

### Terminal 1
The first terminal is dedicated to the `camera_node`. The script begins by navigating to the TurtleBot4 workspace directory and then runs the `colcon build` command to build all the necessary packages. After configuring the environment by retrieving the ROS2 configuration file, the script starts `camera_node` using `ros2 run`. This ensures that the camera functionality is fully operational, allowing the robot to process visual data from its surroundings. A 5 second delay is included to provide sufficient time for processes to finish successfully before the terminal session is closed.

### Terminal 2:
The second terminal focuses on the `navigation_node`, which is critical to the robot's navigation capabilities. Similar to the first terminal, the script navigates to the workspace, builds the necessary packages using `colcon build`, and configures the environment by retrieving the ROS2 installation file. Then start `navigation_node` using `ros2 run`. A 5 second delay is also included here to allow all processes to close.

### Terminal 3:
The third terminal runs the `discovery_node`, which is responsible for the robot's discovery capabilities. The script accesses the TurtleBot4 workspace, compiles the necessary packages, configures the environment, and then launches the `discovery_node`. The node subscribes to the `/discovery` topic and, after receiving a message, commands the robot to rotate 45 degrees to the left, then 90 degrees to the right and finally another 45 degrees to the left, with short pauses in between. The 5 second delay for processes to properly terminate is also included in this script.

### Usage

To use the `nodes.sh` script, run the following command:

```
"bash."
./nodes.sh
```
