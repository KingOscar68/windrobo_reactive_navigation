WindRobo Reactive Navigation Project

📌 Description

This project implements a reactive navigation controller for a TurtleBot3 Burger robot using ROS 2.
The robot navigates toward a user-defined goal while avoiding randomly generated obstacles in a Gazebo simulation.

The navigation strategy is based on:

Goal-oriented motion control

Reactive obstacle avoidance using LaserScan data

Smooth velocity modulation near obstacles

🖥 System Information

Ubuntu: 22.04.5 LTS (Jammy)

ROS 2 Distribution: Humble Hawksbill

Gazebo Version: 11.10.2

Robot Model: TurtleBot3 Burger

src/

 ├── windrobo_cpp/        # Reactive controller (C++)
 
 ├── windrobo_py/         # Goal publisher & waypoint tools (Python)
 
 ├── windrobo_sim/        # Random obstacle generator
 
 ├── windrobo_msgs/       # Custom ROS messages
 
 └── windrobo_bringup/    # Launch configuration
 

 ⚙️ Installation
 
1️⃣ Clone the repository

git clone https://github.com/KingOscar68/windrobo_reactive_navigation.git

cd windrobo_reactive_navigation

2️⃣ Build the workspace

colcon build

source install/setup.bash


🚀 Execution

Terminal 1 – Launch simulation and controller

export TURTLEBOT3_MODEL=burger

ros2 launch windrobo_bringup navigation.launch.py

Terminal 2 – Send goal coordinates

source install/setup.bash

ros2 run windrobo_py goal_publisher

Then enter the desired final goal coordinates in the terminal.

🎯 Features

Smooth reactive obstacle avoidance

Continuous angular correction

Velocity scaling based on obstacle proximity

Clean modular ROS2 architecture

Single-command launch support
