# mantis-6-axis-robot

<img width="2160" height="1050" alt="mantis_main_render" src="https://github.com/user-attachments/assets/c6daedae-1054-4b2f-a0e7-6cacd50ec949" />

## Project Overview
This project is a comprehensive showcase of my robotics engineering skills, built completely from scratch. It mimics industrial robotic arms in a hobbyist-friendly form factor. The project covers the full development lifecycle: mechanical design, CAD modeling, simulation setup in ROS2, motion planning, and custom control software.

## Tech Stack & Tools
* **ROS2 Distribution:** Jazzy
* **Simulation & Visualization:** Gazebo Harmonic, RViz2
* **Motion Planning:** MoveIt 2, ROS2 control
* **CAD Design:** Autodesk Fusion 360
* **Programming Languages:** C++ / Python

## Key Features

1. **Custom Mechanical Design (Fusion 360):**
   * Designed a 6-DOF anthropomorphic robotic arm from scratch, focusing on realistic industrial kinematics.
   * Engineered a custom **pantograph-based gripper** operating fully in Gazebo. It ensures adaptive gripping for various parts while keeping the jaws parallel at any deviation angle.

2. **ROS2 Integration & Simulation:**
   * Exported the CAD model into URDF/Xacro and integrated it into **ROS2 Jazzy**.
   * Set up comprehensive visualization and state monitoring in **RViz2** and physics simulation in **Gazebo**.

3. **Motion Planning & Control (MoveIt 2):**
   * Configured MoveIt 2 for collision-free trajectory generation and motion planning with gripped objects.
   * Developed a lightweight custom helper library to streamline motion implementation.

4. **Virtual Teach Pendant & YAML Waypoint Recorder:**
   * Created a custom program acting as a "virtual teach pendant."
   * Allows saving the robot's current TCP (Tool Center Point) positions as waypoints directly into a **YAML configuration file**, which can be easily parsed later for automated trajectory execution.


## Details and Showcase

1. **Custom Mechanical Gripper.**
   The gripper mechanism works as simple mechanical pantograph steered by servo. Because of it the jaws are always parallel. The possible maximum position is 45 deg, in which it can lifts objects with 67 mm diameter.


https://github.com/user-attachments/assets/b4b6d692-747e-4741-b7bf-9fbcbc70432a

2. **Example program showcase.**

> [!NOTE]
> Remember to build projects and source all terminals before launching any projects.

To launch example program, you need run commands as below in correspodning terminals.
> [!TIP]
> You can use **terminator** program in linux to do it easily.


>[!IMPORTANT]
>Remember to run second command after gazebo simulation will be launched and fully operational.
 
```
ros2 launch mantis_bringup mantis_moveit_gazebo.launch.py
ros2 run mantis_commander main_planner_node --ros-args -p use_sim_time:=true
```

>[!WARNING]
>Always use parameter **use_sim_time:=true** when working with gazebo simulation. The ik solver might not find correct tcp point in time and not calculate real path if this parameter is not present!

After doing steps above you should see something like this:

https://github.com/user-attachments/assets/07d9f59e-9299-4c95-9c0d-f62dd1b11de0





