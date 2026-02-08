# Research Track I – ROS2 Assignment

## Turtle Safety Distance Monitoring and Manual Control

---

## Abstract
This project addresses a basic multi-node coordination problem in ROS2 using the `turtlesim` simulator.
Two turtles are controlled through a combination of manual user input and autonomous safety supervision.
The system continuously monitors the relative distance between the turtles and their position with respect to the workspace boundaries, enforcing safety constraints when predefined conditions are violated.

---

## 1. Introduction
The objective of this assignment is to design and implement a ROS2-based system that combines user-driven control with autonomous safety mechanisms.
The project is developed as part of the **Research Track I – ROS2** course and aims to demonstrate correct usage of ROS2 communication primitives, node coordination, and modular software design.

The `turtlesim` environment is used as a simplified simulation framework to test interaction between multiple nodes and real-time feedback control.

---

## 2. System Architecture
The system is implemented in **ROS2 (Python)** following a publisher–subscriber architecture.
Each functionality is encapsulated in a dedicated node to ensure modularity, scalability, and clarity.

### 2.1 Nodes

- **turtlesim_node**  
  Standard ROS2 node providing the simulation environment and publishing turtle pose information.

- **simple_spawner**  
  Responsible for spawning a second turtle (`turtle2`) at runtime using ROS2 services.

- **ui_node**  
  Implements a terminal-based user interface that allows the user to select a turtle and specify linear and angular velocity commands.
  Each command is applied for a fixed duration of one second.

- **distance_node**  
  Subscribes to the pose topics of both turtles, computes their Euclidean distance, publishes it, and enforces safety constraints.

---

## 3. ROS2 Interfaces

### 3.1 Topics

| Topic Name | Message Type | Description |
|-----------|--------------|-------------|
| `/turtle1/pose` | turtlesim/msg/Pose | Pose of turtle1 |
| `/turtle2/pose` | turtlesim/msg/Pose | Pose of turtle2 |
| `/turtle1/cmd_vel` | geometry_msgs/msg/Twist | Velocity command for turtle1 |
| `/turtle2/cmd_vel` | geometry_msgs/msg/Twist | Velocity command for turtle2 |
| `/turtles_distance` | std_msgs/msg/Float32 | Euclidean distance between the two turtles |

---

## 4. Safety Strategy
The safety logic is fully handled by the `distance_node` and is based on the following rules:

- If the distance between the two turtles becomes smaller than a predefined threshold, the **moving turtle** is stopped (if both turtles are moving, both are stopped).
- If a turtle approaches the boundaries of the simulation workspace, the **moving turtle** close to the boundary is stopped.
- The distance between the turtles is continuously published for monitoring and debugging purposes.
- Safety constraints are enforced continuously, preventing user commands from overriding safety conditions.

This design ensures that safety constraints are enforced independently of user input.

---

## 5. Package Structure

assignment1_rt  
├── assignment1_rt  
│   ├── __init__.py  
│   ├── ui_node.py  
│   ├── distance_node.py  
│   └── turtle_spawn.py  
├── setup.py  
├── package.xml  
└── README.md  

---

## 6. Build Instructions
From the ROS2 workspace root directory, build the package using:

```bash
colcon build
source install/local_setup.bash
```
---

## 7. Execution Instructions
After building the workspace, open four terminals and source ROS2 in each one:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/local_setup.bash
```
Run the nodes in the following order:
### 1. Start the simulation environment

```bash
ros2 run turtlesim turtlesim_node
```
### 2. Spawn the second turtle

```bash
ros2 run assignment1_rt spawn2
```
### 3. Start the distance monitoring and safety node

```bash
ros2 run assignment1_rt distance
```
### 4. Start the manual control interface

```bash
ros2 run assignment1_rt ui
```
The system is now fully operational and ready for user interaction.

## 8. Discussion
The proposed architecture cleanly separates user interaction from safety supervision.
By delegating safety enforcement to a dedicated node, the system remains robust even in the presence of unsafe or inconsistent user commands.

The modular design allows for straightforward extensions, such as:
- Dynamic safety thresholds
- Autonomous navigation behaviors
- Integration with additional sensing or control nodes

This structure reflects common design practices in larger-scale robotic systems.

## 9. Conclusion

This assignment demonstrates a correct and structured use of fundamental ROS2 concepts, including:
- Node modularity
- Topic-based communication
- Runtime coordination between multiple components

The integration of manual control with autonomous safety enforcement provides a clear example of reactive robotic system design and highlights the advantages of distributed control architectures in ROS2.

## 10. Notes

The project was developed and tested on **ROS2 Jazzy**.
All nodes are implemented in **Python** using the `rclpy` library.
Safety enforcement is fully autonomous and does not rely on user cooperation.
