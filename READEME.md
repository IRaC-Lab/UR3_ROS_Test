# ur_arm_test

![ROS](https://img.shields.io/badge/ROS-Noetic-blue) ![License](https://img.shields.io/badge/License-MIT-green)

`ur_arm_test` is a ROS package for testing and simulating the control of a Universal Robots (UR) 7-DoF robotic arm. It provides example nodes for various types of joint control, real-time trajectory publishing, and MoveIt integration, mainly for use in Gazebo simulation environments.

## Main Purpose
- **Joint Trajectory Control**: Test joint trajectory commands using action server/client.
- **Real-Time Control**: Example for real-time trajectory publishing.
- **MoveIt Integration**: Example for position control using MoveIt.
- **ROS/Gazebo Practice**: Example nodes for learning and testing UR arm control in ROS and Gazebo.

## Package Structure
```
ur_arm_test/
├── src/
│   ├── test_ur_joint_traj_action.cpp
│   ├── test_ur_joint_traj_realtime.cpp
│   ├── test_ur_joint_trajectory_publisher_node.cpp
│   └── test_ur_position_moveit.cpp
├── include/
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Dependencies
- ROS Noetic
- `roscpp`, `std_msgs`, `sensor_msgs`, `trajectory_msgs`, `control_msgs`, `moveit_ros_planning_interface`, etc.
- Universal Robots model and Gazebo/MoveIt environment from [ros-industrial/universal_robot](https://github.com/ros-industrial/universal_robot)

## Installation & Build

### 1. Install ROS Noetic
Refer to the [official ROS installation guide](http://wiki.ros.org/ROS/Installation).

### 2. Create and Move to Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

### 3. Clone the Packages
Clone both the `universal_robot` package (for simulation and MoveIt config) and this package:
```bash
git clone https://github.com/ros-industrial/universal_robot.git
# Clone the UR3_ROS_TEST package
git clone https://github.com/IRaC-Lab/UR3_ROS_TEST.git
```

### 4. Install Dependencies
```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 5. Build
```bash
catkin_make
source devel/setup.bash
```

## Usage

### 1. Launch Gazebo Simulation and MoveIt (using universal_robot)
- Use the launch files provided by the `universal_robot` package for Gazebo simulation and MoveIt configuration. For example:
```bash
roslaunch ur_gazebo ur5e.launch
# or for MoveIt
roslaunch ur5e_moveit_config ur5e_moveit_planning_execution.launch
```
- Make sure the correct robot model (e.g., ur5e, ur10, etc.) is specified as needed.

### 2. Run Joint Trajectory Action Client Example
```bash
rosrun ur_arm_test test_ur_joint_traj_action
```
- Sends target values to the JointTrajectoryAction server.

### 3. Run Real-Time Joint Trajectory Publisher
```bash
rosrun ur_arm_test test_ur_joint_traj_realtime
```
- Publishes joint trajectory commands in real time.

### 4. Run JointTrajectory Publisher Node
```bash
rosrun ur_arm_test test_ur_joint_trajectory_publisher_node
```
- Publishes trajectory commands for single or multiple joints.

### 5. Run MoveIt-based Position Control Example
```bash
rosrun ur_arm_test test_ur_position_moveit
```
- Uses MoveIt interface for target position control.

## Main Topics & Actions
- **Published**: `/arm_controller/command` (`trajectory_msgs/JointTrajectory`)
- **Action**: `/arm_controller/follow_joint_trajectory` (`control_msgs/FollowJointTrajectoryAction`)
- **MoveIt**: Uses MoveGroup interface

## Notes
- The robot model, Gazebo simulation, and MoveIt configuration are provided by the [ros-industrial/universal_robot](https://github.com/ros-industrial/universal_robot) package.
- Refer to comments and code in each example source file.
- Gazebo/MoveIt environment must be properly set up and running before executing the example nodes in this package.

## License
MIT License (see LICENSE file)

## Issues & Contributions
Issues and pull requests are welcome!