# test_joint_control_gazebo

![ROS](https://img.shields.io/badge/ROS-Noetic-blue) ![License](https://img.shields.io/badge/License-MIT-green)

`test_joint_control_gazebo` is an example ROS package designed to control and monitor a 7-degree-of-freedom (7-DoF) robotic arm in a Gazebo simulation environment. It includes a publisher node to send joint position commands and a subscriber node to receive joint state data. The robotic arm model used in this package is sourced from the [Mastering ROS for Robotics Programming - Third Edition](https://github.com/PacktPublishing/Mastering-ROS-for-Robotics-Programming-Third-edition) repository.

## Purpose
- **Joint Control**: Send position commands to specific joints to control the robotic arm.
- **State Monitoring**: Receive joint state data (`joint_states`) published by Gazebo to monitor the robot's current state.
- **Learning & Testing**: Provide an example for studying robot control and communication with ROS and Gazebo.

## Package Structure
```
test_joint_control_gazebo/
├── src/
│   ├── test_joint_control_publisher.cpp  # Joint position command publisher
│   └── test_joint_control_subscriber.cpp # Joint state subscriber
├── include/                              # (Optional headers if needed)
├── CMakeLists.txt                       # Build configuration
├── package.xml                          # Package metadata
└── README.md                            # This file
```

## Dependencies
- ROS (Tested on Noetic)
- `roscpp`
- `std_msgs`
- `sensor_msgs`
- Robot model from [Mastering ROS for Robotics Programming - Third Edition](https://github.com/PacktPublishing/Mastering-ROS-for-Robotics-Programming-Third-edition)

## Installation

### 1. Install ROS
If ROS is not installed, follow the [official ROS installation guide](http://wiki.ros.org/ROS/Installation). For example:
```bash
sudo apt-get install ros-noetic-desktop-full
```

### 2. Create a Workspace
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

### 3. Clone the Robot Model
Clone the robot model repository from [Mastering ROS for Robotics Programming - Third Edition](https://github.com/PacktPublishing/Mastering-ROS-for-Robotics-Programming-Third-edition):
```bash
git clone https://github.com/PacktPublishing/Mastering-ROS-for-Robotics-Programming-Third-edition.git
```
- The relevant package (`seven_dof_arm_gazebo`) will be used for the Gazebo simulation.

### 4. Clone This Package
Clone this repository into the same workspace:
```bash
git clone https://github.com/IRaC-Lab/test_joint_control_gazebo.git
```

### 5. Install Dependencies
```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 6. Build the Workspace
```bash
catkin_make
source devel/setup.bash
```

## Usage

### Prerequisites
- The 7-DoF robotic arm model and Gazebo simulation environment are provided by the `seven_dof_arm_gazebo` package from the [Mastering ROS for Robotics Programming - Third Edition](https://github.com/PacktPublishing/Mastering-ROS-for-Robotics-Programming-Third-edition) repository. Ensure this package is built and its launch file (`seven_dof_arm_gazebo_control.launch`) is available:
  ```bash
  roslaunch seven_dof_arm_gazebo seven_dof_arm_gazebo_control.launch
  ```

### 1. Joint Position Control (Publisher)
- **File**: `src/test_joint_control_publisher.cpp`
- **Function**: Publishes joint position commands to a specified joint's control topic (`/seven_dof_arm/jointX_position_controller/command`) based on user input.
- **Run**:
  ```bash
  rosrun test_joint_control_gazebo test_joint_control_publisher <joint_number> <angle>
  ```
  - `<joint_number>`: Joint number between 1 and 7.
  - `<angle>`: Target angle in radians.
  - Example: Move `joint4` to 90 degrees (1.57 radians):
    ```bash
    rosrun test_joint_control_gazebo test_joint_control_publisher 4 1.57
    ```
- **Behavior**: Continuously publishes (10 Hz) the specified angle to the joint and logs the sent value to the console.

### 2. Joint State Monitoring (Subscriber)
- **File**: `src/test_joint_control_subscriber.cpp`
- **Function**: Subscribes to the `/seven_dof_arm/joint_states` topic and prints the name and position of the first joint (`joint1`).
- **Run**:
  ```bash
  rosrun test_joint_control_gazebo test_joint_control_subscriber
  ```
- **Behavior**: Outputs the name and position of `elbow_pitch_joint` to the console whenever a new `joint_states` message is received.

### 3. Full Test
1. Start the Gazebo simulation with the 7-DoF arm model:
   ```bash
   roslaunch seven_dof_arm_gazebo seven_dof_arm_gazebo_control.launch
   ```
2. Run the publisher (e.g., set joint4 to 1.57 radians):
   ```bash
   rosrun test_joint_control_gazebo test_joint_control_publisher 4 1.57
   ```
3. Run the subscriber:
   ```bash
   rosrun test_joint_control_gazebo test_joint_control_subscriber
   ```
   - Check the terminal for `elbow_pitch_joint` state output and observe the robot's movement in Gazebo.

## Topics
- **Published**: `/seven_dof_arm/jointX_position_controller/command` (`std_msgs/Float64`)
  - `X` is the joint number (1 to 7).
- **Subscribed**: `/seven_dof_arm/joint_states` (`sensor_msgs/JointState`)

## Notes
- The robot model and simulation setup rely on the `seven_dof_arm_gazebo` package from [Mastering ROS for Robotics Programming - Third Edition](https://github.com/PacktPublishing/Mastering-ROS-for-Robotics-Programming-Third-edition). Ensure it is properly installed and configured.
- Invalid joint numbers or angles will result in an error message and node termination.
- The subscriber currently outputs only `elbow_pitch_joint`. Modify the code to monitor all joints if needed.

## Future Improvements
- **Multi-Joint Control**: Extend the publisher to control multiple joints simultaneously.
- **Enhanced State Output**: Update the subscriber to display all joint states.
- **Launch File**: Add a launch file to run both nodes together.

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Contributing
Bug reports or suggestions are welcome via [issues](https://github.com/IRaC-Lab/test_joint_control_gazebo/issues) or pull requests!