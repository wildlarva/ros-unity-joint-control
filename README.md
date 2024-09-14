# ROS-Unity integration script for arm robot joint control

This is an Unity Script for ROS-Unity integration.
You can move an arm robot on Unity from ros2_control with this script.

## Requirements

* ROS version: ROS 2 humble
* Unity version: 2022.3.46 or above
* ROS packages
  * topic_based_ros2_control plugin
    * https://github.com/PickNikRobotics/topic_based_ros2_control.
* Unity packages
  * ROS-TCP-Connector
    * https://github.com/Unity-Technologies/ROS-TCP-Connector
  * URDF-Importer
    * https://github.com/Unity-Technologies/URDF-Importer

# Limitations

* Driven joints must be of type "revolute".
* Robot must be of 6 DOF.
* Topics for joint commands and joint states must be ordered from the base to end (near end-effector).

## How to Use this script

### On ROS

This script depends on topic_based_ros2_control plugin, which is ros2_control hardware component for integration. See https://github.com/PickNikRobotics/topic_based_ros2_control.

You must have `<ros2_control>` tag with topic_based_ros2_control plugin on your URDF.
The parameters `joint_commands_topic` and `joint_states_topic` must be the same with the topic names configured on Unity script.

### On Unity

You must copy the ROS-Unity integration script `RosJointControl.cs` to your Unity project and add it to a Game Object.
Set the script properties below:
* Joint Names
  * Joint names in URDF to drive.
* Joint States Topic Name
  * Topic name for joint commands. This must be the same with the topic_based_ros2_control parameter `joint_commands_topic`.
* Joint States Topic Name
  * Topic name for joint states. This must be the same with the topic_based_ros2_control parameter `joint_states_topic`.
* Urdf Robot
  * Manipulation target robot object imported by URDF-Importer.

## Example project

See https://github.com/wildlarva/example-ros-unity-integration.
