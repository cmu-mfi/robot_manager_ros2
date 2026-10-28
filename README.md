# Robot Manager ROS2
This repository holds the ros2 interfaces used by any robot_manager. The purpose of these is to abstract any robot specific control onto behind these ros2 actions and services, allowing for robot-agnostic programming. 

## ⚙️ Prerequisites and Dependencies

This repository has been tested on:

  * **OS:** Ubuntu 24.04 LTS
  * **ROS 2:** Jazzy

### Install dependencies using rosdep
Clone this repository into the `src` directory of your ros2 workspace.

```bash
cd <your-ros2-workspace>
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Making a robot manager
A step-by-step guide to implementing a robot manager on top of these interfaces can be
found in [IMPLEMENTING.md](./IMPLEMENTING.md). For a fully working robot manager, the
following interfaces have to be implemented:

| Interface | Type | Description |
| :--- | :--- | :--- |
| `~/joint_goal` | `action/JointGoal` | Send a joint goal to the robot. |
| `~/pose_goal` | `action/PoseGoal` | Send a pose goal to the robot. Implement target and reference frame handling. |
| `~/pose_servo` | `msg/PoseServo` | Servo the robot by a pose message. |
| `~/twist_servo` | `geometry_msgs/TwistStamped` | Servo the robot by a twist message. |
| `~/admittance_pose_servo` | `msg/PoseServo` | Streamed desired pose while the admittance controller is active. |
| `~/home` | `srv/Home` | Home the robot. |
| `~/park` | `srv/Park` | Park the robot. |
| `~/set_payload` | `srv/SetPayload` | Set the tool payload mass and center of gravity. |
| `~/set_io` | `srv/SetIo` | Write to any IO port. |
| `~/start_admittance_pose_servo` | `srv/StartAdmittancePoseServo` | Start the admittance controller. |
| `~/stop_admittance_pose_servo` | `std_srvs/Trigger` | Stop the admittance controller. |
| `~/wrench` | `geometry_msgs/WrenchStamped` | Publish filtered and calibrated force-torque data. |
| `~/ee_pose`, `~/ee_twist`, `~/ee_accel` | `geometry_msgs/PoseStamped`, `TwistStamped`, `AccelStamped` | Publish end-effector state. |

## Example Robot Managers
Examples of fully implemented robot managers can be found in the following repositories:
- [ynx_robot_manager](https://github.com/cmu-mfi/ynx_ros2)
- [ur_robot_manager](https://github.com/cmu-mfi/ur_ros2)
