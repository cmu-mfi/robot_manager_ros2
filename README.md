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
For a fully working robot manager, the following interfaces have to be implemented:
| Interface     | Type             | Description                                                                   |
| ------------- | ---------------- | ----------------------------------------------------------------------------- |
| ~/joint_goal  | action/JointGoal | Send a joint goal to the robot.                                               |
| ~/pose_goal   | action/PoseGoal  | Send a pose goal to the robot. Implement target and reference frame handling. |
| ~/twist_servo | msg/TwistServo   | Servo the robot by a twist message.                                           |
| ~/pose_servo  | msg/PoseServo    | Servo the robot by a pose message.                                            |
| ~/home        | srv/Home         | Home the robot.                                                               |
| ~/write_io    | srv/WriteIo      | Write to any IO port.                                                         |
| ~/read_io     | srv/ReadIo       | Read from any IO port.                                                        |
| ~/ft          | msg/WrenchStamed | Publish filtered and calibrated force-torque data                             |

## Example Robot Managers
Examples of fully implemented robot managers can be found in the following repositories:
- [ynx_robot_manager](https://github.com/cmu-mfi/ynx_ros2)
- [ur_robot_manager](https://github.com/cmu-mfi/ur_ros2)
