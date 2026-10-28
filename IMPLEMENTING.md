# Implementing a Robot Manager

This guide explains how to implement a **robot manager**: a single ROS 2 node that
abstracts all robot-specific control behind the common ROS 2 actions, services and topics
defined in [`robot_manager_interfaces`](./robot_manager_interfaces). Once your robot
exposes these interfaces, higher-level applications can control it robot-agnostically —
without knowing anything about the underlying hardware, driver, or vendor API.

Two fully working reference implementations exist. **Read them side by side while
following this guide — they are structured identically and only differ in the
robot-specific parts** (see [Robot-Specific Parts](#11-robot-specific-parts-to-adapt)):

| Robot | Repository |
| :--- | :--- |
| Universal Robots (UR) | [cmu-mfi/ur_ros2](https://github.com/cmu-mfi/ur_ros2) |
| Yaskawa YNX | [cmu-mfi/ynx_ros2](https://github.com/cmu-mfi/ynx_ros2) |

> Throughout this guide, `<robot>_` stands for your robot's prefix (e.g. `ur_`, `ynx_`).

---

## Table of Contents

1. [Architecture](#1-architecture)
2. [The Interface Contract](#2-the-interface-contract)
3. [Package Layout](#3-package-layout)
4. [Step 1 — Create the Packages](#4-step-1--create-the-packages)
5. [Step 2 — Node Skeleton](#5-step-2--node-skeleton)
6. [Step 3 — Control Mode Machinery](#6-step-3--control-mode-machinery)
7. [Step 4 — MoveIt Setup](#7-step-4--moveit-setup)
8. [Step 5 — Servo Setup](#8-step-5--servo-setup)
9. [Step 6 — TF Helpers](#9-step-6--tf-helpers)
10. [Step 7 — Implement Each Interface](#10-step-7--implement-each-interface)
11. [Robot-Specific Parts to Adapt](#11-robot-specific-parts-to-adapt)
12. [Step 8 — Bringup Layer](#12-step-8--bringup-layer)
13. [Step 9 — Build, Run, Test](#13-step-9--build-run-test)
14. [Final Checklist](#14-final-checklist)

---

## 1. Architecture

A complete robot integration consists of **three layers**:

```
┌─────────────────────────────────────────────────────────────────────┐
│  <robot>_bringup   launch files + configs (URDF/SRDF, kinematics,   │
│                    joint limits, servo parameters, controllers)     │
├─────────────────────────────────────────────────────────────────────┤
│  <robot>_robot_manager   THE node you are implementing. Implements  │
│                          every interface below on top of the        │
│                          robot's driver + MoveIt.                   │
├─────────────────────────────────────────────────────────────────────┤
│  robot_manager_interfaces   (this repo) shared .action/.msg/.srv    │
│                             definitions — identical for every robot │
└─────────────────────────────────────────────────────────────────────┘
```

- `robot_manager_interfaces` is the **contract**. It never contains robot-specific code.
- `<robot>_robot_manager` is the **implementation**. It is a MoveIt-based node: goal
  actions go through MoveIt planning (`pilz_industrial_motion_planner`, `PTP`/`LIN`), and
  streaming servo commands go through `moveit_servo`, whose trajectory output is consumed
  by the robot's trajectory controller.
- `<robot>_bringup` glues the robot driver, its controllers, MoveIt and the robot manager
  together.

Reference implementations:

- **UR:** [ur_ros2](https://github.com/cmu-mfi/ur_ros2) — packages `ur_robot_manager`,
  `ur_bringup`
- **YNX:** [ynx_ros2](https://github.com/cmu-mfi/ynx_ros2) — packages `ynx_robot_manager`,
  `ynx_bringup`

---

## 2. The Interface Contract

Every robot manager must implement **all** of the following, all inside the node's
namespace (`~` = `/<ns>/...`). Names must match exactly, or robot-agnostic applications
will not find them.

### Actions

| Name | Type | Purpose |
| :--- | :--- | :--- |
| `~/joint_goal` | [`robot_manager_interfaces/action/JointGoal`](./robot_manager_interfaces/action/JointGoal.action) | Move to a joint-space goal (6 joint positions + velocity/acceleration scaling). |
| `~/pose_goal` | [`robot_manager_interfaces/action/PoseGoal`](./robot_manager_interfaces/action/PoseGoal.action) | Move to a Cartesian pose goal. `method` is `PTP` (point-to-point) or `LIN` (linear). Includes target/reference frame handling. |

### Services

| Name | Type | Purpose |
| :--- | :--- | :--- |
| `~/home` | [`robot_manager_interfaces/srv/Home`](./robot_manager_interfaces/srv/Home.srv) | Move the robot to its home joint configuration at `speed`. |
| `~/park` | [`robot_manager_interfaces/srv/Park`](./robot_manager_interfaces/srv/Park.srv) | Move the robot to its park joint configuration at `speed`. |
| `~/set_payload` | [`robot_manager_interfaces/srv/SetPayload`](./robot_manager_interfaces/srv/SetPayload.srv) | Set tool payload `mass` (kg) and center of gravity `cog` (m). |
| `~/set_io` | [`robot_manager_interfaces/srv/SetIo`](./robot_manager_interfaces/srv/SetIo.srv) | Write a digital output `pin` to `state` (1 = high, 0 = low). |
| `~/start_admittance_pose_servo` | [`robot_manager_interfaces/srv/StartAdmittancePoseServo`](./robot_manager_interfaces/srv/StartAdmittancePoseServo.srv) | Initialize the admittance controller (target wrench, selection vector, mass, damping, stiffness, max velocity). |
| `~/stop_admittance_pose_servo` | `std_srvs/srv/Trigger` | Stop the admittance controller and return to IDLE. |

### Subscriptions (streaming commands)

| Name | Type | Purpose |
| :--- | :--- | :--- |
| `~/pose_servo` | [`robot_manager_interfaces/msg/PoseServo`](./robot_manager_interfaces/msg/PoseServo.msg) | Servo toward a streamed target pose. |
| `~/twist_servo` | `geometry_msgs/msg/TwistStamped` | Servo with a streamed twist (velocity) command. |
| `~/admittance_pose_servo` | [`robot_manager_interfaces/msg/PoseServo`](./robot_manager_interfaces/msg/PoseServo.msg) | Streamed desired pose **while the admittance controller is active**. |

### Publications (state feedback)

| Name | Type | Purpose |
| :--- | :--- | :--- |
| `~/wrench` | `geometry_msgs/msg/WrenchStamped` | Filtered and calibrated force-torque data. |
| `~/ee_pose` | `geometry_msgs/msg/PoseStamped` | End-effector pose (planning frame). |
| `~/ee_twist` | `geometry_msgs/msg/TwistStamped` | End-effector twist (finite-differenced). |
| `~/ee_accel` | `geometry_msgs/msg/AccelStamped` | End-effector acceleration (finite-differenced). |

The node additionally publishes servo output on the topic configured by
`servo_parameters.yaml` (`command_out_topic`), typically consumed by a
`joint_trajectory_controller`.

### Semantic rules

These rules are part of the contract — both reference implementations follow them:

1. **Exclusive control** — at most one "movement" interface may be active at a time
   (see [Control Modes](#6-step-3--control-mode-machinery)). Any new goal/service request
   is rejected while another is running.
2. **Error codes** — action results use the shared convention:
   `0` = none, `1` = planning failed, `2` = execution failed, `3` = cancelled. Always
   fill in a human-readable `message`.
3. **Scaling factors** — velocity/acceleration scaling are fractions in `(0.0, 1.0]`.
   Reject or clamp out-of-range values; never accept `0.0` or negatives.
4. **Frame conventions** — in `PoseGoal` and `PoseServo`:
   - `frame_id` empty → the robot's planning frame (base); otherwise any TF frame.
   - `target_id` empty → `tool0`; otherwise **only a child frame of `tool0`** is valid
     (e.g. a TCP frame). Reject anything else.
5. **Servo watchdog** — streaming interfaces stop and return to IDLE if no new command
   arrives within ~200 ms. Never leave the robot moving under a stale command.
6. **Service responses** — every service fills `success` and a human-readable `message`,
   including on early validation failures.

---

## 3. Package Layout

Mirror the reference implementations:

```
<robot>_ros2/
├── <robot>_robot_manager/          # the node
│   ├── include/<robot>_robot_manager/<robot>_robot_manager.hpp
│   ├── src/
│   │   ├── <robot>_robot_manager.cpp   # constructor + setup() + main()
│   │   ├── moveit.cpp                  # MoveGroupInterface setup
│   │   ├── servo.cpp                   # moveit_servo setup
│   │   ├── tf.cpp                      # frame helper functions
│   │   ├── joint_goal.cpp              # ~/joint_goal action
│   │   ├── pose_goal.cpp               # ~/pose_goal action
│   │   ├── home_service.cpp            # ~/home
│   │   ├── park_service.cpp            # ~/park
│   │   ├── set_payload_service.cpp     # ~/set_payload
│   │   ├── set_io_service.cpp          # ~/set_io
│   │   ├── admittance_pose_servo.cpp   # ~/start|stop_admittance_pose_servo + ~/admittance_pose_servo
│   │   ├── pose_servo.cpp              # ~/pose_servo
│   │   ├── twist_servo.cpp             # ~/twist_servo
│   │   ├── wrench_publisher.cpp        # ~/wrench
│   │   └── ee_state_publisher.cpp      # ~/ee_pose, ~/ee_twist, ~/ee_accel
│   ├── CMakeLists.txt
│   └── package.xml
└── <robot>_bringup/                # launch + config
    ├── launch/
    │   ├── bringup.launch.py       # driver + moveit + robot_manager
    │   ├── driver.launch.py
    │   ├── moveit.launch.py
    │   └── robot_manager.launch.py
    ├── config/                     # kinematics, joint_limits, servo_parameters, ...
    └── package.xml
```

One source file per feature keeps the diff between robots small — when comparing the two
references, most files are literally identical except for the namespace.

---

## 4. Step 1 — Create the Packages

### `package.xml`

Depend on `robot_manager_interfaces` plus the MoveIt stack. Copy and rename:

- UR: [ur_robot_manager/package.xml](https://github.com/cmu-mfi/ur_ros2/blob/main/ur_robot_manager/package.xml)
- YNX: [ynx_robot_manager/package.xml](https://github.com/cmu-mfi/ynx_ros2/blob/main/ynx_robot_manager/package.xml)

```xml
<package format="3">
  <name><robot>_robot_manager</name>
  <version>0.0.1</version>
  <description>Robot Manager for <Your Robot></description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>BSD-3-Clause</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>moveit_ros_planning_interface</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>moveit_servo</depend>
  <depend>std_srvs</depend>
  <depend>robot_manager_interfaces</depend>
  <!-- add robot-driver-specific deps here, e.g. ur_msgs / control_msgs -->

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### `CMakeLists.txt`

Both references use the same structure — one executable built from all `src/` files:

- UR: [ur_robot_manager/CMakeLists.txt](https://github.com/cmu-mfi/ur_ros2/blob/main/ur_robot_manager/CMakeLists.txt)
- YNX: [ynx_robot_manager/CMakeLists.txt](https://github.com/cmu-mfi/ynx_ros2/blob/main/ynx_robot_manager/CMakeLists.txt)

```cmake
cmake_minimum_required(VERSION 3.8)
project(<robot>_robot_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(DEPENDENCIES
  ament_cmake
  moveit_core
  moveit_ros_planning_interface
  moveit_servo
  rclcpp
  rclcpp_action
  tf2_ros
  robot_manager_interfaces
  std_srvs
  geometry_msgs
)

foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(
  <robot>_robot_manager
  src/<robot>_robot_manager.cpp
  src/moveit.cpp
  src/servo.cpp
  src/tf.cpp
  # ... one file per feature, see layout above
)

target_include_directories(<robot>_robot_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(<robot>_robot_manager ${DEPENDENCIES})
ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES})

install(TARGETS <robot>_robot_manager DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_package()
```

---

## 5. Step 2 — Node Skeleton

- UR: [ur_robot_manager.cpp](https://github.com/cmu-mfi/ur_ros2/blob/main/ur_robot_manager/src/ur_robot_manager.cpp)
- YNX: [ynx_robot_manager.cpp](https://github.com/cmu-mfi/ynx_ros2/blob/main/ynx_robot_manager/src/ynx_robot_manager.cpp)

### Constructor

The node is created with `automatically_declare_parameters_from_overrides(true)` so every
parameter passed by the launch file is pre-declared. It reads `ns` (namespace) and
`tf_prefix` and derives the MoveIt planning group name, which by convention is
`<tf_prefix>manipulator`:

```cpp
XRobotManager::XRobotManager()
    : Node("x_robot_manager", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {
  ns_ = this->get_parameter("ns").as_string();
  tf_prefix_ = this->get_parameter("tf_prefix").as_string();
  moveit_planning_group_ = tf_prefix_ + "manipulator";
  RCLCPP_INFO(this->get_logger(), "Initializing X Robot Manager with namespace: /%s and planning group: %s",
      ns_.c_str(), moveit_planning_group_.c_str());
}
```

**Namespace convention:** `ns` and `tf_prefix` are separate because TF frame names cannot
contain the leading slash. The launch files set `tf_prefix = ns + "_"` when a namespace is
given, so a robot in namespace `cell1` has frames `cell1_base_link`, `cell1_tool0`, ….

### `setup()`

`setup()` creates the callback groups, TF listener and **all** interfaces. It is called
from a separate thread in `main()` (see below) because some setup calls block — e.g.
`MoveGroupInterface` construction waits for the MoveGroup action server, and service
setups may wait for driver services:

```cpp
void XRobotManager::setup() {
  // ROS2 Setup — three mutually-exclusive callback groups
  service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  servo_cb_group_   = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  ee_state_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // TF setup
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  // Setup MoveIt, Servo and every interface
  moveit_setup();
  servo_setup();
  joint_goal_setup();
  pose_goal_setup();
  home_service_setup();
  park_service_setup();
  set_payload_service_setup();
  set_io_service_setup();
  wrench_publisher_setup();
  ee_state_publisher_setup();
  admittance_pose_servo_setup();

  RCLCPP_INFO(this->get_logger(), "X Robot Manager is ready!");
}
```

Callback group assignment convention:

| Group | Used by |
| :--- | :--- |
| `service_cb_group_` | all service servers (blocking work, so they must not starve other callbacks) |
| `servo_cb_group_` | servo loop timers (real-time-ish, high rate) |
| `ee_state_cb_group_` | EE state publisher timer (200 Hz) |
| default (node) group | action servers, servo subscription callbacks |

### `main()`

`MultiThreadedExecutor` is **required** — the node uses multiple callback groups and
detached execution threads that re-enter ROS calls:

```cpp
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<x_robot_manager::XRobotManager>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  std::thread setup_thread([&node]() { node->setup(); });

  executor.spin();

  rclcpp::shutdown();
  if (setup_thread.joinable()) setup_thread.join();
  return 0;
}
```

### Class declaration

Copy the header structure from the reference (UR:
[`ur_robot_manager.hpp`](https://github.com/cmu-mfi/ur_ros2/blob/main/ur_robot_manager/include/ur_robot_manager/ur_robot_manager.hpp) /
YNX:
[`ynx_robot_manager.hpp`](https://github.com/cmu-mfi/ynx_ros2/blob/main/ynx_robot_manager/include/ynx_robot_manager/ynx_robot_manager.hpp)).
Start with the common aliases, which you will use everywhere:

```cpp
using JointGoal = robot_manager_interfaces::action::JointGoal;
using JointGoalHandle = rclcpp_action::ServerGoalHandle<JointGoal>;
using PoseGoal = robot_manager_interfaces::action::PoseGoal;
using PoseGoalHandle = rclcpp_action::ServerGoalHandle<PoseGoal>;
using PoseServo = robot_manager_interfaces::msg::PoseServo;
using Home = robot_manager_interfaces::srv::Home;
using Park = robot_manager_interfaces::srv::Park;
using SetPayload = robot_manager_interfaces::srv::SetPayload;
using SetIo = robot_manager_interfaces::srv::SetIo;
using WrenchStamped = geometry_msgs::msg::WrenchStamped;
using StartAdmittancePoseServo = robot_manager_interfaces::srv::StartAdmittancePoseServo;
using Trigger = std_srvs::srv::Trigger;
```

---

## 6. Step 3 — Control Mode Machinery

The heart of safe operation: **only one control interface may drive the robot at a
time**. Every interface claims a `ControlMode`, and everything else is rejected while a
mode is claimed.

Defined in the header of both references:

```cpp
enum class ControlMode {
  IDLE,
  POSE_GOAL,
  JOINT_GOAL,
  HOME,
  PARK,
  POSE_SERVO,
  TWIST_SERVO,
  ADMITTANCE_POSE_SERVO
};

// RAII guard: resets the mode to IDLE when the execution scope exits,
// no matter whether the goal succeeded, aborted, or threw.
struct ModeReset {
  std::mutex &mtx;
  ControlMode &mode;
  ~ModeReset() {
    std::lock_guard<std::mutex> lock(mtx);
    mode = ControlMode::IDLE;
  }
};
```

Rules for claiming the mode (applied identically in **every** interface handler):

1. Lock `control_mode_mutex_`.
2. If the current mode is not allowed for this interface → **reject** with a clear log
   that names the current mode (`controlModeToString`), and return a failing
   response/result.
3. Otherwise set your mode **inside the same lock**, then release.
4. Long-running handlers construct `ModeReset mode_reset = {control_mode_mutex_,
   control_mode_};` so the mode automatically returns to `IDLE` when the handler finishes.

Which mode transitions are allowed:

| Interface | Allowed current modes |
| :--- | :--- |
| `joint_goal`, `pose_goal`, `home`, `park`, `set_payload`, `start_admittance_pose_servo` | `IDLE` only |
| `pose_servo` | `IDLE` → claims `POSE_SERVO`; stays if already `POSE_SERVO` or `ADMITTANCE_POSE_SERVO` (admittance drives the robot through the pose-servo path) |
| `twist_servo` | `IDLE` → claims `TWIST_SERVO`; stays if already `TWIST_SERVO` |
| streaming watchdog timeout | any servo mode → back to `IDLE` |

> The servo subscription callbacks are *stateless claimers*: they never construct
> `ModeReset`. Instead, the watchdog in their timer callback returns the mode to `IDLE`
> when the stream stops (see [Servo interfaces](#servo-interfaces-pose_servo-twist_servo-admittance_pose_servo)).

---

## 7. Step 4 — MoveIt Setup

- UR: [moveit.cpp](https://github.com/cmu-mfi/ur_ros2/blob/main/ur_robot_manager/src/moveit.cpp)
- YNX: [moveit.cpp](https://github.com/cmu-mfi/ynx_ros2/blob/main/ynx_robot_manager/src/moveit.cpp)

```cpp
void XRobotManager::moveit_setup() {
  moveit::planning_interface::MoveGroupInterface::Options options(
      moveit_planning_group_, "robot_description", "/" + ns_);
  moveit_move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(), options);
  // default settings
  moveit_move_group_->setPlanningTime(5.0);
  moveit_move_group_->setNumPlanningAttempts(10);
  moveit_move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
  moveit_move_group_->setPlannerId("PTP");
}
```

Requirements:

- The planning group **must** be named `<tf_prefix>manipulator` in the SRDF.
- The pilz planner provides the `PTP` and `LIN` planner IDs used by the action servers —
  configure its `cartesian_limits.yaml` in your bringup.
- All MoveIt parameters (`robot_description`, `robot_description_semantic`,
  `robot_description_kinematics`, `robot_description_planning`) are provided by the
  launch file, so keep the node's `use_sim_time` / parameter handling as in the reference.
- `planning_scene_parameters` in the launch file make the external planning scene
  (maintained by the MoveIt node) available to this process — the servo stack depends on
  it.

---

## 8. Step 5 — Servo Setup

- UR: [servo.cpp](https://github.com/cmu-mfi/ur_ros2/blob/main/ur_robot_manager/src/servo.cpp)
- YNX: [servo.cpp](https://github.com/cmu-mfi/ynx_ros2/blob/main/ynx_robot_manager/src/servo.cpp)

`moveit_servo` computes joint trajectories from streamed pose/twist commands. One `Servo`
instance serves both the pose and twist paths (and, indirectly, admittance), by switching
`CommandType` per command:

```cpp
void XRobotManager::servo_setup() {
  // 1. Parameters + planning scene monitor (hooks into the external planning scene)
  servo_param_listener_ = std::make_shared<servo::ParamListener>(shared_from_this(), "moveit_servo");
  servo_params_ = servo_param_listener_->get_params();
  servo_planning_scene_monitor_ = moveit_servo::createPlanningSceneMonitor(shared_from_this(), servo_params_);
  // 2. Servo instance
  servo_ = std::make_unique<moveit_servo::Servo>(shared_from_this(), servo_param_listener_, servo_planning_scene_monitor_);
  servo_->setCommandType(moveit_servo::CommandType::POSE);
  // 3. Output publisher (topic comes from servo_parameters.yaml) + command subscribers
  servo_trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      servo_params_.command_out_topic, rclcpp::SystemDefaultsQoS());
  pose_servo_subscriber_ = this->create_subscription<PoseServo>(
      "pose_servo", rclcpp::SystemDefaultsQoS(),
      std::bind(&XRobotManager::pose_servo_subscription_callback_, this, std::placeholders::_1));
  twist_servo_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "twist_servo", rclcpp::SystemDefaultsQoS(),
      std::bind(&XRobotManager::twist_servo_subscription_callback_, this, std::placeholders::_1));
  // 4. Loop timers at the servo publish period, in the servo callback group
  pose_servo_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(servo_params_.publish_period),
      std::bind(&XRobotManager::pose_servo_loop_callback_, this), servo_cb_group_);
  twist_servo_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(servo_params_.publish_period),
      std::bind(&XRobotManager::twist_servo_loop_callback_, this), servo_cb_group_);
}
```

Key parameters in `servo_parameters.yaml`: `move_group_name` (must be your planning
group), `publish_period`, `command_out_topic`, `max_expected_latency`. Copy the file from
a reference and adjust:

- UR: [servo_parameters.yaml](https://github.com/cmu-mfi/ur_ros2/blob/main/ur_bringup/config/servo_parameters.yaml)
- YNX: [servo_parameters.yaml](https://github.com/cmu-mfi/ynx_ros2/blob/main/ynx_bringup/config/servo_parameters.yaml)

> **Known MoveIt Servo bug workaround:** the bringup launch files multiply each joint's
> `max_velocity` in `robot_description_planning` by 10 before passing it to the robot
> manager. If servo motions are unexpectedly slow, check you did not remove this
> workaround (see [Bringup](#12-step-8--bringup-layer)).

---

## 9. Step 6 — TF Helpers

- UR: [tf.cpp](https://github.com/cmu-mfi/ur_ros2/blob/main/ur_robot_manager/src/tf.cpp)
- YNX: [tf.cpp](https://github.com/cmu-mfi/ynx_ros2/blob/main/ynx_robot_manager/src/tf.cpp)

Two helpers enforce the frame contract. **Copy them unchanged** (they only use
`tf_prefix_`):

`transform_pose_to_tool0_frame(pose, input_frame)`:

- Returns `pose` unchanged if `input_frame` is empty or equals the end-effector link.
- Otherwise verifies `input_frame` is a **child of `tool0`** (walks the TF parent chain);
  throws `tf2::TransformException` if it is not.
- Composes `pose (in input_frame) * tool0 (in input_frame)` to express the target as a
  `tool0` pose.

`transform_pose_to_base_link_frame(pose, input_frame)`:

- Returns `pose` unchanged if `input_frame` is empty or already `base_link`.
- Otherwise uses `tf_buffer_->transform()` to convert the pose into
  `<tf_prefix>base_link` (0.5 s timeout; throws on failure).

Callers must catch `tf2::TransformException` and turn it into a rejection/abort with the
error text in the response `message`.

---

## 10. Step 7 — Implement Each Interface

Every interface follows the same macro-pattern:

> **validate → claim mode → (plan and) execute → respond.**

Full source for each interface is linked. The excerpts below are from the UR
implementation; the YNX files are identical modulo the namespace.

### Goal actions (`~/joint_goal`, `~/pose_goal`)

Files:

- UR: [joint_goal.cpp](https://github.com/cmu-mfi/ur_ros2/blob/main/ur_robot_manager/src/joint_goal.cpp) ·
  [pose_goal.cpp](https://github.com/cmu-mfi/ur_ros2/blob/main/ur_robot_manager/src/pose_goal.cpp)
- YNX: [joint_goal.cpp](https://github.com/cmu-mfi/ynx_ros2/blob/main/ynx_robot_manager/src/joint_goal.cpp) ·
  [pose_goal.cpp](https://github.com/cmu-mfi/ynx_ros2/blob/main/ynx_robot_manager/src/pose_goal.cpp)

Wire the standard three-callback action server:

```cpp
void XRobotManager::joint_goal_setup() {
  joint_goal_action_server_ = rclcpp_action::create_server<JointGoal>(
      this, "joint_goal",
      std::bind(&XRobotManager::joint_goal_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&XRobotManager::joint_goal_handle_cancel, this, std::placeholders::_1),
      std::bind(&XRobotManager::joint_goal_handle_accepted, this, std::placeholders::_1));
}
```

**1. Goal callback — validate and claim:**

```cpp
rclcpp_action::GoalResponse XRobotManager::joint_goal_handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const JointGoal::Goal> goal) {
  (void)uuid;
  { // claim exclusive control
    std::lock_guard<std::mutex> lock(control_mode_mutex_);
    if (control_mode_ != ControlMode::IDLE) {
      RCLCPP_ERROR(this->get_logger(), "[JointGoal] Robot is already being controlled (current mode: %s). Rejecting goal.",
          controlModeToString(control_mode_));
      return rclcpp_action::GoalResponse::REJECT;
    }
    control_mode_ = ControlMode::JOINT_GOAL;
  }
  // Interface-specific validation
  if (goal->positions.size() != 6) { /* reject: invalid joint count */ }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
```

Interface-specific validation:

- `joint_goal`: `positions.size() == 6`.
- `pose_goal`: `method` is `PTP` or `LIN`; quaternion has norm ≈ 1 (rejects
  uninitialized orientations); `0 < velocity_scaling ≤ 1.0` and
  `0 < acceleration_scaling ≤ 1.0`.

**2. Cancel callback — stop trajectory execution, accept the cancel:**

```cpp
rclcpp_action::CancelResponse XRobotManager::joint_goal_handle_cancel(
    const std::shared_ptr<JointGoalHandle> goal_handle) {
  (void)goal_handle;
  if (moveit_move_group_) moveit_move_group_->stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}
```

**3. Accepted callback — detach an execution thread:**

```cpp
void XRobotManager::joint_goal_handle_accepted(const std::shared_ptr<JointGoalHandle> goal_handle) {
  std::thread{std::bind(&XRobotManager::joint_goal_handle_execution, this, std::placeholders::_1), goal_handle}.detach();
}
```

**4. Execution callback — plan, execute, report progress:** (`pose_goal` is the same,
plus frame resolution and planner ID selection.)

```cpp
void XRobotManager::joint_goal_handle_execution(const std::shared_ptr<JointGoalHandle> goal_handle) {
  ModeReset mode_reset = {control_mode_mutex_, control_mode_};   // RAII: back to IDLE on exit
  auto result = std::make_shared<JointGoal::Result>();
  auto goal = goal_handle->get_goal();
  moveit_move_group_->clearPoseTargets();
  moveit_move_group_->clearPathConstraints();
  moveit_move_group_->clearTrajectoryConstraints();
  // Configure target + limits
  bool within_bounds = moveit_move_group_->setJointValueTarget(goal->positions);
  if (!within_bounds) RCLCPP_WARN(...); // plan anyway, clamped to limits
  double velocity_scaling = std::max(0.01, std::min(goal->velocity_scaling, 1.0));
  double acceleration_scaling = std::max(0.05, std::min(goal->acceleration_scaling, 1.0));
  moveit_move_group_->setMaxVelocityScalingFactor(velocity_scaling);
  moveit_move_group_->setMaxAccelerationScalingFactor(acceleration_scaling);
  moveit_move_group_->setPlannerId("PTP");
  // Plan
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  if (moveit_move_group_->plan(my_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    result->success = false; result->error_code = 1; result->message = "Planning failed";
    goal_handle->abort(result);
    return;
  }
  // Execute asynchronously and poll for feedback + cancellation at 20 Hz
  auto exec_future = std::async(std::launch::async,
      [this, &my_plan]() { return moveit_move_group_->execute(my_plan); });
  double total_distance = /* joint-space distance from start to goal */;
  auto feedback = std::make_shared<JointGoal::Feedback>();
  rclcpp::Rate loop_rate(20);
  while (exec_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
    if (goal_handle->is_canceling()) {
      result->success = false; result->error_code = 3; result->message = "Execution canceled";
      goal_handle->canceled(result);
      return;
    }
    double progress = (1.0 - /* current distance to goal */ / total_distance) * 100.0;
    feedback->progress = std::max(0.0f, std::min(100.0f, static_cast<float>(progress)));
    goal_handle->publish_feedback(feedback);
    loop_rate.sleep();
  }
  // Final result
  auto exec_status = exec_future.get();
  if (exec_status == moveit::core::MoveItErrorCode::SUCCESS) {
    feedback->progress = 100.0f;
    goal_handle->publish_feedback(feedback);
    result->success = true; result->error_code = 0; result->message = "Execution successful";
    goal_handle->succeed(result);
  } else {
    result->success = false; result->error_code = 2; result->message = "Execution failed";
    goal_handle->abort(result);
  }
}
```

`pose_goal` differences:

- Resolve frames first: `reference_frame = goal->frame_id.empty() ?
  getPlanningFrame() : goal->frame_id`; the target pose is converted with
  `transform_pose_to_tool0_frame(goal->target_pose, goal->target_id)` (abort with
  error code 1 on `tf2::TransformException`).
- `moveit_move_group_->setPoseReferenceFrame(reference_frame)` and
  `setPoseTarget(goal_pose, tool0_frame)`.
- `setPlannerId(goal->method)` — `PTP` or `LIN` as requested.
- Progress feedback uses the Cartesian distance between the start and goal positions of
  `tool0`; publish a fixed 50 % when the goal is purely rotational.
- Scaling factors are *validated* here (not clamped) because the goal callback already
  rejected out-of-range values.

### Service interfaces (`~/home`, `~/park`, `~/set_payload`, `~/set_io`)

Files:

- UR: [home_service.cpp](https://github.com/cmu-mfi/ur_ros2/blob/main/ur_robot_manager/src/home_service.cpp) ·
  [park_service.cpp](https://github.com/cmu-mfi/ur_ros2/blob/main/ur_robot_manager/src/park_service.cpp) ·
  [set_payload_service.cpp](https://github.com/cmu-mfi/ur_ros2/blob/main/ur_robot_manager/src/set_payload_service.cpp) ·
  [set_io_service.cpp](https://github.com/cmu-mfi/ur_ros2/blob/main/ur_robot_manager/src/set_io_service.cpp)
- YNX: [home_service.cpp](https://github.com/cmu-mfi/ynx_ros2/blob/main/ynx_robot_manager/src/home_service.cpp) ·
  [park_service.cpp](https://github.com/cmu-mfi/ynx_ros2/blob/main/ynx_robot_manager/src/park_service.cpp) ·
  [set_payload_service.cpp](https://github.com/cmu-mfi/ynx_ros2/blob/main/ynx_robot_manager/src/set_payload_service.cpp) ·
  [set_io_service.cpp](https://github.com/cmu-mfi/ynx_ros2/blob/main/ynx_robot_manager/src/set_io_service.cpp)

All four follow the same shape — claim mode, validate, act, respond. Services run in
`service_cb_group_` with reliable/volatile QoS:

```cpp
void XRobotManager::home_service_setup() {
  home_service_ = this->create_service<Home>(
      "home",
      std::bind(&XRobotManager::home_service_callback, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile(),
      service_cb_group_);
}
```

**Home/Park** — MoveIt joint move to a robot-specific configuration:

```cpp
void XRobotManager::home_service_callback(
    const std::shared_ptr<Home::Request> request, std::shared_ptr<Home::Response> response) {
  { // claim mode
    std::lock_guard<std::mutex> lock(control_mode_mutex_);
    if (control_mode_ != ControlMode::IDLE) {
      response->success = false;
      response->message = "Robot is already being controlled.";
      return;
    }
    control_mode_ = ControlMode::HOME;
  }
  ModeReset mode_reset = {control_mode_mutex_, control_mode_};
  // Robot-specific: home/park joint positions (see section 11)
  std::vector<double> home_positions = {0.0, -M_PI/2, M_PI/2, -M_PI/2, -M_PI/2, 0.0};
  double velocity_scaling = std::max(0.01, std::min(abs(request->speed), 1.0));
  moveit_move_group_->clearPoseTargets();
  moveit_move_group_->clearPathConstraints();
  moveit_move_group_->setJointValueTarget(home_positions);
  moveit_move_group_->setMaxVelocityScalingFactor(velocity_scaling);
  moveit_move_group_->setMaxAccelerationScalingFactor(velocity_scaling);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  if (moveit_move_group_->plan(my_plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    response->success = false; response->message = "Planning failed. Cannot move home.";
    return;
  }
  auto exec_status = moveit_move_group_->execute(my_plan);
  response->success = (exec_status == moveit::core::MoveItErrorCode::SUCCESS);
  response->message = response->success ? "Robot successfully homed."
                                        : "Execution failed or was cancelled.";
}
```

**Set IO** — this is where robots diverge the most; adapt the *backend*, keep the
contract:

- *UR:* forwards to the driver's `io_and_status_controller/set_io` service
  (`ur_msgs/srv/SetIO`, `FUN_SET_DIGITAL_OUT`) with a 3 s future timeout; valid pins are
  `0–7`. The client is created (and waited for) in `set_io_service_setup()`.
- *YNX:* publishes a `control_msgs/msg/DynamicInterfaceGroupValues` on
  `gpio_command_controller/commands` with interface name `digital_output_<pin>`; valid
  pins are `1–16`.

Validate `pin` (against your robot's range) and `state ∈ {0, 1}` before acting, and
always fill `success`/`message` on every failure path, including driver-timeouts.

**Set payload** — same idea: *UR* forwards to
`io_and_status_controller/set_payload` (`ur_msgs/srv/SetPayload`); the YNX
implementation is currently a validated stub (`TODO`) that only checks `mass ≥ 0`.
Implement whatever your driver supports, but keep request validation and the
response contract.

### Servo interfaces (`~/pose_servo`, `~/twist_servo`, `~/admittance_pose_servo`)

Files:

- UR: [pose_servo.cpp](https://github.com/cmu-mfi/ur_ros2/blob/main/ur_robot_manager/src/pose_servo.cpp) ·
  [twist_servo.cpp](https://github.com/cmu-mfi/ur_ros2/blob/main/ur_robot_manager/src/twist_servo.cpp) ·
  [admittance_pose_servo.cpp](https://github.com/cmu-mfi/ur_ros2/blob/main/ur_robot_manager/src/admittance_pose_servo.cpp)
- YNX: [pose_servo.cpp](https://github.com/cmu-mfi/ynx_ros2/blob/main/ynx_robot_manager/src/pose_servo.cpp) ·
  [twist_servo.cpp](https://github.com/cmu-mfi/ynx_ros2/blob/main/ynx_robot_manager/src/twist_servo.cpp) ·
  [admittance_pose_servo.cpp](https://github.com/cmu-mfi/ynx_ros2/blob/main/ynx_robot_manager/src/admittance_pose_servo.cpp)

These are **streaming** interfaces: a subscriber stores the latest command, and a
high-rate timer turns it into joint commands. Pattern (pose servo; twist is analogous):

**Subscription callback** — soft mode claim, frame resolution, store latest target:

```cpp
void XRobotManager::pose_servo_subscription_callback_(const PoseServo::SharedPtr msg) {
  { // claim: IDLE → POSE_SERVO; stay if POSE_SERVO / ADMITTANCE_POSE_SERVO
    std::lock_guard<std::mutex> lock(control_mode_mutex_);
    if (control_mode_ != ControlMode::IDLE &&
        control_mode_ != ControlMode::POSE_SERVO &&
        control_mode_ != ControlMode::ADMITTANCE_POSE_SERVO) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "[Pose Servo] Robot is already being controlled (current mode: %s).",
          controlModeToString(control_mode_));
      return;
    }
    if (control_mode_ == ControlMode::IDLE) control_mode_ = ControlMode::POSE_SERVO;
  }
  std::lock_guard<std::mutex> lock(pose_servo_mutex_);
  std::string reference_frame = msg->frame_id.empty() ? moveit_move_group_->getPlanningFrame() : msg->frame_id;
  geometry_msgs::msg::Pose goal_pose;
  try {
    goal_pose = transform_pose_to_tool0_frame(msg->target_pose, msg->target_id);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "[PoseServo] TF Verification Failed: %s", ex.what());
    return;
  }
  pose_servo_target_pose_cmd_.frame_id = reference_frame;
  tf2::fromMsg(goal_pose, pose_servo_target_pose_cmd_.pose);
  pose_servo_last_msg_time_ = this->now();
  pose_servo_active_ = true;
}
```

**Loop callback** — watchdog, IK, sliding-window smoothing, publish:

```cpp
void XRobotManager::pose_servo_loop_callback_() {
  if (!pose_servo_active_) return;
  auto now = this->now();
  // Watchdog: no new command within 200 ms → stop streaming and release the mode
  if ((now - pose_servo_last_msg_time_).seconds() > 0.2) {
    pose_servo_active_ = false;
    pose_servo_joint_cmd_rolling_window_.clear();
    {
      std::lock_guard<std::mutex> lock(control_mode_mutex_);
      if (control_mode_ == ControlMode::POSE_SERVO) control_mode_ = ControlMode::IDLE;
    }
    RCLCPP_WARN(this->get_logger(), "[Servo] Stream timed out. Halting.");
    return;
  }
  std::lock_guard<std::mutex> lock(pose_servo_mutex_);
  auto robot_state = servo_planning_scene_monitor_->getStateMonitor()->getCurrentState();
  if (!robot_state) return;
  // IK step toward the streamed target (twist path: setCommandType(TWIST) first)
  moveit_servo::KinematicState joint_state = servo_->getNextJointState(robot_state, pose_servo_target_pose_cmd_);
  if (servo_->getStatus() != moveit_servo::StatusCode::INVALID) {
    // Smooth via the rolling window, then publish
    moveit_servo::updateSlidingWindow(joint_state, pose_servo_joint_cmd_rolling_window_,
                                      servo_params_.max_expected_latency, now);
    if (auto msg = moveit_servo::composeTrajectoryMessage(servo_params_, pose_servo_joint_cmd_rolling_window_)) {
      servo_trajectory_publisher_->publish(msg.value());
    }
    // Feed the commanded state back for continuity on the next iteration
    if (!pose_servo_joint_cmd_rolling_window_.empty()) {
      const auto* jmg = robot_state->getJointModelGroup(servo_params_.move_group_name);
      robot_state->setJointGroupPositions(jmg, pose_servo_joint_cmd_rolling_window_.back().positions);
      robot_state->setJointGroupVelocities(jmg, pose_servo_joint_cmd_rolling_window_.back().velocities);
    }
  }
}
```

Twist servo specifics: `header.frame_id` selects the reference frame (empty → planning
frame); `twist_servo_target_cmd_.velocities` is filled in linear-x/y/z then
angular-x/y/z order; the loop calls `servo_->setCommandType(CommandType::TWIST)` before
`getNextJointState`.

**Admittance pose servo** — the only interface with start/stop services:

1. `~/start_admittance_pose_servo` rejects if already active; otherwise claims
   `ADMITTANCE_POSE_SERVO`, copies the request parameters into a config struct, snapshots
   the current EE pose as the desired pose, resets filter history, and **tares the
   sensor** by averaging 50 samples (~0.1 s) of `~/wrench` data into an offset. Note the
   two-level lock ordering when touching both `ee_state_mutex_` and
   `admittance_pose_servo_mutex_`.
2. `~/admittance_pose_servo` (streaming `PoseServo`) transforms the desired pose into
   `base_link` and stores it — the *desired*, sensor-independent path.
3. The loop callback implements the discrete admittance equation per axis
   (mass-damper-spring with velocity clamping to `max_velocity`), applies the selection
   vector, subtracts the tared wrench offset from the reading and the target wrench from
   it to get the contact error, composes the linear deviation into position and the
   angular deviation (RPY) as a quaternion product onto the desired orientation, then
   **feeds the result into the pose-servo path internally** by constructing a
   `PoseServo` message and calling `pose_servo_subscription_callback_()`.
4. `~/stop_admittance_pose_servo` (`std_srvs/Trigger`) deactivates and returns to `IDLE`.

> The wrench axes are remapped per robot (the UR code indexes `[y, x, z, ty, tx, tz]` —
> check your sensor's convention carefully; see section 11).

### `~/wrench` publisher

Files:

- UR: [wrench_publisher.cpp](https://github.com/cmu-mfi/ur_ros2/blob/main/ur_robot_manager/src/wrench_publisher.cpp)
- YNX: [wrench_publisher.cpp](https://github.com/cmu-mfi/ynx_ros2/blob/main/ynx_robot_manager/src/wrench_publisher.cpp)

Subscribe to your robot's raw/filtered FT topic, sanitize, and republish on `~/wrench`
under `wrench_mutex_`:

```cpp
void XRobotManager::wrench_publisher_setup() {
  // Robot-specific: the driver's FT topic
  ur_wrench_subscriber_ = this->create_subscription<WrenchStamped>(
      "force_torque_sensor_broadcaster/wrench_filtered", rclcpp::QoS(10),
      std::bind(&XRobotManager::ur_wrench_subscription_callback_, this, std::placeholders::_1));
  // Contract: the robot manager publishes filtered + calibrated data here
  wrench_publisher_ = this->create_publisher<WrenchStamped>("wrench", rclcpp::QoS(10));
}

void XRobotManager::ur_wrench_subscription_callback_(const WrenchStamped::SharedPtr msg) {
  // Drop NaN readings
  if (std::isnan(msg->wrench.force.x) || ...) return;
  { std::lock_guard<std::mutex> lock(wrench_mutex_); current_wrench_ = *msg; }
  wrench_publisher_->publish(*msg);
}
```

- *UR* passes the driver's `wrench_filtered` topic straight through.
- *YNX* subscribes to `force_torque_sensor_broadcaster/wrench` and **flips the x/y
  components** of both force and torque (sensor convention difference) before storing and
  publishing.

### `~/ee_pose`, `~/ee_twist`, `~/ee_accel` publishers

Files:

- UR: [ee_state_publisher.cpp](https://github.com/cmu-mfi/ur_ros2/blob/main/ur_robot_manager/src/ee_state_publisher.cpp)
- YNX: [ee_state_publisher.cpp](https://github.com/cmu-mfi/ynx_ros2/blob/main/ynx_robot_manager/src/ee_state_publisher.cpp)

A 200 Hz timer (in `ee_state_cb_group_`) publishes the end-effector state in
`<tf_prefix>base_link`:

- Setup: resolve the EE link (`getEndEffectorLink()`, falling back to the last link of
  the planning group with a warning), start the MoveIt state monitor, and **block until
  TF `base_link → tool0` yields a valid quaternion** (guards against publishing
  uninitialized poses), then start the timer at 5 ms.
- Loop: `getCurrentPose(tool0)`; compute twist by finite-differencing position
  (linear) and orientation via quaternion difference `q_curr * q_last.inverse()`
  (angular, scaled by `angle/dt`); compute acceleration by finite-differencing the
  twist. Skip ticks with `dt <= 0`. Update the shared state under `ee_state_mutex_` and
  publish all three topics.

---

## 11. Robot-Specific Parts to Adapt

These are the **only** places where the two reference implementations diverge. Everything
else in this guide is identical between them. Diff the repos to see the extent:

- UR: [ur_robot_manager/src/](https://github.com/cmu-mfi/ur_ros2/tree/main/ur_robot_manager/src)
- YNX: [ynx_robot_manager/src/](https://github.com/cmu-mfi/ynx_ros2/tree/main/ynx_robot_manager/src)

| Area | UR ([ur_ros2](https://github.com/cmu-mfi/ur_ros2)) | YNX ([ynx_ros2](https://github.com/cmu-mfi/ynx_ros2)) |
| :--- | :--- | :--- |
| Home joints | `{0, -π/2, π/2, -π/2, -π/2, 0}` | all zeros |
| Park joints | `{π/2, -2.0, 2.6, -2.6, -π/2, 0}` | `{-π/2, -0.5, -1.0, -0.0, 0.9, 0}` |
| `set_io` backend | service call to `io_and_status_controller/set_io` (`ur_msgs`), pins 0–7 | publish to `gpio_command_controller/commands` (`control_msgs`), pins 1–16 |
| `set_payload` backend | service call to `io_and_status_controller/set_payload` (`ur_msgs`) | validated stub (TODO) |
| Wrench source | `force_torque_sensor_broadcaster/wrench_filtered`, pass-through | `force_torque_sensor_broadcaster/wrench`, x/y axes negated |
| Wrench index order in admittance | `[y, x, z, ty, tx, tz]` | same remapping |
| Extra dependency | `ur_msgs` | `control_msgs` |
| Launch specifics | `model` choices (ur3…ur30), factory calibration file | `model` choices (nex10), per-model URDF/SRDF paths |

---

## 12. Step 8 — Bringup Layer

Files (copy and adapt):

- UR: [ur_bringup/launch/robot_manager.launch.py](https://github.com/cmu-mfi/ur_ros2/blob/main/ur_bringup/launch/robot_manager.launch.py)
- YNX: [ynx_bringup/launch/robot_manager.launch.py](https://github.com/cmu-mfi/ynx_ros2/blob/main/ynx_bringup/launch/robot_manager.launch.py)

`robot_manager.launch.py` starts the robot manager node and passes everything MoveIt and
servo need:

1. **Launch arguments:** `log_level` (default `error`), `ns` (empty = root), `model`
   (robot-specific choices), and `tf_prefix` derived as `ns + "_"` if `ns` else `''`.
2. **Robot description:** run xacro on your URDF/SRDF (`robot_description`,
   `robot_description_semantic`).
3. **Kinematics:** `kinematics.yaml` under `robot_description_kinematics`, keyed by
   `<tf_prefix>manipulator`.
4. **Joint limits:** `joint_limits.yaml` under `robot_description_planning`, with each
   joint name prefixed by `tf_prefix` and **`max_velocity` multiplied by 10** (MoveIt
   Servo bug workaround).
5. **Planning scene parameters** (publish geometry/state/transform updates).
6. **Servo parameters:** `servo_parameters.yaml` via `ParameterFile`.
7. The node itself: `package='<robot>_robot_manager'`, namespaced, with `ns` and
   `tf_prefix` parameters.

`bringup.launch.py` then composes the full stack: `driver.launch.py` (robot driver +
controllers) + `moveit.launch.py` (MoveIt + RViz) + `robot_manager.launch.py`.

- UR: [bringup.launch.py](https://github.com/cmu-mfi/ur_ros2/blob/main/ur_bringup/launch/bringup.launch.py)
- YNX: [bringup.launch.py](https://github.com/cmu-mfi/ynx_ros2/blob/main/ynx_bringup/launch/bringup.launch.py)

---

## 13. Step 9 — Build, Run, Test

### Build

Clone this repo (`robot_manager_ros2`) and your `<robot>_ros2` repo into a ROS 2
workspace, then:

```bash
cd <your-ros2-workspace>
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### Run

```bash
# Full stack (driver + MoveIt + robot manager)
ros2 launch <robot>_bringup bringup.launch.py ns:=<optional-namespace>

# Or just the robot manager, if driver + MoveIt are already up
ros2 launch <robot>_bringup robot_manager.launch.py ns:=<optional-namespace>
```

### Exercise every interface

All names below assume an empty namespace; with `ns:=cell1` they become
`/cell1/joint_goal` etc.

```bash
# Actions
ros2 action list -t | grep -E "joint_goal|pose_goal"
ros2 action send_goal /joint_goal robot_manager_interfaces/action/JointGoal \
  "{positions: [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0], velocity_scaling: 0.5, acceleration_scaling: 0.5}"
ros2 action send_goal --feedback /pose_goal robot_manager_interfaces/action/PoseGoal \
  "{target_pose: {position: {x: 0.4, y: 0.0, z: 0.3}, orientation: {w: 1.0}}, velocity_scaling: 0.3, acceleration_scaling: 0.3, method: PTP}"

# Services
ros2 service call /home robot_manager_interfaces/srv/Home "{speed: 0.2}"
ros2 service call /park robot_manager_interfaces/srv/Park "{speed: 0.2}"
ros2 service call /set_io robot_manager_interfaces/srv/SetIo "{pin: 0, state: 1}"
ros2 service call /set_payload robot_manager_interfaces/srv/SetPayload "{mass: 1.0, cog: {x: 0.0, y: 0.0, z: 0.05}}"
ros2 service call /start_admittance_pose_servo robot_manager_interfaces/srv/StartAdmittancePoseServo \
  "{target_wrench: {force: {x: 0.0, y: 0.0, z: -10.0}}, selection_vector: [false, false, true, false, false, false], \
    mass: [1.0, 1.0, 1.0, 0.05, 0.05, 0.05], damping: [100.0, 100.0, 100.0, 20.0, 20.0, 20.0], \
    stiffness: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], max_velocity: [0.1, 0.1, 0.1, 0.2, 0.2, 0.2]}"
ros2 service call /stop_admittance_pose_servo std_srvs/srv/Trigger "{}"

# Streaming servo (keep a steady stream, < 200 ms between messages)
ros2 topic pub --rate 50 /twist_servo geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.05}}}"

# State feedback
ros2 topic echo /wrench
ros2 topic echo /ee_pose
ros2 topic hz /ee_twist
```

### Verify the contract

- [ ] Rejecting a second goal while one runs (e.g. send `home` during a `joint_goal`) —
      the new request must be rejected with the current mode in the log.
- [ ] Cancelling a running action returns result `error_code = 3` and stops the robot.
- [ ] A servo stream that stops mid-motion halts the robot within ~200 ms.
- [ ] `pose_goal`/`pose_servo` with a `target_id` that is not a child of `tool0` is
      rejected with a clear TF error.
- [ ] Action feedback `progress` ramps from 0 to 100 during execution.

---

## 14. Final Checklist

- [ ] `robot_manager_interfaces` is built and every interface in
      [Section 2](#2-the-interface-contract) exists with the exact name and type.
- [ ] Node skeleton: `MultiThreadedExecutor` + setup thread + three callback groups.
- [ ] `ControlMode` machinery with `ModeReset` guards; exclusive mode claiming everywhere.
- [ ] `joint_goal` and `pose_goal` actions: validation, plan, async execute, 20 Hz
      progress feedback, cancel handling, correct error codes.
- [ ] `home`, `park`: robot-specific joint configurations, speed scaling.
- [ ] `set_io`, `set_payload`: robot-appropriate backend, request validation, response
      contract.
- [ ] `pose_servo`, `twist_servo`: streaming targets, 200 ms watchdog, sliding-window
      smoothing, `JointTrajectory` output.
- [ ] `start/stop_admittance_pose_servo` + `~/admittance_pose_servo`: sensor taring,
      admittance math, selection vector, velocity clamping.
- [ ] `~/wrench`: sanitized (and, if needed, convention-corrected) FT data.
- [ ] `~/ee_pose`, `~/ee_twist`, `~/ee_accel`: finite-differenced state at 200 Hz.
- [ ] Frame helpers enforce the `frame_id`/`target_id` contract.
- [ ] Bringup launch passes all MoveIt/servo parameters, including the joint-limit
      workaround.
- [ ] All interfaces tested per Section 13.

If anything behaves unexpectedly, diff your implementation against
[ur_ros2](https://github.com/cmu-mfi/ur_ros2) or
[ynx_ros2](https://github.com/cmu-mfi/ynx_ros2) — the two are nearly identical, so the
difference is almost always in your robot-specific adaptation.
