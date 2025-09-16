# Whole-Body Controller with QP Solver

ROS 2 node wrapping MATLAB-exported Whole-Body QP (from the vendor package `wbqp_codegen_vendor`).
It computes optimal joint/base velocity commands from a Jacobian and desired end-effector twist, then publishes commands and TFs for integration into your robot system.

---

## 1. Dependencies

* `wbqp_codegen_vendor` (provides static libs: `wbqp_init`, `wbqp_solve`, `whole_body_jacobian`, plus headers under `include/<libname>`)
* `rclcpp`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `tf2_ros`, `std_srvs`

Build order:

```bash
colcon build --packages-select wbqp_codegen_vendor
colcon build --packages-select wbqp_controller
```

---

## 2. Topics

* **Subscribes**

  * `/joint_states` (`sensor_msgs/JointState`)
  * `/mobile_manipulator/cmd_vel` (`geometry_msgs/Twist`) — desired EE twist in world frame

* **Publishes**

  * `/cmd_vel` (`geometry_msgs/Twist`) — base velocity command
  * `/manipulator/js_cmd_vel` (`std_msgs/JointState`) — joint speed command (topic name configurable via `topics.q_speed`)
  * `/mobile_robot/pose` (`geometry_msgs/PoseStamped`) — integrated base pose in `map`

---

## 3. TF Frames

* **Dynamic**: `map → mobile_base` (integrated from base velocity)
* **Static**: `mobile_base → world_arm` (translation + rotation offsets)

---

## 4. Parameters

All configuration is handled in YAML:
`config/wbqp_params.yaml`

Includes:

* QP weights and limits
* Jacobian column mapping
* Control timestep `dt`
* Frame names
* Topic names

---

## 5. Utilities

### 5.1 Robotic Arm Kinematics

Run a planner (disable publishing joint states if you are using the real robot):

```bash
ros2 launch manipulators ur5e_eecam.launch.py publish_joint_states:=False xacro_args:='camera:=false gripper:=false gfloor:=false gripper_collision_box:=true tcp_gripper:=0.08'
```

Manipulator menu:

```bash
ros2 run manipulators manipulator_menu_user
```

### 5.2 Real Robot Drivers

Launch robot drivers:

```bash
ros2 launch manipulators real_control_driver.launch.py ur_type:=ur5e
```

UR hardware communication:

```bash
ros2 launch ur_rtde_controller rtde_controller.launch.py enable_gripper:=false ROBOT_IP:=192.168.137.102
```

### 5.3 Interfaces

Joystick teleop for the arm:

```bash
ros2 launch manipulators joystick_controller.launch.py
```

Keyboard teleop for the whole-body platform:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/mobile_manipulator/cmd_vel
```

GUI:

```bash
ros2 run sirio_utilities sirio_arm_gui --ros-args -r /manipulator/tcp_force:=/mobile_manipulator/filtered_wrench
```

### 5.4 Force-Driven Control

Admittance controller:

```bash
ros2 launch admittance_controller mobile_ur5e.launch.py
```

Admittance interface:

```bash
ros2 run admittance_controller admittance_menu_node --ros-args -p manipulator_name:=mobile_manipulator
```

---

## 6. Running the Controller

### 6.1 Standard Launch

```bash
ros2 launch wbqp_controller wbqp_controller.launch.py
```

Enable or disable control with the menu:

```bash
ros2 run wbqp_controller wbqp_menu
```

Publish TCP pose in global `map` frame:

```bash
ros2 run wbqp_controller tcp_pose_converter
```

Send a simple EE twist:

```bash
ros2 topic pub /mobile_manipulator/cmd_vel geometry_msgs/Twist \
"{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

Observe outputs:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /manipulator/js_cmd_vel
ros2 topic echo /base_pose
```

Inspect TFs with tools like `tf2_tools` or `rviz2`.

---

## 7. Debug Mode (Manual Inputs & Menu)

For development and testing, you can run the controller directly in a terminal and inject your own inputs.

1. Load params from the installed config file:

```bash
PARAMS="$(ros2 pkg prefix wbqp_controller)/share/wbqp_controller/config/wbqp_params.yaml"

ros2 run wbqp_controller wbqp_controller \
  --ros-args \
  --params-file "$PARAMS"
```

2. A **menu** will appear in the terminal. Use it to:

   * Set joint positions (degrees, converted internally to radians)
   * Set base translation/orientation
   * Set desired twist
   * Step the solver **one iteration at a time**

### Example Debug Workflow

* Run the node as above.
* In the menu, press **1** → enter 6 joint angles (deg).
* Press **2** → enter base orientation (Euler angles in deg).
* Press **7** → step once and compute outputs.
* Observe results in the terminal or echo the published topics.

This is useful for profiling, checking timing, and debugging solver behavior without requiring real sensors or teleop inputs.

### Example Whole-Body Control

You can run all the above mentioned files (without interfaces) with the following command:

```bash
ros2 launch wbqp_controller wb_ur5e_admittance.launch.py real:=true joy:=true gui:=true
```

---

## 8. Disclaimer

Maintained by **Italo Almirante**.
For issues or collaboration opportunities, please contact the maintainer.
