# WHOLE BODY CONTROLLER WITH QP SOLVER

ROS 2 node that wraps MATLAB-exported Whole-Body QP (via the vendor package `wbqp_codegen_vendor`).
It consumes `WholeBodyJacobian`, `wbqp_init`, and `wbqp_solve` from the vendor, subscribes to joint state and EE twist, solves the QP, and publishes base/joint commands, while broadcasting TFs and a pose.

---

## Depends on

- `wbqp_codegen_vendor` (builds & installs static libs: `wbqp_init`, `wbqp_solve`, `whole_body_jacobian`, and headers under `include/<libname>`)
- `rclcpp`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `tf2_ros`, `std_srvs`

Ensure the vendor package is built first:

```bash
colcon build --packages-select wbqp_codegen_vendor
```

Then build this:

```bash
colcon build --packages-select wbqp_controller
```

---

## Topics

- Subscribes
  - `/joint_states` (`sensor_msgs/JointState`)
  - `/mobile_manipulator/cmd_vel` (`geometry_msgs/Twist`) — desired EE twist in world

- Publishes
  - `/cmd_vel` (`geometry_msgs/Twist`) — base cmd
  - `/manipulator/js_cmd_vel` (`std_msgs/Float64MultiArray`) — joint speed cmd (param-driven via `topics.q_speed`)
  - `/mobile_robot/pose` (`geometry_msgs/PoseStamped`) — integrated base pose in `map`

## TF

- **dynamic**: `map → mobile_base` (integrated from Vx, Vy, Ωz)
- **static**: `mobile_base → world_arm` (from translation and rotation offsets)

---

## Parameters (YAML)

See `config/wbqp_params.yaml` — includes QP weights/limits, Jacobian column mapping, dt, frames, topics.

---

## Utils

### Robotic arm kinematics

Planner (set _publish_joint_states:=False_ when the real robot is used):

```bash
ros2 launch manipulators ur5e_eecam.launch.py publish_joint_states:=True xacro_args:='camera:=false gripper:=false gfloor:=false gripper_collision_box:=true'
```

Manipulator menu:
```bash
ros2 run manipulators manipulator_menu_user
```

### Real robot

Drivers:
```bash
ros2 launch manipulators real_control_driver.launch.py ur_type:=ur5e
```

Hardware communication for UR:
```bash
ros2 launch ur_rtde_controller rtde_controller.launch.py enable_gripper:=false ROBOT_IP:=192.168.137.102 
```

### Interfaces

Joystick teleop for the arm:
```bash
ros2 launch manipulators joystick_controller.launch.py 
```

Keyboard teleop for the whole-body platform:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/mobile_manipulator/cmd_vel
```

Gui:
```bash
ros2 run sirio_utilities sirio_arm_gui --ros-args -r /manipulator/tcp_force:=/mobile_manipulator/filtered_wrench
```

### Force_driven control

Admittance controller:
```bash
ros2 launch admittance_controller mobile_ur5e.launch.py
```

Admittance interface:
```bash
ros2 run admittance_controller admittance_menu_node --ros-args -p manipulator_name:=mobile_manipulator
```

---

## Practical test example

1) Run the node

```bash
ros2 launch wbqp_controller wbqp_controller.launch.py
```

2) Enable or disable the control through the menu:

```bash
ros2 run wbqp_controller wbqp_menu
```

3) Publish tcp pose in the global _map_ frame
```bash
ros2 run wbqp_controller tcp_pose_converter
```

4) Send a simple EE twist

```bash
ros2 topic pub /mobile_manipulator/cmd_vel geometry_msgs/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

5) Observe outputs

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /manipulator/js_cmd_vel
ros2 topic echo /base_pose
```

TFs can be inspected with tools like `tf2_tools` or `rviz2` .

---

## Disclaimer

Contact the maintainer Italo Almirante for any issue with the pkg or for collabs.

