# WHOLE BODY CONTROLLER WITH QP SOLVER

ROS 2 node that wraps MATLAB-exported Whole-Body QP (via the vendor package `wbqp_codegen_vendor`).
It consumes `WholeBodyJacobian`, `wbqp_init`, and `wbqp_solve` from the vendor, subscribes to joint state and EE twist, solves the QP, and publishes base/joint commands, while broadcasting TFs and a pose.

---

## Depends on

- `wbqp_codegen_vendor` (builds & installs static libs: `wbqp_init`, `wbqp_solve`, `whole_body_jacobian`, and headers under `include/<libname>`)
- `rclcpp`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `tf2_ros`

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
  - `/manipulator/js_cmd_vel` (`std_msgs/Float64MultiArray`) — joint speed cmd (param-driven
    via `topics.q_speed`)
  - `/mobile_robot/pose` (`geometry_msgs/PoseStamped`) — integrated base pose in `map`

## TF

- **dynamic**: `map → base` (integrated from Vx, Vy, Ωz)
- **static**: `base → base_link` (from translation and rotation offsets)

---

## Parameters (YAML)

See `config/params.yaml` — includes QP weights/limits, Jacobian column mapping, dt, frames, topics.

---

## Launch

```bash
ros2 launch wbqp_controller wbqp_controller.launch.py
```

---

## Practical test example

1) Run the node

```bash
ros2 launch wbqp_controller wbqp_controller.launch.py
```

2) Send a simple EE twist

```bash
ros2 topic pub /mobile_manipulator/cmd_vel geometry_msgs/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

3) Observe outputs

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /manipulator/js_cmd_vel
ros2 topic echo /base_pose
```

TFs can be inspected with tools like `tf2_tools` or `rviz2` .

---

## Disclaimer

Contact the maintainer Italo Almirante for any issue with the pkg or for collabs.

