from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    real_arg = DeclareLaunchArgument(
        "real", default_value="true",
        description="If true, launch real robot driver + RTDE and set planner publish_joint_states:=False."
    )
    joy_arg = DeclareLaunchArgument(
        "joy", default_value="true",
        description="If true, launch joystick controller."
    )
    gui_arg = DeclareLaunchArgument(
        "gui", default_value="true",
        description="If true, launch sirio_arm_gui."
    )

    real = LaunchConfiguration("real")
    joy = LaunchConfiguration("joy")
    gui = LaunchConfiguration("gui")

    # Package share paths
    pkg_manip = get_package_share_directory("manipulators")
    pkg_rtde  = get_package_share_directory("ur_rtde_controller")
    pkg_adm   = get_package_share_directory("admittance_controller")
    pkg_wbqp  = get_package_share_directory("wbqp_controller")

    # 1) Planner (ur5e_eecam) — publish_joint_states depends on `real`
    #    xacro_args are kept identical to your command.
    ur5e_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_manip, "launch", "ur5e_eecam.launch.py")
        ),
        launch_arguments={
            "publish_joint_states": PythonExpression(["'false'" if real == "true" else "'true'"]),
            "xacro_args": "'camera:=false gripper:=false gfloor:=false gripper_collision_box:=true'",
        }.items()
    )

    # 2) Real robot stack (only if real==true)
    real_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_manip, "launch", "real_control_driver.launch.py")
        ),
        launch_arguments={
            "ur_type": "ur5e",
        }.items(),
        condition=IfCondition(real),
    )

    rtde_ctrl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_rtde, "launch", "rtde_controller.launch.py")
        ),
        launch_arguments={
            "enable_gripper": "false",
            "ROBOT_IP": "192.168.137.102",
        }.items(),
        condition=IfCondition(real),
    )

    # 3) Joystick controller (only if joy==true)
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_manip, "launch", "joystick_controller.launch.py")
        ),
        # Only works if that launch declares an argument you can use to pass the remap.
        launch_arguments={}.items(),
        condition=IfCondition(joy),
    )

    # 4) GUI (only if gui==true) with the remap:
    #    --ros-args -r /manipulator/tcp_force:=/mobile_manipulator/filtered_wrench
    gui_node = Node(
        package="sirio_utilities",
        executable="sirio_arm_gui",
        name="sirio_arm_gui",
        output="screen",
        remappings=[
            ("/manipulator/tcp_force", "/mobile_manipulator/filtered_wrench"),
        ],
        condition=IfCondition(gui),
    )

    # 5) Admittance controller (always, per your sequence)
    admittance = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_adm, "launch", "mobile_ur5e.launch.py")
        ),
        launch_arguments={}.items()
    )

    # 6) WBQP controller (always)
    wbqp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_wbqp, "launch", "wbqp_controller.launch.py")
        ),
        launch_arguments={}.items()
    )

    # 7) TCP pose converter (always)
    tcp_pose_converter = Node(
        package="wbqp_controller",
        executable="tcp_pose_converter",
        name="tcp_pose_converter",
        output="screen",
    )

    return LaunchDescription([
        real_arg, joy_arg, gui_arg,
        ur5e_planner,
        real_driver,
        rtde_ctrl,
        joystick_launch,
        gui_node,
        admittance,
        wbqp,
        tcp_pose_converter,
    ])
