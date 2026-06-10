import os 

from typing import List

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription,
    OpaqueFunction, 
    Shutdown
    )
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution
    )
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs) -> List[DeclareLaunchArgument]:
    
    stretch_driver_params = {
        "use_cameras": LaunchConfiguration("use_cameras"),
        "use_rviz": LaunchConfiguration("use_rviz"),
        "use_mujoco_viewer": LaunchConfiguration("use_mujoco_viewer"),
        "use_robocasa": LaunchConfiguration("use_robocasa"),
        "scene_xml": LaunchConfiguration("scene_xml"),
        "scene_name": LaunchConfiguration("scene_name"),
        "mode": LaunchConfiguration("mode"),
        "broadcast_odom_tf": LaunchConfiguration("broadcast_odom_tf"),
        "controller_calibration_file": LaunchConfiguration("controller_calibration_file"),
        "fail_out_of_range_goal": LaunchConfiguration("fail_out_of_range_goal"),
        "fail_if_motor_initial_point_is_not_trajectory_first_point": LaunchConfiguration("fail_if_motor_initial_point_is_not_trajectory_first_point"),
        "action_server_rate": LaunchConfiguration("action_server_rate"),
        "joint_state_rate": LaunchConfiguration("joint_state_rate"),
        "timeout": LaunchConfiguration("timeout"),
        "default_goal_timeout_s": LaunchConfiguration("default_goal_timeout_s"),
    }



    stretch_mujoco_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("stretch_simulation"),   
                "launch",
                "stretch_mujoco_driver.launch.py"
        ])
    ),
    launch_arguments=stretch_driver_params.items()
    )
    
    return [stretch_mujoco_simulation_launch]

def generate_launch_description() -> LaunchDescription:

    return LaunchDescription(
        generate_declared_arguments() +
        [OpaqueFunction(function=launch_setup)]
    )


def generate_declared_arguments() -> List[DeclareLaunchArgument]:

    return [

        DeclareLaunchArgument(
            "broadcast_odom_tf",
            default_value="true",
            choices=["true", "false"],
            description="Whether to broadcast the odom TF",
        ),
        DeclareLaunchArgument(
            "fail_out_of_range_goal",
            default_value="false",
            choices=["true", "false"],
            description="Whether the motion action servers fail on out-of-range commands",
        ),
        DeclareLaunchArgument(
            "fail_if_motor_initial_point_is_not_trajectory_first_point",
            default_value="false",
            choices=["true", "false"],
            description="Whether the motion action servers fail on mismatched starting points",
        ),
        DeclareLaunchArgument(
            "controller_calibration_file",
            default_value=os.path.join(get_package_share_path("stretch_core"), "config", "controller_calibration_head.yaml")
        ),
        DeclareLaunchArgument(
            "mode",
            default_value="position",
            choices=["position", "navigation", "trajectory", "gamepad"],
            description="The mode in which the ROS driver commands the robot",
        ),
        DeclareLaunchArgument(
            "action_server_rate", default_value='30.0', description="Action server update rate",
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="true", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "use_mujoco_viewer", default_value="true", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "use_cameras", default_value="false", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "use_robocasa", default_value="false", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "scene_xml", 
            default_value=os.path.join(get_package_share_path("hello_stretch_sim_bringup"), "scenes", "radiation_room.xml"), 
            description='The absolute path to a Mujoco scene xml',
        ),
        DeclareLaunchArgument(
            "scene_name", 
            default_value="", 
            description='The name of a scene xml within stretch4_mujoco/models, e.g. some_rooms',
        ),
        DeclareLaunchArgument(
            "joint_state_rate", default_value="30.0", description="Joint state update rate",
        ),
        DeclareLaunchArgument(
            "timeout", default_value="0.5", description="Timeout (sec) after which Twist/Joy commands are considered stale",
        ),
        DeclareLaunchArgument(
            "default_goal_timeout_s", default_value='10.0', description="Timeout (sec) for goal execution",
        ),
    ]
