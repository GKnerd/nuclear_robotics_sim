import os 

import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from utils.scene_generator import write_scene_from_config

from typing import List, Dict
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
    Shutdown,
    )
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution
    )
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare
from hello_helpers.multi_yaml import MultiYaml


def launch_setup(context, *args, **kwargs) -> List[DeclareLaunchArgument]:

    scene_config = LaunchConfiguration("scene_config").perform(context)
    stretch_driver_params: Dict[str, LaunchConfiguration] = {
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
        "use_sim_time": LaunchConfiguration("use_sim_time")
    }

    if scene_config:
        stretch_driver_params["scene_xml"] = write_scene_from_config(config_path=scene_config, 
                                                template_path=stretch_driver_params["scene_xml"].perform(context)
        )

    autonomous = LaunchConfiguration("autonomous").perform(context) == "true"
    if autonomous:
        # nav2 brings its own RViz (navigation.rviz); avoid launching a second one.
        stretch_driver_params["use_rviz"] = "false"

    stretch_mujoco_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("stretch_simulation"),
                "launch",
                "stretch_mujoco_driver.launch.py",
            ])
        ),
        launch_arguments=stretch_driver_params.items(),
    )

    nodes = [stretch_mujoco_simulation_launch]

    if autonomous:
        # Reuse the existing nav2 core. NOT navigation_mppi.launch.py -- that also
        # starts the real-robot driver + lidar; in sim our driver already provides
        # those. Mirror how navigation_mppi wires nav_core (MPPI params via MultiYaml)
        # and add the footprint publisher.
        stretch_nav2 = FindPackageShare("stretch_nav2")
        stretch_core = FindPackageShare("stretch_core")

        footprint_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([stretch_core, "launch", "robot_footprint.launch.py"])
            ),
            launch_arguments={"tool_preset": "sg4"}.items(),
        )

        nav2_launch = GroupAction([
            SetLaunchConfiguration("use_sim_time", "false"),  # nav_core doesn't forward it
            SetRemap("cmd_vel", "/stretch/cmd_vel"),         # nav2 output -> sim driver
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([stretch_nav2, "launch", "include", "nav_core.launch.py"])
                ),
                launch_arguments={
                    "map": LaunchConfiguration("map"),
                    "use_slam": "False",
                    "use_rviz": LaunchConfiguration("use_rviz"),
                    "use_composition": "False",  # so the cmd_vel SetRemap reaches nav2 nodes
                    "params_file": MultiYaml([
                        PathJoinSubstitution([stretch_nav2, "config", "original_nav2_params.yaml"]),
                        PathJoinSubstitution([stretch_nav2, "config", "nav2_params_core.yaml"]),
                        PathJoinSubstitution([stretch_nav2, "config", "nav2_params_mppi.yaml"]),
                        PathJoinSubstitution([stretch_nav2, "config", "mppi_params.yaml"]),
                    ]),
                }.items(),
            ),
        ])

        nodes += [footprint_launch, nav2_launch]

    return nodes

def generate_launch_description() -> LaunchDescription:

    return LaunchDescription(
        generate_declared_arguments() + [OpaqueFunction(function=launch_setup)]
    )


def generate_declared_arguments() -> List[DeclareLaunchArgument]:

    return [

        DeclareLaunchArgument(
            "autonomous",
            default_value="false",
            choices=["true", "false"],
            description="true = launch the nav2 stack (needs map:=); false = sim only (run teleop separately)",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use sim time or not",
        ),
        DeclareLaunchArgument(
            "map",
            default_value="",
            description="Path to the map .yaml for autonomous (nav2) mode; required when operation:=autonomous",
        ),
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
            default_value="navigation",
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
            "scene_config",
            default_value=os.path.join(get_package_share_path("hello_stretch_sim_bringup"), "config", "radiation_room.yaml"),
            description="Path to a YAML scene config (config/radiation_room.yaml). "
                        "If set, the scene XML is generated from it and overrides scene_xml.",
        ),
        DeclareLaunchArgument(
            "scene_xml",
            default_value=os.path.join(get_package_share_path("hello_stretch_sim_bringup"), "scenes", "radiation_room.xml"),
            description='The absolute path to a Mujoco scene xml (used when scene_config is empty)',
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
