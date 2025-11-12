from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    assets_dir = get_package_share_path("simulation")

    simulator = WebotsLauncher(
        world=assets_dir / "worlds" / "simulation.wbt",
        ros2_supervisor=True,
    )

    rover = WebotsController(
        robot_name="Rover",
        parameters=[{"robot_description": assets_dir / "models" / "rover.urdf"}],
        respawn=True,
    )

    exit_with_webots = RegisterEventHandler(
        OnProcessExit(target_action=simulator, on_exit=[EmitEvent(event=Shutdown())])
    )

    return LaunchDescription(
        [
            # arguments
            DeclareLaunchArgument(
                "rviz",
                default_value="False",
                description="Open RViz2.",
            ),
            # simulator
            simulator,
            simulator._supervisor,
            rover,
            exit_with_webots,
            # GUIs
            Node(
                package="rviz2",
                executable="rviz2",
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
