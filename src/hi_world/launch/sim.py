import os
import launch
from pathlib import Path
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    package_dir = Path(get_package_share_directory("hi_world"))
    urdf_path = package_dir / "models" / "robot.urdf"

    webots = WebotsLauncher(world=package_dir / "worlds" / "world.wbt")

    robot_driver = WebotsController(
        robot_name="my_robot",
        parameters=[
            {"robot_description": urdf_path},
        ],
    )

    return LaunchDescription(
        [
            webots,
            robot_driver,
            launch.actions.RegisterEventHandler(
                event_handler=launch.event_handlers.OnProcessExit(
                    target_action=webots,
                    on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
                )
            ),
        ]
    )
