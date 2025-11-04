from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ros_gz_sim.actions import GzServer


def generate_launch_description():
    pkg_simulation_share = FindPackageShare("simulation")
    gz_gui = ExecuteProcess(cmd=["gz", "sim", "-g", "--render-engine", "ogre"])

    return LaunchDescription(
        [
            GzServer(
                world_sdf_file=PathJoinSubstitution(
                    [pkg_simulation_share, "worlds", "vehicle.sdf"]
                )
            ),
            gz_gui,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="bridge_node",
                arguments=[
                    "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
                ],
                output="screen",
            ),
        ]
    )
