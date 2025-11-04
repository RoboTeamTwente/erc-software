from launch import LaunchDescription
from ament_index_python.packages import get_package_share_path
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ros_gz_sim.actions import GzServer


def generate_launch_description():
    assets_dir = get_package_share_path("simulation")

    with open(assets_dir / "models" / "vehicle" / "model.sdf") as f:
        robot_sdf = f.read()

    return LaunchDescription(
        [
            # arguments
            DeclareLaunchArgument(
                "gzgui",
                default_value="False",
                description="Open Gazebo GUI.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="False",
                description="Open RViz2.",
            ),
            # simulator
            GzServer(world_sdf_file=(assets_dir / "worlds" / "vehicle.sdf").as_posix()),
            # ROS<->GZ bridge
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/model/vehicle/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                    "/world/demo/model/vehicle/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
                    "/world/demo/model/vehicle/link/gps/sensor/navsat/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
                ],
                parameters=[
                    {"qos_overrides./tf_static.publisher.durability": "transient_local"}
                ],
                remappings={
                    "/model/vehicle/cmd_vel": "/cmd_vel",
                    "/world/demo/model/vehicle/joint_state": "/joint_states",
                }.items(),
                output="screen",
            ),
            # Calculators
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "frame_prefix": "vehicle/",
                        "robot_description": robot_sdf,
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            ),
            # GUIs
            ExecuteProcess(
                cmd=["gz", "sim", "-g", "--render-engine", "ogre"],
                condition=IfCondition(LaunchConfiguration("gzgui")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
        ]
    )
