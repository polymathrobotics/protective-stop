import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    machine_uuid = LaunchConfiguration("machine_uuid")
    machine_uuid_arg = DeclareLaunchArgument(
        "machine_uuid",
        default_value="1111",
        description="Machine UUID",
    )

    heartbeat_timeout = LaunchConfiguration("heartbeat_timeout")
    heartbeat_timeout_arg = DeclareLaunchArgument(
        "heartbeat_timeout",
        default_value=0.1
        description="Heartbeat timeout",
    )


    protective_stop_node = Node(
        package="protective_stop_node",
        executable="protective_stop_node",
        name="protective_stop_node",
        output="screen",
        parameters=[
            {"machine_uuid": machine_uuid},
            {"heartbeat_timeout": heartbeat_timeout}
        ],
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
        parameters=[
            {"autostart": True},
            {"bond_timeout": 0.0},
            {"node_names": ["protective_stop_node"]},
            {"use_sim_time": True},
        ],
    )

    return launch.LaunchDescription([
        heartbeat_timeout_arg,
        machine_uuid_arg,
        protective_stop_node, nav2_lifecycle_manager])
