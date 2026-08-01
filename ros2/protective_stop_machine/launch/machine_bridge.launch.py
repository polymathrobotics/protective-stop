# SPDX-FileCopyrightText: 2026 Polymath Robotics, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Bring up the machine bridge lifecycle node, optionally auto-activating it.

No nav2_lifecycle_manager dependency: we emit the configure/activate transitions
from launch event handlers when autostart:=true (the default).
"""
import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg = get_package_share_directory("protective_stop_machine")
    default_params = os.path.join(pkg, "config", "machine_params.yaml")

    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")

    node = LifecycleNode(
        package="protective_stop_machine",
        executable="machine_bridge_node",
        name="machine_bridge",
        namespace="",
        output="screen",
        parameters=[params_file],
    )

    # autostart: configure on launch, then activate once it reaches 'inactive'.
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart),
    )
    activate_on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        ),
        condition=IfCondition(autostart),
    )

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("autostart", default_value="true"),
        node,
        activate_on_inactive,
        configure,
    ])
