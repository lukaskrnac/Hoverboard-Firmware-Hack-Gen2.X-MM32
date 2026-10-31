"""One hoverboard wheel on odrive_ros2_control: ros2 launch ros/hoverboard_wheel.launch.py

Start uart_can_bridge.py on vcan0 first. Commands: /wheel_velocity_controller/commands (rad/s).
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    with open(os.path.join(HERE, "hoverboard_wheel.urdf")) as f:
        robot_description = {"robot_description": f.read()}
    controllers = os.path.join(HERE, "hoverboard_controllers.yaml")
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[robot_description]),
        Node(package="controller_manager", executable="ros2_control_node", parameters=[controllers], output="screen"),
        Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"]),
        Node(package="controller_manager", executable="spawner", arguments=["wheel_velocity_controller"]),
    ])
