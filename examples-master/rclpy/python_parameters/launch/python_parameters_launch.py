from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="python_parameters",
            node_executable="param_talker",
            name="parameter_node",
            output="screen",
            parameters=[
                {"my_parameter": "earth"}
            ]
        )
    ])
