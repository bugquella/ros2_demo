from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cpp_parameters2",
            node_executable="parameter_node",
            name="custom_parameter_node",
            output="screen",
            parameters=[
                {"my_parameter": "earth"}
            ]
        )
    ])
