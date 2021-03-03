from launch import LaunchDescription
import launch.actions
from launch_ros.substitutions import ExecutableInPackage

def generate_launch_description():
    executable = ExecutableInPackage(package='cpp_parameters2', executable='parameter_node')
    return LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=[executable],
            output='screen')
    ]) 
