import rclpy
import rclpy.node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.declare_parameter("my_parameter")

    def timer_callback(self):
        # First get the value parameter "my_parameter" and get its string value
        my_param = self.get_parameter("my_parameter").get_parameter_value().string_value

        # Send back a hello with the name
        self.get_logger().info('Hello %s!' % my_param)

        # Then set the parameter "my_parameter" back to string value "world"
        my_new_param = rclpy.parameter.Parameter(
            "my_parameter",
            rclpy.Parameter.Type.STRING,
            "world"
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)

def main():
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
