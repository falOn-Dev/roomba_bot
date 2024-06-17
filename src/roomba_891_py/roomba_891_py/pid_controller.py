import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        self.declare_parameter('kp', 0.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)

        self.declare_parameter('setpoint_topic', f'/{self.get_name()}_setpoint')
        self.declare_parameter('measured_topic', f'/{self.get_name()}_measured')
        self.declare_parameter('output_topic', f'/{self.get_name()}_output')

        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value

        self.setpoint_topic = self.get_parameter('setpoint_topic').get_parameter_value().string_value
        self.measured_topic = self.get_parameter('measured_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.input_sub = self.create_subscription(Float64, self.measured_topic, self.input_callback, 10)
        self.setpoint_pub = self.create_subscription(Float64, self.setpoint_topic, self.setpoint_callback, 10)
        self.output_pub = self.create_publisher(Float64, self.output_topic, 10)

        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = self.get_clock().now().nanoseconds
        self.setpoint = 0.0

    def setpoint_callback(self, msg: Float64):
        self.setpoint = msg.data
        self.get_logger().info(f'Setpoint: {self.setpoint}')

    def input_callback(self, msg: Float64):
        # Get the current time
        current_time = self.get_clock().now().nanoseconds

        # Calculate the time difference
        dt = (current_time - self.prev_time) / 1e9

        # Calculate the error
        error = msg.data

        # Calculate the proportional term
        p_term = self.kp * error

        # Calculate the integral term
        self.integral += error * dt
        i_term = self.ki * self.integral

        # Calculate the derivative term
        d_term = self.kd * (error - self.prev_error) / dt

        # Calculate the control signal
        control_signal = p_term + i_term + d_term

        # Publish the control signal
        control_signal_msg = Float64()
        control_signal_msg.data = control_signal
        self.output_pub.publish(control_signal_msg)

        # Update the previous error and time
        self.prev_error = error
        self.prev_time = current_time

        self.get_logger().info(f'Error: {error}, Control Signal: {control_signal}, i: {i_term}, p: {p_term}, d: {d_term}')

def main():
    rclpy.init()
    node = PIDController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()