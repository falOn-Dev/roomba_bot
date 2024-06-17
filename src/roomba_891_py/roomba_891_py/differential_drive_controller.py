import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class DifferentialDriveController(Node):
    def __init__(self):
        super().__init__('differential_drive_controller')
        
        self.declare_parameter('wheel_radius', 0.0375)
        self.declare_parameter('wheel_sep', 0.220)

        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_sep = self.get_parameter('wheel_sep').get_parameter_value().double_value

        self.left_wheel_pub = self.create_publisher(Float64, '/left_wheel_velocity', 10)
        self.right_wheel_pub = self.create_publisher(Float64, '/right_wheel_velocity', 10)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg: Twist):
        # Calculate the linear and angular velocities
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Calculate the left and right wheel velocities
        left_wheel_vel = (linear_vel - (angular_vel * self.wheel_sep / 2)) / self.wheel_radius
        right_wheel_vel = (linear_vel + (angular_vel * self.wheel_sep / 2)) / self.wheel_radius

        # Publish the left and right wheel velocities
        left_wheel_msg = Float64()
        left_wheel_msg.data = left_wheel_vel
        self.left_wheel_pub.publish(left_wheel_msg)

        right_wheel_msg = Float64()
        right_wheel_msg.data = right_wheel_vel
        self.right_wheel_pub.publish(right_wheel_msg)

        # self.get_logger().info(f'vX: {linear_vel}, vTheta: {angular_vel} -> vL: {left_wheel_vel}, vR: {right_wheel_vel}')

def main():
    rclpy.init()
    node = DifferentialDriveController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()