import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/zed/zed_node/pose', 10)
        timer_period = 0.1  # segundos (10 Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('Publicando PoseStamped no /zed/zed_node/pose...')

    def timer_callback(self):
        msg = PoseStamped()

        # Preenche o header
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = 'zed_camera_center'  # frame da ZED

        # Define a posição (x, y, z)
        msg.pose.position.x = 1.0
        msg.pose.position.y = 2.0
        msg.pose.position.z = 1.0

        # Define a orientação (quaternion)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.publisher_.publish(msg)
        self.get_logger().info(f'Pose publicada: {msg.pose.position}')

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
