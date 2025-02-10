import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation')
        
        # Create a publisher on the /cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publish the velocity message at a fixed interval
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.publish_velocity)

        self.get_logger().info("Navigation node started, publishing to /cmd_vel")
    
    def publish_velocity(self):
        msg = Twist()
        
        # Set velocity values
        msg.linear.x = 1.0   # Move forward at 0.1 m/s
        msg.linear.y = 0.0   # No sideways motion
        msg.linear.z = 0.0   # No vertical motion
        
        msg.angular.x = 0.0  # No roll
        msg.angular.y = 0.0  # No pitch
        msg.angular.z = 1.0 # Rotate counterclockwise at 0.2 rad/s
        
        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: linear.x={msg.linear.x}, angular.z={msg.angular.z}")


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
