import rclpy
from rclpy.node import Node
import serial
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist



class Interface(Node):
    def __init__(self):
        super().__init__('interface')
        self.get_logger().info("Hello, jak w domu")
        
        self.port = '/dev/ttyACM0'
        #self.port2 = '/dev/ttyACM1'  #define second port in case the teensy is not on the first one
        self.baudrate = 115200
        self.ser = None

        # Initialize the serial connection
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
            self.get_logger().info("Connected to serial port: :" + self.port)
            # self.ser.readline()   #remove existing data reading
            # self.ser.flushInput()
            # self.ser.flushOutput()
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            return

        
        # ROS 2 subscriber to receive velocity commands
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # # Init Odometry
        # self.velocity = 0.0
        # self.angular_velocity = 0.0
        # self.x = 0.0
        # self.y = 0.0
        # self.angle = 0.0
        # self._last_odom_time = time.time()

    def cmd_vel_callback(self, msg):
        """Processes received Twist messages and sends them to the Teensy."""
        try:
            # Format the message (example: "v0.5,o0.2\n")
            output_str = f'v{msg.linear.x:.2f},o{msg.angular.z:.2f}\n'
            
            # Send data over serial
            self.ser.write(output_str.encode('utf-8'))
            
            self.get_logger().info(f"Sent to Teensy: {output_str.strip()}")
        except Exception as e:
            self.get_logger().error(f"Error writing to serial: {e}")

    def destroy_node(self):
        """Closes serial connection when shutting down."""
        if self.ser:
            self.ser.close()
        super().destroy_node()




def main(args=None):
    rclpy.init(args=args)
    node = Interface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Teensy interface node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
