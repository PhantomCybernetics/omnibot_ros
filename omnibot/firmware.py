import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray

import serial

class OmnibotFirmware(Node):

    def __init__(self):
        super().__init__('omnibot_firmware')
        
        ser = serial.Serial('/dev/tty.usbserial-0001')  # open serial port to esp32
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/_motors_cmd',
            self.motors_cmd_callback,
            10)
        self.subscription  # prevent unused variable warning

    def motors_cmd_callback(self, msg):
        self.get_logger().info('Got motors cmd: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    fw_node = OmnibotFirmware()

    rclpy.spin(fw_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fw_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()