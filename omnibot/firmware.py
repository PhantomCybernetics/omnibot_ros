import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray

import serial

class OmnibotFirmware(Node):

    def __init__(self, ser):
        super().__init__('omnibot_firmware')
        self.ser = ser
        
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/_motors_cmd',
            self.motors_cmd_callback,
            10)
        self.subscription  # prevent unused variable warning

    def motors_cmd_callback(self, msg):
        # self.get_logger().info('Got motors cmd: "%s"' % msg.data)
        if not msg.data or len(msg.data) < 2:
            self.get_logger().err('Got invalid motors cmd: "%s"' % msg.data)
            return
        left_amount = int(msg.data[0])
        right_amount = int(msg.data[1])
        self.get_logger().info(f'Writing motors L={left_amount} R={right_amount}')
        self.ser.write(bytes(f'm:{left_amount};{right_amount}', 'utf-8'))
        line = self.ser.readline()
        print(line)

def main(args=None):
    
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)  # open serial port to esp32
    print(f'Serial open: {ser.name}')
    ser.write(b'a')
    line = ser.readline()
    print(line)
    
    rclpy.init(args=args)

    fw_node = OmnibotFirmware(ser)

    rclpy.spin(fw_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fw_node.destroy_node()
    rclpy.shutdown()
    
    ser.write(b'd')
    ser.close()


if __name__ == '__main__':
    main()