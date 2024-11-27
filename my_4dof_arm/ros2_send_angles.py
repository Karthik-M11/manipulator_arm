import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int32MultiArray, String
from kinematics import Kinematics
import numpy as np
import serial  # PySerial for communication with ESP32


class JointAnglePublisher(Node):
    def __init__(self):
        super().__init__('joint_angle_publisher')

        # Create a publisher for the "joint" topic
        self.publisher_ = self.create_publisher(Quaternion, 'joint', 10)

        # Subscribers for button states and input string
        self.subscription = self.create_subscription(
            Int32MultiArray,
            'button_states',
            self.button_state_callback,
            10
        )
        self.subscription = self.create_subscription(
            String,
            'input_string',
            self.string_callback,
            10
        )

        # Initialize serial communication with ESP32
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Adjust port and baud rate as needed
            self.get_logger().info('Serial connection established with ESP32.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to ESP32: {e}')
            self.serial_port = None

        # Kinematics and predefined scenarios
        self.kinematics = Kinematics(
            [0, 1, 1, 1],
            [1, 0, 0, 0],
            [np.pi / 2, 0, 0, 0]
        )
        self.scenarios = {
            1: [30, 45, 90, 100],
            2: [60, 30, 120, 200],
            3: [90, 90, 0, 0],
            4: [45, 10, 180, 50],
            5: [0, 0, 0, 0],
            6: [15, 60, 45, 150],
            7: [15, 60, 135, 150],
            8: [50, 50, 50, 100],
            9: [90, 20, 70, 250],
            10: [120, 80, 160, 300],
            11: [120, 80, 160, 300],
            12: [120, 80, 160, 80],
        }

        self.home = [100, 100, 100, 100]
        self.home_check = 0
        self.button_state_array = []
        self.current_scenario = 1
        self.string = ''

        # Timer for periodically publishing joint angles
        self.timer = self.create_timer(1.0, self.publish_joint_angles)  # Publish every 1 second

    def button_state_callback(self, msg):
        self.button_state_array = msg.data
        self.get_logger().info(f'Updated button states: {self.button_state_array}')

        if self.button_state_array:
            button_state = self.button_state_array[0]
            if button_state in self.scenarios:
                self.current_scenario = button_state
                self.get_logger().info(f'Scenario updated to: {self.current_scenario}')
            else:
                self.get_logger().warn('Invalid button state received.')

    def string_callback(self, msg):
        self.string = msg.data

    def parse_command(self, input_str):
        cleaned_input = input_str[1:-1]
        parts = cleaned_input.split('*')
        if len(parts) != 3:
            raise ValueError("Invalid command format. Expected format is $Funcn*Action*Value#")

        Funcn = int(parts[0])
        Action = int(parts[1])
        Value = int(parts[2])

        result = [0, 0, 0, 0]
        if Funcn == 1:
            result[0] = Value if Action == 1 else -Value
        elif Funcn == 2:
            result[1] = Value if Action == 1 else -Value
        elif Funcn == 3:
            result[2] = Value if Action == 1 else -Value
        elif Funcn == 4:
            result[3] = 1 if Action == 4 else 0

        return result

    def publish_joint_angles(self):
        msg = Quaternion()
        if self.string != 0:
            # angles = self.parse_command(self.string)
            angles = [0, 0, 0, 0]
            self.string = ''
        # else:
        #     angles = self.scenarios.get(self.current_scenario, [0, 0, 0, 0])

        msg.x, msg.y, msg.z, msg.w = map(float, angles)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published joint angles: {angles}')

        # Send angles to ESP32 as a comma-separated string
        if self.serial_port and self.serial_port.is_open:
            try:
                angle_str = ','.join(map(str, angles)) + '\n'
                self.serial_port.write(angle_str.encode('utf-8'))
                self.get_logger().info(f'Sent angles to ESP32: {angle_str.strip()}')
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to send data to ESP32: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = JointAnglePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
