import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int32MultiArray, String  # Message type for multi-element button states
from kinematics import Kinematics
import numpy as np

class JointAnglePublisher(Node):
    def __init__(self):
        super().__init__('joint_angle_publisher')

        # Create a publisher for the "joint_states" topic
        self.publisher_ = self.create_publisher(Quaternion, 'joint', 10)

        # Create a subscriber for the "button_states" topic (multi-element array)
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

        # Kinematics object for IK calculations (not used here, but can be integrated)
        self.kinematics = Kinematics(
            [0, 1, 1, 1],
            [1, 0, 0, 0],
            [np.pi / 2, 0, 0, 0]
        )

        # Predefined scenarios (joint angles for each scenario)
        self.scenarios = {
            1: [30, 45, 90, 100],  # Scenario 1
            2: [60, 30, 120, 200],  # Scenario 2
            3: [90, 90, 0, 0],      # Scenario 3
            4: [45, 10, 180, 50],   # Scenario 4
            5: [0, 0, 0, 0],        # Scenario 5
            6: [15, 60, 45, 150],   # Scenario 6
            7: [15, 60, 135, 150],  # Scenario 7
            8: [50, 50, 50, 100],   # Scenario 8
            9: [90, 20, 70, 250],   # Scenario 9
            10: [120, 80, 160, 300],# Scenario 10
            11: [120, 80, 160, 300],# Scenario 11
            12: [120, 80, 160, 80], # Scenario 12
        }

        self.home = [100,100,100,100]
        self.home_check = 0

        # Store button states in an array (list)
        self.button_state_array = []

        # Current scenario state (default to 1)
        self.current_scenario = 1
        self.val = 0

        # Store string value
        self.string = ''

        # Timer for periodically publishing joint angles
        self.timer = self.create_timer(1.0, self.publish_joint_angles)  # Publish every 1 second

    def button_state_callback(self, msg):
        # Update the button states array with the new data from the message
        self.button_state_array = msg.data
        self.get_logger().info(f'Updated button states: {self.button_state_array}')

        # Determine the scenario to use based on button states
        # You can implement logic here to select a scenario based on button states.
        # For simplicity, let's assume the first button in the array determines the scenario
        if self.button_state_array:
            button_state = self.button_state_array[0]  # Use first button state for scenario
            if button_state in self.scenarios:
                self.current_scenario = button_state
                self.get_logger().info(f'Scenario updated to: {self.current_scenario}')
            else:
                self.get_logger().warn('Invalid button state received.')

    def string_callback(self, msg):
        self.string = msg.data


    def parse_command(self, input_str):
        """
        Parses the input command string and returns the corresponding joint angles or actions.
        
        :param input_str: Command string in the format $Funcn*Action*Value#
        :return: List representing the joint angles/actions [joint1, joint2, joint3, joint4].
        """
        # Remove the starting '$' and ending '#' to clean the input
        cleaned_input = input_str[1:-1]
        
        # Split the string by '*' to extract Funcn, Action, and Value
        parts = cleaned_input.split('*')
        if len(parts) != 3:
            raise ValueError("Invalid command format. Expected format is $Funcn*Action*Value#")
        
        Funcn = int(parts[0])  # Joint or action identifier (1, 2, 3, 4, etc.)
        Action = int(parts[1])  # Action (Clockwise, Anti-Clockwise, Pick, Drop)
        Value = int(parts[2])  # Angle or action value
        
        # Initialize the result array for 4 joints [Base, ARM1, ARM2, End Effector]
        result = [0, 0, 0, 0]  # Default values (0 indicates no movement or no action)
        
        # Assign the values based on the Funcn and Action
        if Funcn == 1:  # Base Rotation
            if Action == 1:  # Clockwise
                result[0] = Value
            elif Action == 2:  # Anti-clockwise
                result[0] = -Value  # Negative value for anti-clockwise rotation
        elif Funcn == 2:  # ARM1 Rotation
            if Action == 1:  # Clockwise
                result[1] = Value
            elif Action == 2:  # Anti-clockwise
                result[1] = -Value  # Negative value for anti-clockwise rotation
        elif Funcn == 3:  # ARM2 Rotation
            if Action == 1:  # Clockwise
                result[2] = Value
            elif Action == 2:  # Anti-clockwise
                result[2] = -Value  # Negative value for anti-clockwise rotation
        elif Funcn == 4:  # End Effector Action
            if Action == 4:  # Pick
                result[3] = 1  # Pick action
            elif Action == 5:  # Drop
                result[3] = 0  # Drop action
        
        return result


    def publish_joint_angles(self):
        # Publish joint angles based on the current scenario
        msg = Quaternion()
        
        # if self.button_state_array == []:
        #     angles = [0,0,0,0]
        # elif self.val < len(self.button_state_array):
        #     self.current_scenario = self.button_state_array[self.val]
        #     angles = self.scenarios[self.current_scenario]
        #     self.val += 1
        # else:
        #     angles = [0,0,0,0]

        if self.string != '':
            angles = self.parse_command(self.string)
            
            msg.x = float(angles[0])
            msg.y = float(angles[1])
            msg.z = float(angles[2])
            msg.w = float(angles[3])

            self.publisher_.publish(msg)
            self.get_logger().info(f'Published joint angles for scenario {self.current_scenario}: {msg}')
            self.string = ''


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of the JointAnglePublisher node
    node = JointAnglePublisher()

    # Keep the node running
    rclpy.spin(node)

    # Clean up and shutdown the node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
