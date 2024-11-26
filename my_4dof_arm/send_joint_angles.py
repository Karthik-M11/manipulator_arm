import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Float32MultiArray
from kinematics import Kinematics
import numpy as np

class JointAngleSender(Node):
    def __init__(self):
        super().__init__('joint_angle_sender')
        
        # Create a publisher to the "joint_states" topic
        self.publisher_ = self.create_publisher(Float32MultiArray, 'joint_states', 10)
        
        # Create a timer to publish joint angles every second (1 Hz)
        self.timer = self.create_timer(1.0, self.publish_joint_angles)

        # Create the kinematics object with the dh params
        self.kinematics = Kinematics(
            [0, 1, 1, 1], 
            [1, 0, 0, 0],
            [np.pi/2, 0, 0, 0]
        )

    def publish_joint_angles(self):
        # Create a JointState message
        msg = Float32MultiArray()

        # Set example joint names

        angles = self.kinematics.inverse_kinematics([1, 1, 1, 1, 1, 1])
        
        angles_64 = []
        for item in angles:
            angles_64.append(float(item))
        
        # Set example joint angles (in radians)
        msg.data = angles_64
        # Publish the message
        self.publisher_.publish(msg)
        
        # Log the joint angles to the console
        self.get_logger().info(f'Publishing joint angles: {msg.data}')


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create an instance of the JointAngleSender node
    node = JointAngleSender()
    
    # Keep the node running
    rclpy.spin(node)
    
    # Clean up and shutdown the node
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
