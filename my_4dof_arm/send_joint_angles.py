import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from kinematics import Kinematics
import numpy as np

class JointAngleSender(Node):
    def __init__(self):
        super().__init__('joint_angle_sender')
        
        # Create a publisher to the "joint_states" topic
        self.publisher_ = self.create_publisher(Quaternion, 'joint_states', 10)
        
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
        msg = Quaternion()     
        angles = self.kinematics.inverse_kinematics([1, 1, 1, 1, 1, 1])
        msg.x = float(angles[0])
        msg.y = float(angles[1])
        msg.z = float(angles[2])
        msg.w = float(angles[3])

        # Publish the message
        self.publisher_.publish(msg)
        
        # Log the joint angles to the console
        self.get_logger().info(f'Publishing joint angles: {msg}')


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
