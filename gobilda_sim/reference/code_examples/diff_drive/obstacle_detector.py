import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped

import tf2_ros
import tf2_geometry_msgs
from tf_transformations import translation_matrix, quaternion_matrix, quaternion_from_matrix
from tf2_ros import TransformException

import numpy as np

''' For this lab we are going to use the TF2 library provided by ROS
to create an obstable detector node. In the end we want the following transformation:

--- odom2object

The functions "transform_to_matrix" and "multiply_transforms" are provided
as utility functions so that we can multiply two geometry_msgs/msg/TransformStamped
data types.

You're code should have one call the "multiply_transforms" function. That is all that
is needed for this assignment. The idea is we calculate the laser2object transform and
then get the odom2laser transfrom from the /tf topic (Note: this is NOT done through)
subscriptions). Then we can multiply to get the following:
odom2object = odom2laser * laser2object (Note: that the laser parts of the transforms "cancel out")

Check the assignment notes for more information.
'''

class ObstacleDetectorImprovedNode(Node):

    def __init__(self):
        super().__init__('obstacle_detector_improved')

        # Task 1:
        self.scan_sub = self.create_subscription(
        )

        # We will use these variables to listen to the ROS2 transformation tree
        # and to broadcast our own transformation from Odom -> Object
        self.tf_buffer = tf2_ros.Buffer()
        # this object let's us listen to the /tf topic
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # this object let's us publish the transform to the /tf topic
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Arguments are:
        # Period in seconds (float) | callback (callable)
        self.timer = self.create_timer(1.0, self.process_scan)

        self.latest_laser = LaserScan()

    def transform_to_matrix(self, transform):
        # Convert a Transform message to a 4x4 transformation matrix
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])
        rotation = np.array(quaternion_matrix((
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ))[:3, :3])  # Extract the rotation part

        # Create the full transformation matrix
        matrix = np.eye(4)
        matrix[:3, :3] = rotation
        matrix[:3, 3] = translation
        return matrix

    def multiply_transforms(self, transform_a, transform_b):
        # Multiply two Transform messages and return the resulting Transform
        # Convert transforms to matrices
        matrix_a = self.transform_to_matrix(transform_a)
        matrix_b = self.transform_to_matrix(transform_b)

        # Perform matrix multiplication
        result_matrix = np.dot(matrix_a, matrix_b)

        # Extract translation and rotation from the result matrix
        result_transform = TransformStamped()
        result_transform.transform.translation.x = result_matrix[0, 3]
        result_transform.transform.translation.y = result_matrix[1, 3]
        result_transform.transform.translation.z = result_matrix[2, 3]
        
        # Convert the rotation part back to a quaternion
        result_rotation = quaternion_from_matrix(result_matrix)
        result_transform.transform.rotation.x = result_rotation[0]
        result_transform.transform.rotation.y = result_rotation[1]
        result_transform.transform.rotation.z = result_rotation[2]
        result_transform.transform.rotation.w = result_rotation[3]

        return result_transform

    # Task 2:
    def scan_callback(self, msg):
        # Copy the data from the laser scanner into member variable

    def process_scan(self,):
        # Do nothing if there is no data from the laser yet
        if not self.latest_laser.ranges:
            return

        # Task 3:
        # Access the laser scan that is aligned with x-axis of the robot
        # Put it in a variable called 'distance'

        # Check that the laser reading is not 'inf'
        if :

            laser2object_msg = TransformStamped()
            # What is the transformation from the Lidar --> Object?
            # Think in terms of 2D displacement

            # Task 4:
            laser2object_msg.header.frame_id = # Set the correct parent frame
            laser2object_msg.child_frame_id = # Set correct child frame

            # Task 5:
            try:
                # Lookup transform

            except TransformException as ex:
                self.get_logger().warn(f'Obstacle transform not found: {ex}')
                return

            # Its usually best not to work with the gemoetry_msgs message types directly
            # since they have limited functionality. In this example, I have provided a some
            # functions to work with the robot: 'transform_to_matrix', 'multiply_transforms'

            # Task 6:
            # What order should the two transformations be multiplied?

            odom2object_msg.header.frame_id = # Set correct parent frame
            odom2object_msg.child_frame_id = # Set correct child frame

            # Task 7:
            # Call the broadcaster

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorImprovedNode()

    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
