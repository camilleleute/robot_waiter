import rclpy
from rclpy.node import Node

from std_msgs.msg import String

# Object orientated nodes for ROS in python
class MinimalPublisher(Node):

    def __init__(self):
        # Init the node with a name (this is the name that appears when running)
        # 'ros2 node list'
        super().__init__('minimal_publisher')
        # Create a publisher object
        # Arguments include the type of message, the name of the topic,
        # and the queue size
        self.publisher_ = self.create_publisher(String, 'my_topic', 10)

        # Create a time class
        # This will make sure that the code runs at a specified
        # frequency
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    # Callback for the events
    def timer_callback(self):
        # Initialize the message to be sent
        msg = String()
        # 'std_msgs String' object has a member called 'data'
        msg.data = 'Hello World: %d' % self.i
        # Publish the message
        self.publisher_.publish(msg)
        # Get logger function call similar to 'cout' in C++
        # or 'print()' in python
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    # Spin function *important*
    # makes sure that the program does not terminate
    # immediately
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
