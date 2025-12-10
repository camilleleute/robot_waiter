import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Bool
from rclpy.qos import qos_profile_sensor_data
import time
import math

# robot fsm states: {IDLE, TURN, FORWARD, WAIT, BACK}
#   - IDLE: robot being driven manually by keyboard
#   - TURN: person detected, turning until person ~middle of camera
#   - FORWARD: moves forward until person 1 ft in front
#   - WAIT: wait 60 s for person to take order
#   - BACK: moves back for 3 sec

class waiter(Node):
    def __init__(self):
        super().__init__('waiter')

        self.get_logger().info('Initializing Node...')

        # Publisher for velocity
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/gobilda/cmd_vel', 10)

        # Subscriber for LIDAR data
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile_sensor_data
        )

		# Subscriber for keyboard teleop
		self.teleop_sub = self.create_subscription(TwistStamped, '/gobilda/cmd_vel', self.teleop_callback, 10)
		
		self.last_teleop = TwistStamped()

		self.autonomous = True

        # Initial state
        self.state = 'IDLE'
        
        # Timer for different movements
        self.back_s = 0
        self.turn_s = 0
        self.last_lidar_s = 0
        self.lidar_timeout = 1.0

        self.closest_front = float('inf')
        self.closest_back = float('inf')

        self.create_timer(0.1, self.control_callback)

    def lidar_callback(self, msg: LaserScan):
        self.get_logger().info('LiDAR callback...')
        self.last_lidar_s = time.time()

        n = len(msg.ranges)
                
        # Process front sector with angles
        front_vals = []
        front_weighted_sum = 0.0
        front_weight_total = 0.0
        
        # front sector (indices 0 to n//3 and 2*n//3 to n)
        for i in list(range(0, n//3)) + list(range(2*n//3, n)):
            r = msg.ranges[i]
            if r > 0.0 and r < float('inf') and not math.isnan(r):
                front_vals.append(r)
                if r < 3.0:
                    angle = msg.angle_min + i * msg.angle_increment
                    weight = 1.0 / (r * r + 0.1)
                    front_weighted_sum += math.sin(angle) * weight
                    front_weight_total += weight
        
        self.closest_front = min(front_vals) if front_vals else float('inf')
        
        self.front_obstacle_direction = front_weighted_sum / front_weight_total if front_weight_total > 0 else 0.0



        self.get_logger().info(f'FSM: state={self.state}; front={self.closest_front}; back={self.closest_back}')
        if self.state == 'forward':
            # obstacle detected less than 1 meter in front, move back
            if self.closest_front < 1:
                self.state = 'back'
                self.get_logger().info('Obstacle detected, moving back...')
                self.back_s = time.time()
            
        elif self.state == 'back':
            if time.time() - self.back_s > 3.0 or self.closest_back < 1:
                self.state = 'turn'
                self.get_logger().info('Backing up complete, turning...')
                self.turn_s = time.time()
            
        elif self.state == 'turn':
            if time.time() - self.turn_s > 3.0 or self.closest_front > 4.0:
                self.state = 'forward'
                self.get_logger().info('Turning complete, moving forward...')

        elif self.state == 'stop':
            self.state = 'forward'

    
    def control_callback(self):
        self.get_logger().info('Control callback')
		ts = TwistStamped()
        t = Twist()
		
		if not self.autonomous:
  			if self.state == 'turn':
				# turn towards the human until it is in the center of the camera
				if self.human_pos:
					offset = self.human_pos[0] - 320	# get distance from person to camera center
					if(abs(offset) > 50):
						t.angular.z = -0.1 * offset/320
					else:
						t.angular.z = 0.0
						self.state = 'forward'
						self.get_logger().info("Facing human, moving forward now")
				else:
					t.angular.z = 0.0
					self.state = 'idle'
					self.get_logger().info("No human detected, going idle")

        	# set velocities
        	elif self.state == 'forward':
				if self.closest_front < 0.3:
					t.linear.x = 0.0
					self.state = 'wait'
					self.wait_s = time.time()
					self.get_logger().info("Reached human, waiting")
				else:
            		t.linear.x = 0.2
            		t.angular.z = 0.0
					self.get_logger().info("Moving forward")
			elif self.state = 'wait':
				if time.time() = self.wait_s > 60:
					self.state = 'back'
					self.back_s = time.time()
					self.get_logger().info("Order has been picked up, moving back")
        
        	elif self.state == 'back':
				if time.time() - self.back_s > 3:
					self.state = 'idle'
					t.linear.x = 0.0
					t.angular.z = 0.0
					self.autonomous = True
				else:
            		t.linear.x = -0.2
            		t.angular.z = 0.0
			else:
            	t.linear.x = 0.0
            	t.angular.z = 0.0
			ts.twist = t
		else:
			ts.twist = self.last_teleop

        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'base_link'
        self.cmd_vel_pub.publish(ts)
        self.get_logger().info(f'Publishing vel (x{t.linear.x}, z={t.angular.z})...')

	def teleop_callback(self, msg):
		self.last_teleop = msg
	
def main(args=None):
    rclpy.init(args=args)
    node = BumpAndGo()

    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
