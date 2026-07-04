import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
import math

class SimpleRelocalizationPublisher(Node):

    def __init__(self):
        super().__init__('simple_relocalization_publisher')
        
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Publisher for the robot's calculated (X, Y) position in the room
        self.publisher_ = self.create_publisher(Point, '/relocalize', 10)
        
        # --- KNOWN ROOM DIMENSIONS FROM BEFORE (in meters) ---
        self.ROOM_WIDTH = 8.0   # Total East-to-West distance
        self.ROOM_LENGTH = 2.5  # Total North-to-South distance
        self.TOLERANCE = 0.20   # Max allowable deviation (15cm) due to noise/obstacles

        self.get_logger().info('Lidar Localization Node has started.')

    def get_index_for_angle(self, target_angle, msg):
        """Calculates array index for a target angle in radians."""
        if target_angle < msg.angle_min:
            target_angle += 2 * math.pi
        if target_angle > msg.angle_max:
            target_angle -= 2 * math.pi
        index = int((target_angle - msg.angle_min) / msg.angle_increment)
        return max(0, min(index, len(msg.ranges) - 1))

    def scan_callback(self, msg):
        # 1. Find indices for the 4 cardinal directions
        idx_north = self.get_index_for_angle(0.0, msg)          # Front
        idx_west  = self.get_index_for_angle(math.pi / 2.0, msg) # Left
        idx_south = self.get_index_for_angle(math.pi, msg)       # Back
        idx_east  = self.get_index_for_angle(-math.pi / 2.0, msg)# Right

        # 2. Extract raw distances
        d_north = msg.ranges[idx_north]
        d_west  = msg.ranges[idx_west]
        d_south = msg.ranges[idx_south]
        d_east  = msg.ranges[idx_east]

        # 3. Establish cross-checks using your known room dimensions
        # Validate X position (Length: South wall to North wall)
        x_calculated = d_south
        total_length_measured = d_north + d_south
        
        # Validate Y position (Width: East wall to West wall)
        y_calculated = d_east
        total_width_measured = d_west + d_east

        # 4. Filter out readings if an obstacle blocks a wall
        is_x_valid = abs(total_length_measured - self.ROOM_LENGTH) <= self.TOLERANCE
        is_y_valid = abs(total_width_measured - self.ROOM_WIDTH) <= self.TOLERANCE

        if is_x_valid and is_y_valid:
            # Publish position relative to the back-right corner (0,0)
            pos_msg = Point()
            pos_msg.x = float(x_calculated)
            pos_msg.y = float(y_calculated)
            pos_msg.z = 0.0
            
            self.publisher_.publish(pos_msg)
            self.get_logger().info(f"Robot Position -> X: {pos_msg.x:.2f}m, Y: {pos_msg.y:.2f}m")
        else:
            self.get_logger().warn(
                f"Localization unreliable. Measured room: {total_length_measured:.2f}x{total_width_measured:.2f}m. "
                f"Expected: {self.ROOM_LENGTH}x{self.ROOM_WIDTH}m."
            )

def main(args=None):
    rclpy.init(args=args)
    node = SimpleRelocalizationPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

