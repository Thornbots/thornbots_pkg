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
        self.TOLERANCE = 0.20   # Max allowable deviation (20cm) due to noise/obstacles
        
        # Window configuration to fallback on close points
        self.SEARCH_WINDOW_DEG = 5.0  # Look +/- 5 degrees around target direction

        self.get_logger().info('Lidar Localization Node has started.')

    def get_index_for_angle(self, target_angle, msg):
        """Calculates array index for a target angle in radians."""
        if target_angle < msg.angle_min:
            target_angle += 2 * math.pi
        if target_angle > msg.angle_max:
            target_angle -= 2 * math.pi
        index = int((target_angle - msg.angle_min) / msg.angle_increment)
        return max(0, min(index, len(msg.ranges) - 1))

    def get_furthest_valid_in_window(self, center_angle, msg):
        """Searches a window around center_angle and returns the maximum valid distance."""
        window_rad = math.radians(self.SEARCH_WINDOW_DEG)
        
        # Find index boundaries for the window
        idx_start = self.get_index_for_angle(center_angle - window_rad, msg)
        idx_end = self.get_index_for_angle(center_angle + window_rad, msg)
        
        # Safely sort the bounds to handle positive/negative index ordering
        start = min(idx_start, idx_end)
        end = max(idx_start, idx_end)

        valid_ranges = []
        for i in range(start, end + 1):
            r = msg.ranges[i]
            # Check if point is numeric, finite, and within sensor hardware bounds
            if not math.isinf(r) and not math.isnan(r) and msg.range_min <= r <= msg.range_max:
                valid_ranges.append(r)
        
        # Return furthest point if found (changed from min to max to prioritize the background wall),
        # otherwise return None to indicate a blind direction
        return max(valid_ranges) if valid_ranges else None

    def scan_callback(self, msg):
        # 1. Grab furthest points inside windows for all 4 cardinal directions
        d_north = self.get_furthest_valid_in_window(0.0, msg)           # Front
        d_west  = self.get_furthest_valid_in_window(math.pi / 2.0, msg)  # Left
        d_south = self.get_furthest_valid_in_window(math.pi, msg)        # Back
        d_east  = self.get_furthest_valid_in_window(-math.pi / 2.0, msg) # Right

        # 2. Safety check: Exit if a full 10-degree window completely missed data
        if None in [d_north, d_west, d_south, d_east]:
            self.get_logger().warn("Localization skipped. One or more cardinal directions are completely blind.")
            return

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
