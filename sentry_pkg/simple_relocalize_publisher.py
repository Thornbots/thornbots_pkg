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
        self.ROOM_WIDTH = 2.5   # Total East-to-West distance
        self.ROOM_LENGTH = 8.0  # Total North-to-South distance
        self.TOLERANCE = 0.20   # Max allowable deviation (20cm) due to noise/obstacles
        
        # Window configuration to fallback on close points
        self.SEARCH_WINDOW_DEG = 5.0  # Look +/- 5 degrees around target direction

        # --- LIDAR MOUNTING ANGLE OFFSET ---
        # 0.611 radians clockwise means a negative offset relative to the counter-clockwise ROS frame
        self.LIDAR_ROTATION_OFFSET = 0.96

        self.get_logger().info('Lidar Localization Node has started.')

    def get_index_for_angle(self, target_angle, msg):
        """Calculates array index for a target angle in radians, accounting for lidar offset."""
        # Shift the target angle by the mounting offset
        adjusted_angle = target_angle + self.LIDAR_ROTATION_OFFSET

        # Normalize the angle to keep it within the standard [-pi, pi] bounds
        adjusted_angle = (adjusted_angle + math.pi) % (2 * math.pi) - math.pi

        if adjusted_angle < msg.angle_min:
            adjusted_angle += 2 * math.pi
        if adjusted_angle > msg.angle_max:
            adjusted_angle -= 2 * math.pi

        index = int((adjusted_angle - msg.angle_min) / msg.angle_increment)
        return max(0, min(index, len(msg.ranges) - 1))

    def get_furthest_valid_in_window(self, center_angle, msg):
        """Searches a window around center_angle and returns the maximum valid distance."""
        window_rad = math.radians(self.SEARCH_WINDOW_DEG)
        
        # Find index boundaries for the window (offset is automatically applied inside get_index_for_angle)
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
        
        # Return furthest point if found, otherwise return None to indicate a blind direction
        return max(valid_ranges) if valid_ranges else None

    def scan_callback(self, msg):
        # 1. Grab furthest points inside windows for all 4 cardinal directions (shifted by offset)
        d_north = self.get_furthest_valid_in_window(0.0, msg)           # Front
        d_west  = self.get_furthest_valid_in_window(math.pi / 2.0, msg)  # Left
        d_south = self.get_furthest_valid_in_window(math.pi, msg)        # Back
        d_east  = self.get_furthest_valid_in_window(-math.pi / 2.0, msg) # Right

        # 2. Safety check: Exit if a full window completely missed data
        if None in [d_north, d_west, d_south, d_east]:
            self.get_logger().warn("Localization skipped. One or more cardinal directions are completely blind.")
            return

        # 3. Calculate room dimensions to verify clarity
        total_length_measured = d_north + d_south
        total_width_measured = d_west + d_east

        is_x_valid = abs(total_length_measured - self.ROOM_LENGTH) <= self.TOLERANCE
        is_y_valid = abs(total_width_measured - self.ROOM_WIDTH) <= self.TOLERANCE

        # Target value for selecting an optimal alternative wall when blocked
        TARGET_FALLBACK = 0.5

        # --- X POSITION DECISION LOGIC (Length: South wall to North wall) ---
        if is_x_valid:
            x_calculated = d_south
        else:
            # Check which reading is closer to the 0.5m target threshold
            diff_south = abs(d_south  )
            diff_north = abs(d_north  )
            
            if diff_south >= diff_north:
                x_calculated = d_south  # Trust South reading directly
                self.get_logger().debug("X Obstructed: Chose South wall (closer to 0.5m).")
            else:
                x_calculated = self.ROOM_LENGTH - d_north  # Derive X position using North reading
                self.get_logger().debug("X Obstructed: Chose North wall (closer to 0.5m).")

        # --- Y POSITION DECISION LOGIC (Width: East wall to West wall) ---
        if is_y_valid:
            y_calculated = d_east
        else:
            # Check which reading is closer to the 0.5m target threshold
            diff_east = abs(d_east - TARGET_FALLBACK)
            diff_west = abs(d_west - TARGET_FALLBACK)
            
            if diff_east <= diff_west:
                y_calculated = d_east  # Trust East reading directly
                self.get_logger().debug("Y Obstructed: Chose East wall (closer to 0.5m).")
            else:
                y_calculated = self.ROOM_WIDTH - d_west  # Derive Y position using West reading
                self.get_logger().debug("Y Obstructed: Chose West wall (closer to 0.5m).")

        # 4. Construct and publish position
        pos_msg = Point()
        pos_msg.x = 4.0 - float(x_calculated)
        pos_msg.y = 1.063 - float(y_calculated)
        pos_msg.z = 0.0
        
        self.publisher_.publish(pos_msg)

        # Log running states
        if not is_x_valid or not is_y_valid:
            self.get_logger().info(
                f"Obstructed Guess -> X: {pos_msg.x:.2f}m, Y: {pos_msg.y:.2f}m "
                f"(Measured: {total_length_measured:.2f}x{total_width_measured:.2f}m)"
            )
        else:
            self.get_logger().info(f"Confident Position -> X: {pos_msg.x:.2f}m, Y: {pos_msg.y:.2f}m")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleRelocalizationPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
