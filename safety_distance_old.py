import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


# This node watches two turtles and:
# 1. Calculates distance between them
# 2. Stops them if they get too close or go near the walls
class DistanceNode(Node):

    def __init__(self):
        super().__init__('distance_node')   # Name of the node

        # Subscribe to the position of both turtles
        self.create_subscription(Pose, '/turtle1/pose', self.turtle1_callback, 10)
        self.create_subscription(Pose, '/turtle2/pose', self.turtle2_callback, 10)

        # Publish the distance between turtles
        self.distance_publisher = self.create_publisher(Float32, '/turtles_distance', 10)

        # Publishers to control (stop) the turtles
        self.cmd_t1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.cmd_t2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        # Store latest position of each turtle
        self.pos_turtle1 = None
        self.pos_turtle2 = None

        # If distance < min_allowed_distance → stop both turtles
        self.min_allowed_distance = 1.0

        # -------------------- [NEW] Parameters for stable safety behavior --------------------
        # Wall safe zone: turtlesim area is roughly x,y in [0, 11]. We keep a margin to stop before boundaries.
        self.wall_margin = 1.0  # [NEW] keep the same idea...(safe zone 1..10)

        # Hysteresis to avoid "spam" and to avoid flipping SAFE/UNSAFE around the threshold
        self.dist_hysteresis = 0.20  # [NEW] distance must become > min_allowed_distance + hysteresis to clear alarm
        self.wall_hysteresis = 0.20  # [NEW] wall must become safely inside (margin +/- hysteresis) to clear alarm

        # State flags so messages do not remain "stuck" and prints are only on transitions
        self.too_close_active = False  # [NEW]
        self.t1_wall_active = False    # [NEW]
        self.t2_wall_active = False    # [NEW]
        # -------------------------------------------------------------------------------------

        self.get_logger().info("Distance node started! Watching turtle1 and turtle2...")
        self.get_logger().info("If they get too close or near walls → I will stop them!")

    # Called every time we get new position of turtle1
    def turtle1_callback(self, msg):
        self.pos_turtle1 = msg
        self.update_distance_and_safety()

    # Called every time we get new position of turtle2
    def turtle2_callback(self, msg):
        self.pos_turtle2 = msg
        self.update_distance_and_safety()

    # Main function: calculate distance and check safety
    def update_distance_and_safety(self):
        # Wait until we have position from both turtles
        if self.pos_turtle1 is None or self.pos_turtle2 is None:
            return

        # Calculate distance using Pythagoras
        dx = self.pos_turtle1.x - self.pos_turtle2.x
        dy = self.pos_turtle1.y - self.pos_turtle2.y
        distance = math.sqrt(dx * dx + dy * dy)

        # Publish the distance so others can see it
        dist_msg = Float32()
        dist_msg.data = float(distance)
        self.distance_publisher.publish(dist_msg)

        # Keep this print if you like; logger is less noisy than print
        self.get_logger().info(f"Distance between turtles: {distance:.3f} meters")

        # -------------------- [CHANGED] Distance safety with hysteresis + state --------------------
        # Trigger unsafe if distance < min_allowed_distance
        if distance < self.min_allowed_distance:
            if not self.too_close_active:  # [NEW] transition: SAFE -> TOO_CLOSE
                self.get_logger().warn("TOO CLOSE! Stopping both turtles!")
            self.too_close_active = True   # [NEW]
            self.stop_both_turtles()       # still enforce stop
        else:
            # Clear unsafe only if distance is clearly safe again (hysteresis)
            if self.too_close_active and distance > (self.min_allowed_distance + self.dist_hysteresis):  # [NEW]
                self.get_logger().info("Distance back to safe. Releasing TOO CLOSE state.")
                self.too_close_active = False
        # -------------------------------------------------------------------------------------------

        # -------------------- [CHANGED] Wall safety with hysteresis + state --------------------
        # turtle1 wall check
        near1 = self.is_near_wall(self.pos_turtle1)
        if near1:
            if not self.t1_wall_active:  # [NEW] transition
                self.get_logger().warn("turtle1 is near the wall! Stopping it.")
            self.t1_wall_active = True
            self.stop_turtle1()
        else:
            if self.t1_wall_active and self.is_clearly_away_from_wall(self.pos_turtle1):  # [NEW]
                self.get_logger().info("turtle1 is back to safe zone. Releasing WALL state.")
                self.t1_wall_active = False

        # turtle2 wall check
        near2 = self.is_near_wall(self.pos_turtle2)
        if near2:
            if not self.t2_wall_active:  # [NEW] transition
                self.get_logger().warn("turtle2 is near the wall! Stopping it.")
            self.t2_wall_active = True
            self.stop_turtle2()
        else:
            if self.t2_wall_active and self.is_clearly_away_from_wall(self.pos_turtle2):  # [NEW]
                self.get_logger().info("turtle2 is back to safe zone. Releasing WALL state.")
                self.t2_wall_active = False
        # --------------------------------------------------------------------------------------

    # Check if a turtle is too close to the border (unsafe zone)
    def is_near_wall(self, pose):
        # [CHANGED] Use wall_margin variable instead of hard-coded 1.0/10.0
        x = pose.x
        y = pose.y
        min_xy = 0.0 + self.wall_margin
        max_xy = 11.0 - self.wall_margin
        return (x < min_xy) or (x > max_xy) or (y < min_xy) or (y > max_xy)

    # [NEW] Clear condition with hysteresis (must be safely inside)
    def is_clearly_away_from_wall(self, pose):
        x = pose.x
        y = pose.y
        min_xy = 0.0 + self.wall_margin + self.wall_hysteresis
        max_xy = 11.0 - self.wall_margin - self.wall_hysteresis
        return (min_xy <= x <= max_xy) and (min_xy <= y <= max_xy)

    # Stop both turtles by sending zero velocity
    def stop_both_turtles(self):
        self.stop_turtle1()
        self.stop_turtle2()

    # Send zero speed to turtle1
    def stop_turtle1(self):
        stop_msg = Twist()  # All zeros = stop
        self.cmd_t1.publish(stop_msg)

    # Send zero speed to turtle2
    def stop_turtle2(self):
        stop_msg = Twist()
        self.cmd_t2.publish(stop_msg)


# Main function - start the node
def main():
    rclpy.init()
    node = DistanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
