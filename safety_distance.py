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

        # If distance < 1.0 → stop both turtles
        self.min_allowed_distance = 1.0

        print("Distance node started! Watching turtle1 and turtle2...")
        print("If they get too close or near walls → I will stop them!\n")

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
        distance = math.sqrt(dx*dx + dy*dy)

        # Publish the distance so others can see it
        dist_msg = Float32()
        dist_msg.data = distance
        self.distance_publisher.publish(dist_msg)

        print(f"Distance between turtles: {distance:.3f} meters")

        # If turtles are too close → STOP both!
        if distance < self.min_allowed_distance:
            print("TOO CLOSE! Stopping both turtles!")
            self.stop_both_turtles()

        # Check if any turtle is near the edge of the window (0-11 range, but safe zone 1-10)
        if self.is_near_wall(self.pos_turtle1):
            print("turtle1 is near the wall! Stopping it.")
            self.stop_turtle1()

        if self.is_near_wall(self.pos_turtle2):
            print("turtle2 is near the wall! Stopping it.")
            self.stop_turtle2()

    # Check if a turtle is too close to the border
    def is_near_wall(self, pose):
        x = pose.x
        y = pose.y
        return x < 1.0 or x > 10.0 or y < 1.0 or y > 10.0

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
    rclpy.init()                    # Start ROS2
    node = DistanceNode()           # Create our node

    print("Distance node is running... Press Ctrl+C to stop.\n")

    try:
        rclpy.spin(node)            # Keep node alive and listening
    except KeyboardInterrupt:
        print("\nShutting down distance node...")

    # Clean shutdown
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()