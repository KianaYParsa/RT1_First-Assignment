#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


# This node watches two turtles and:
# 1. Calculates distance between them
# 2. Stops the moving turtle if they get too close or go near the walls
class DistanceNode(Node):

    def __init__(self):
        super().__init__('distance_node')   # Name of the node

        # Subscribe to the position of both turtles
        self.create_subscription(Pose, '/turtle1/pose', self.turtle1_callback, 10)
        self.create_subscription(Pose, '/turtle2/pose', self.turtle2_callback, 10)

        # NEW: Subscribe to cmd_vel of both turtles to know which one is "moving"
        self.create_subscription(Twist, '/turtle1/cmd_vel', self.turtle1_cmd_callback, 10)
        self.create_subscription(Twist, '/turtle2/cmd_vel', self.turtle2_cmd_callback, 10)

        # Publish the distance between turtles
        self.distance_publisher = self.create_publisher(Float32, '/turtles_distance', 10)

        # Publishers to control (stop) the turtles
        self.cmd_t1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.cmd_t2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        # Store latest position of each turtle  (same names as oldVer)
        self.pos_turtle1 = None
        self.pos_turtle2 = None

        # NEW: Store latest commanded velocity for each turtle
        self.last_cmd_turtle1 = Twist()
        self.last_cmd_turtle2 = Twist()

        # If distance < threshold → stop the moving turtle
        self.min_allowed_distance = 1.0

        # NEW: Flags for continuous safety enforcement
        self.stop_turtle1_active = False
        self.stop_turtle2_active = False

        # NEW: Timer to keep enforcing stop while unsafe (prevents UI overriding)
        self.safety_timer = self.create_timer(0.05, self.enforce_safety)  # 20 Hz

        print("Distance node started! Watching turtle1 and turtle2...")
        print("If they get too close or near walls → I will stop the moving turtle!\n")

    # Called every time we get new position of turtle1
    def turtle1_callback(self, msg):
        self.pos_turtle1 = msg
        self.update_distance_and_safety()

    # Called every time we get new position of turtle2
    def turtle2_callback(self, msg):
        self.pos_turtle2 = msg
        self.update_distance_and_safety()

    # NEW: track UI commands for turtle1
    def turtle1_cmd_callback(self, msg):
        self.last_cmd_turtle1 = msg

    # NEW: track UI commands for turtle2
    def turtle2_cmd_callback(self, msg):
        self.last_cmd_turtle2 = msg

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
        dist_msg.data = float(distance)
        self.distance_publisher.publish(dist_msg)

        print(f"Distance between turtles: {distance:.3f} meters")

        # Reset safety flags each update
        self.stop_turtle1_active = False
        self.stop_turtle2_active = False

        # If turtles are too close → STOP the moving turtle(s)
        if distance < self.min_allowed_distance:
            print("TOO CLOSE! Stopping the moving turtle(s)!")

            if self.is_moving(self.last_cmd_turtle1):
                self.stop_turtle1_active = True
            if self.is_moving(self.last_cmd_turtle2):
                self.stop_turtle2_active = True

            # Fallback: if movement cannot be detected, stop both (safe)
            if (not self.stop_turtle1_active) and (not self.stop_turtle2_active):
                self.stop_turtle1_active = True
                self.stop_turtle2_active = True

        # Check if any turtle is near the edge; stop it only if it is moving
        if self.is_near_wall(self.pos_turtle1) and self.is_moving(self.last_cmd_turtle1):
            print("turtle1 is near the wall and moving! Stopping it.")
            self.stop_turtle1_active = True

        if self.is_near_wall(self.pos_turtle2) and self.is_moving(self.last_cmd_turtle2):
            print("turtle2 is near the wall and moving! Stopping it.")
            self.stop_turtle2_active = True

        # Immediate stop for instant effect
        if self.stop_turtle1_active:
            self.stop_turtle1()
        if self.stop_turtle2_active:
            self.stop_turtle2()

    # NEW: Keep enforcing stop while unsafe conditions are active
    def enforce_safety(self):
        if self.stop_turtle1_active:
            self.stop_turtle1()
        if self.stop_turtle2_active:
            self.stop_turtle2()

    # Check if a turtle is too close to the border
    def is_near_wall(self, pose):
        x = pose.x
        y = pose.y
        return x < 1.0 or x > 10.0 or y < 1.0 or y > 10.0

    # NEW: detect if UI command is non-zero
    def is_moving(self, cmd):
        eps = 1e-3
        return abs(cmd.linear.x) > eps or abs(cmd.linear.y) > eps or abs(cmd.angular.z) > eps

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
