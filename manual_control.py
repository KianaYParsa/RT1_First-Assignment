import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

# Our main node that controls two turtles
class TurtleUI(Node):

    def __init__(self):
        super().__init__('ui_node')                # Node name is "ui_node"
        
        # Create publishers for turtle1 and turtle2
        self.pub_t1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pub_t2 = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        
        self.get_logger().info("TurtleUI node is ready! You can control turtle1 or turtle2.")

    # This function moves one turtle for exactly 1 second then stops it
    def move_turtle_one_second(self, turtle_name, linear_speed, angular_speed):
        
        # Create the movement message
        move_msg = Twist()
        move_msg.linear.x = float(linear_speed)
        move_msg.angular.z = float(angular_speed)
        
        # Create a message to stop the turtle (all zeros)
        stop_msg = Twist()
        
        # Remember when we started
        start_time = time.time()
        
        print(f"Sending to {turtle_name} → linear: {linear_speed}, angular: {angular_speed}")
        
        # Keep publishing the command for 1 second
        while time.time() - start_time < 1.0:
            if turtle_name == "turtle1":
                self.pub_t1.publish(move_msg)
            elif turtle_name == "turtle2":
                self.pub_t2.publish(move_msg)
                
            # Let ROS2 process things (important!)
            rclpy.spin_once(self, timeout_sec=0.01)
        
        # After 1 second, stop the turtle
        print(f"Stopping {turtle_name}...")
        if turtle_name == "turtle1":
            self.pub_t1.publish(stop_msg)
        elif turtle_name == "turtle2":
            self.pub_t2.publish(stop_msg)
            
        rclpy.spin_once(self, timeout_sec=0.01)


def main():
    rclpy.init()                     # Start ROS2
    node = TurtleUI()                  # Create our node
    
    print("\nTurtle Controller")
    print("You can control turtle1 or turtle2")
    print("Type 'q' to quit\n")
    
    try:
        while True:
            # Ask user which turtle to control
            choice = input("Which turtle? (turtle1 / turtle2 / q to quit): ").strip().lower()
            
            if choice == 'q':
                print("FINISH")
                break
                
            if choice not in ['turtle1', 'turtle2']:
                print("Error: Please type 'turtle1' or 'turtle2'")
                continue
            
            # Get speeds from user
            try:
                linear = float(input("Linear velocity (example: 1.0): "))
                angular = float(input("Angular velocity (example: 2.0): "))
            except ValueError:
                print("Error: Please enter numbers only!")
                continue
            
            # Send the command for 1 second
            node.move_turtle_one_second(choice, linear, angular)
            
    except KeyboardInterrupt:
        print("\nStopped by user")
    
    # Clean shutdown
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()