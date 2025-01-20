import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from time import sleep

class RobotArmMover(Node):

    def __init__(self):
        super().__init__('robot_arm_mover')

        # Create publishers for the joint controllers
        self.joint1_pub = self.create_publisher(Float64, '/joint1_position_controller/command', 10)
        self.joint2_pub = self.create_publisher(Float64, '/joint2_position_controller/command', 10)

        # Set the movement parameters
        self.target_joint1_position = Float64()
        self.target_joint2_position = Float64()

    def move_joint1(self, position):
        self.target_joint1_position.data = position
        self.joint1_pub.publish(self.target_joint1_position)
        self.get_logger().info(f'Moving joint1 to position: {position}')

    def move_joint2(self, position):
        self.target_joint2_position.data = position
        self.joint2_pub.publish(self.target_joint2_position)
        self.get_logger().info(f'Moving joint2 to position: {position}')

def main(args=None):
    rclpy.init(args=args)
    robot_arm_mover = RobotArmMover()

    # Move joint1 and joint2 to specified positions
    robot_arm_mover.move_joint1(1.57)  # Move joint1 to 90 degrees (1.57 radians)
    robot_arm_mover.move_joint2(0.78)  # Move joint2 to 45 degrees (0.78 radians)

    # Sleep to allow movements to complete
    sleep(2)

    # Move joint1 and joint2 back to their initial positions
    robot_arm_mover.move_joint1(0.0)   # Move joint1 to 0 radians (initial position)
    robot_arm_mover.move_joint2(0.0)   # Move joint2 to 0 radians (initial position)

    # Sleep to allow movements to complete
    sleep(2)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
