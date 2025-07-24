import sys
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.h1.loco.h1_loco_client import LocoClient

sport_client = None

class CmdVelSubscriber(Node):
    """
    ROS2 Node that subscribes to the /cmd_vel topic and controls
    the Unitree robot's movement based on received Twist messages.
    """
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10  # QoS
        )
        self.get_logger().info('Subscribing to /cmd_vel topic...')

    def listener_callback(self, msg):
        """
        Callback function for the /cmd_vel topic.
        Processes the received Twist message and sends movement commands to the robot.
        """
        global sport_client

        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z

        # Clip values for saftey and to match robot capabilities
        angular_z = max(-0.5, min(0.5, angular_z))
        linear_x = max(-0.2, min(0.2, linear_x))
        linear_y = max(-0.2, min(0.2, linear_y))
        self.get_logger().info(f'Received cmd_vel: Linear X={linear_x:.2f}, Linear_Y={linear_y:.2f}, Clipped Angular Z={angular_z:.2f}')

        if sport_client:
            # The LocoClient.Move method expects (x, y, yaw)
            # linear.x from cmd_vel maps to x (forward/backward)
            # linear.y (sideways) sidestepping
            # angular.z from cmd_vel maps to yaw (rotation around Z-axis)
            sport_client.Move(linear_x, linear_y, angular_z)
            print(f"linear_x: {linear_x}, linear_y: {linear_y}, angular_z: {angular_z}")
        else:
            self.get_logger().warn("Sport client not initialized. Cannot send move commands.")

if __name__ == "__main__":
    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    print("Initializing ROS2 client library...")
    rclpy.init(args=sys.argv)
    print("ROS2 client library initialized.")

    print("Initializing Unitree SDK Channel Factory...")
    ChannelFactoryInitialize()
    print("Unitree SDK Channel Factory initialized.")

    print("Initializing Unitree Sport Client...")
    sport_client = LocoClient()
    sport_client.Init()
    print("Unitree Sport Client initialized.")

    print("Creating ROS2 subscriber node...")
    cmd_vel_subscriber_node = CmdVelSubscriber()
    print("ROS2 subscriber node created. Spinning node...")

    try:
        # Spin the ROS2 node to process callbacks
        rclpy.spin(cmd_vel_subscriber_node)
        print("rclpy.spin() has exited.") # This line will only print if spin() returns for some reason
    except KeyboardInterrupt:
        print("KeyboardInterrupt detected. Exiting gracefully...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if sport_client:
            print("Stopping robot movement...")
            sport_client.Move(0.0, 0.0, 0.0)

        # Destroy the ROS2 node
        if 'cmd_vel_subscriber_node' in locals() and cmd_vel_subscriber_node:
            print("Destroying ROS2 node...")
            cmd_vel_subscriber_node.destroy_node()
        # Shutdown ROS2 client library
        print("Shutting down ROS2 client library...")
        rclpy.shutdown()
        print("ROS2 client library shut down. Exiting application.")
        sys.exit(0)