import websockets
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import mujoco
import pydrake.all

# Checks all the dependencies are working
print("Websockets version: {0}".format(websockets.__version__))
print(f"the opencv version is {cv2.__version__}")
print(f"the mujoco version is {mujoco.mj_versionString()}")
camera = mujoco.MjvCamera()
print(f"the numpy version is {np.__version__}")
print(f"Random operation with pydrake: {pydrake.math.abs(-500000)}")

#Taken from ROS tutorials as a proof of concept showing 
#that the python code can run in this bazel workspace with ROS 2
class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
