import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import websockets

class VariotImage(Node):
    def __init__(self):
        super().__init__('variotImage_pub')
        self.publisher_ = self.create_publisher(Image, 'variot/images', 10)
        self.timer_ = self.create_timer(0.5, self.img_callback)
        self.count_ = 0

    def img_callback(self):
        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "variot"
        message.height = 480
        message.width = 640
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = 640 * 3
        message.data = np.zeros(640 * 480 * 3, dtype=np.uint8).tobytes()
        self.publisher_.publish(message)
        self.get_logger().info(f"Publishing Image with Data: {message.header.frame_id}")

        path = "dwsl/variot/sample_data/sample_img.png"
        try:
            with open(path, 'rb') as f:
                image = cv2.imdecode(np.frombuffer(f.read(), np.uint8), cv2.IMREAD_COLOR)
        except FileNotFoundError:
            self.get_logger().error("File does not exist")
            return

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "variot"
        message.height = image.shape[0]
        message.width = image.shape[1]
        message.encoding = "rgb8"
        message.is_bigendian = 0
        message.step = message.width * 3
        message.data = image.tobytes()
        self.publisher_.publish(message)
        self.get_logger().info(f"Publishing Image with Data: {message.header.frame_id}")
        self.count_ += 1

        # Add websocket publisher here
        

def main(args=None):
    rclpy.init(args=args)
    node = VariotImage()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
