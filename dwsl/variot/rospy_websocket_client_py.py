#!/usr/bin/env python3
import websocket
import rclpy
import cv2
import os
import ssl

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ConnectionWebsocket:
    '''
    This class implements a ROS node that communicates with a remote server through a WebSocket connection.
    The node subscribes to a ROS topic that publishes image frames and sends the frames to the remote server.    
    '''
    def __init__(self, ws_host: str, ws_port: str, ros_image_node: str):
        self.ws_host = ws_host
        self.ws_port = ws_port
        self.ros_image_node = ros_node
        self.frame = None
        self.bridge = CvBridge()

    def set_frame(self, frame_in):
        self.frame = frame_in

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        self.set_frame(cv_image)

    def listener(self):
        rclpy.Subscriber("rgb_frame", Image, self.callback)
        rclpy.spin()

    def get_frame(self):
        return self.frame

    def on_message(self, ws, msg):
        if msg == "start":
            frame = self.get_frame()
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 65]
            frame_enc = cv2.imencode('.jpg', frame)[1]
            ws.send(frame_enc.tobytes(), opcode=0x02)
            print("Sent Frame of shape: {}".format(frame.shape))

    def on_error(self, ws, error):
        ## report error
        print(error) 

    def on_close(self, ws):
        print("### closed ###")

    def on_open(self, ws):
        print("Connected to Server")
        ws.send("robot1")

    def run(self):
        rclpy.init()
        websocket.enableTrace(False)
        ws = websocket.WebSocketApp(f"ws://{self.ws_host}:{self.ws_port}", on_open=self.on_open,
                                    on_message=self.on_message, on_error=self.on_error, keep_running=True)
        result = ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})
        print(result)


if __name__ == "__main__":
    ws_host = "127.0.1.1"
    ws_port = "8000"
    ros_node = "variotImage_pub"

    connection_websocket = ConnectionWebsocket(ws_host, ws_port, ros_node)
    connection_websocket.run()
