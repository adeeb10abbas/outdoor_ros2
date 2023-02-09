## Author: Adeeb Abbas
## License: MIT
## Description: This file contains the data handlers to be used by the websocket server.

import requests
import json
import websocket

class DataHandlers:
    def __init__(self, websocket, thingsboard_ip, thingsboard_port, device_access_token):
        """Initialize the DataHandlers class with the ThingsBoard server connection details.

        :param thingsboard_ip: IP address of the ThingsBoard server.
        :type thingsboard_ip: str
        :param thingsboard_port: Port number of the ThingsBoard server.
        :type thingsboard_port: str
        :param device_access_token: Access token for the device in ThingsBoard.
        :type device_access_token: str
        """
        self.thingsboard_ip = thingsboard_ip
        self.thingsboard_port = thingsboard_port
        self.device_access_token = device_access_token
        self.websocket = websocket
        self.url = 'http://{thingsboard_ip}:{thingsboard_port}/api/v1/{device_access_token}/telemetry'

    def state_space_handler(data):
        """
        Handler for state space data received from the robot.
        :param data: The state space data received from the robot.
        :type data: dict
        """
        url = 'http://{thingsboard_ip}:{thingsboard_port}/api/v1/{device_access_token}/telemetry'
        headers = {'Content-Type': 'application/json'}

        state_space_data = {
            "state_space": {
                "x": data["x"],
                "y": data["y"],
                "z": data["z"],
                "vx": data["vx"],
                "vy": data["vy"],
                "vz": data["vz"],
            }
        }

        try:
            response = requests.post(self.url, json=state_space_data, headers=headers)
            if response.status_code != 200:
                print(f'Error sending state space data to ThingsBoard: {response.text}')
        except requests.exceptions.RequestException as e:
            print(f'Error sending state space data to ThingsBoard: {e}')


    def image_handler(websocket: websocket.WebSocket, data: dict):
        """Handler for image data received from the robot.
        :param data: The image data received from the robot.
        :type data: dict
        """
        # Sending image data to ThingsBoard server
        # NOTE: The image data is sent as a base64 encoded string. 
        # We might have to decode it before sending it to ThingsBoard.

        headers = {'Content-Type': 'application/json'}
        image_data = {'image': data['image']}
        try:
            response = requests.post(self.url, headers=headers, json=image_data)
            if response.status_code == 200:
                print("Image data sent to ThingsBoard successfully.")
            else:
                print("Failed to send image data to ThingsBoard. Status code: {}".format(response.status_code))
        except requests.exceptions.RequestException as e:
            print("Failed to send image data to ThingsBoard. Error: {}".format(e))