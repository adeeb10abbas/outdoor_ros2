import asyncio
import websockets
import json

class WebsocketServer:
    """Websocket Server Class to handle incoming data from robot.

    This class creates a websocket server that listens to incoming data from
    the robot. It provides a mechanism to handle different types of data coming
    in from the robot.
    """

    def __init__(self, host, port):
        """Initialize the websocket server with host and port.

        :param host: The host IP address to bind the server.
        :type host: str
        :param port: The port number to bind the server.
        :type port: int
        """
        self.host = host
        self.port = port
        self.data_handlers = {}

    async def handle_connection(self, websocket, path):
        """Handles incoming data from a connected client.

        :param websocket: The connected websocket object.
        :type websocket: websockets.WebSocket
        :param path: The path of the connected websocket.
        :type path: str
        """
        async for message in websocket:
            message = json.loads(message)
            message_type = message.get('type')
            data = message.get('data')
            if message_type in self.data_handlers:
                await self.data_handlers[message_type](websocket, data)

    async def send_data(self, websocket, data):
        """Sends data to the connected client.

        :param websocket: The connected websocket object.
        :type websocket: websockets.WebSocket
        :param data: The data to be sent to the client.
        :type data: dict
        """
        await websocket.send(json.dumps(data))

    def register_handler(self, message_type, handler):
        """Registers a handler for a specific message type.

        :param message_type: The message type for which the handler is to be registered.
        :type message_type: str
        :param handler: The handler function to be registered.
        :type handler: function
        """
        self.data_handlers[message_type] = handler

    def start(self):
        """Starts the websocket server."""
        start_server = websockets.serve(
            self.handle_connection, self.host, self.port)
        asyncio.get_event_loop().run_until_complete(start_server)
        asyncio.get_event_loop().run_forever()


import requests

def route_handler(websocket, data):
    """Handler for route data received from the robot.

    :param websocket: The connected websocket object.
    :type websocket: websockets.Web
    :param data: The route data received from the robot.
    :type data: dict
    """
        
    # Send the route data to the client
    return websocket.send(json.dumps(data))

def main():
    """Main function to start the websocket server."""
    server = WebsocketServer('localhost', 8765)
    server.register_handler('image', image_handler)
    server.register_handler('route', route_handler)
    server.start()
    server.send_data('image', {'image': 'image_data'})
    server.send_data('state', {'state': 'state_data'})

if __name__ == '__main__':
    main()