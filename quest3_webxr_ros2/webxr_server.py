#!/usr/bin/env python3
"""
Quest 3 WebXR ROS2 Server
WebXR server for Oculus Quest 3 that publishes TF transforms and topics
"""

import asyncio
import signal
import sys
import logging
import json
import ssl
import threading
import time
from datetime import datetime
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

# WebSocket imports
import websockets
from websockets.server import serve

# ROS2 imports
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
import tf2_geometry_msgs

# HTTP server imports
import http.server
import socketserver
from urllib.parse import urlparse
import os

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class WebXRHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
    """Custom HTTP handler that serves files from the webxr_app directory"""
    
    def __init__(self, *args, **kwargs):
        # Set the directory to serve from
        try:
            pkg_share_dir = get_package_share_directory('quest3_webxr_ros2')
            self.directory = os.path.join(pkg_share_dir, 'webxr_app')
        except:
            # Fallback to relative path for development
            self.directory = os.path.join(os.path.dirname(__file__), '..', 'webxr_app')
        
        super().__init__(*args, directory=self.directory, **kwargs)

class Quest3WebXRServer(Node):
    def __init__(self):
        super().__init__('quest3_webxr_server')
        
        # Server configuration
        self.host = '0.0.0.0'
        self.http_port = 8443
        self.ws_port = 8080
        self.wss_port = 8444
        self._clients = set()
        
        # TF2 broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Quest 3 controller state tracking
        self.controller_data = {
            'headset': {'position': None, 'rotation': None, 'timestamp': None},
            'left': {'position': None, 'rotation': None, 'buttons': {}, 'timestamp': None},
            'right': {'position': None, 'rotation': None, 'buttons': {}, 'timestamp': None}
        }
        
        # ROS2 publishers
        self.setup_ros2_publishers()
        
        logger.info("Quest 3 WebXR Server initialized")
    
    def setup_ros2_publishers(self):
        """Setup ROS2 publishers for Quest 3 data"""
        # Transform topics
        self.headset_transform_pub = self.create_publisher(
            TransformStamped, 
            '/quest3/headset/transform', 
            10
        )
        
        self.left_controller_transform_pub = self.create_publisher(
            TransformStamped, 
            '/quest3/left_controller/transform', 
            10
        )
        
        self.right_controller_transform_pub = self.create_publisher(
            TransformStamped, 
            '/quest3/right_controller/transform', 
            10
        )
        
        # Button topics (Joy messages)
        self.left_controller_buttons_pub = self.create_publisher(
            Joy, 
            '/quest3/left_controller/buttons', 
            10
        )
        
        self.right_controller_buttons_pub = self.create_publisher(
            Joy, 
            '/quest3/right_controller/buttons', 
            10
        )
        
        logger.info("ROS2 publishers initialized")
    
    async def register_client(self, websocket, path=None):
        """Register a new WebSocket client"""
        self._clients.add(websocket)
        client_addr = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        logger.info(f"New Quest 3 client connected: {client_addr}")
    
    async def unregister_client(self, websocket):
        """Unregister a WebSocket client"""
        self._clients.discard(websocket)
        client_addr = f"{websocket.remote_address[0]}:{websocket.remote_address[1]}"
        logger.info(f"Quest 3 client disconnected: {client_addr}")
    
    async def handle_message(self, websocket, message):
        """Handle incoming WebSocket messages"""
        try:
            data = json.loads(message)
            
            if data.get('type') == 'quest3_data':
                await self.process_quest3_data(data)
            elif data.get('type') == 'test':
                logger.info(f"Test message received: {data.get('message', 'No message')}")
            else:
                logger.warning(f"Unknown message type: {data.get('type')}")
                
        except json.JSONDecodeError as e:
            logger.error(f"Invalid JSON received: {e}")
        except Exception as e:
            logger.error(f"Error processing message: {e}")
    
    async def process_quest3_data(self, data):
        """Process Quest 3 data from WebXR"""
        try:
            timestamp = data.get('timestamp', time.time())
            
            # Debug: Log received data structure (commented out for performance)
            # logger.debug(f"Received Quest 3 data: {json.dumps(data, indent=2)}")
            
            # Process headset data
            if 'headset' in data:
                headset_data = data['headset']
                self.controller_data['headset'] = {
                    'position': headset_data.get('position', {}),
                    'rotation': headset_data.get('rotation', {}),
                    'timestamp': timestamp
                }
                await self.publish_headset_data()
            
            # Process controller data
            for controller in ['left', 'right']:
                if controller in data:
                    controller_data = data[controller]
                    buttons = controller_data.get('buttons', {})
                    
                    # Debug: Log button data (commented out for performance)
                    # if buttons:
                    #     logger.debug(f"{controller} controller buttons: {buttons}")
                    
                    self.controller_data[controller] = {
                        'position': controller_data.get('position', {}),
                        'rotation': controller_data.get('rotation', {}),
                        'buttons': buttons,
                        'timestamp': timestamp
                    }
                    await self.publish_controller_data(controller)
            
        except Exception as e:
            logger.error(f"Error processing Quest 3 data: {e}")
    
    async def publish_headset_data(self):
        """Publish headset transform data"""
        try:
            headset_data = self.controller_data['headset']
            if not headset_data['position'] or not headset_data['rotation']:
                return
            
            # Create TransformStamped message
            transform_msg = TransformStamped()
            transform_msg.header.stamp = self.get_clock().now().to_msg()
            transform_msg.header.frame_id = 'world'
            transform_msg.child_frame_id = 'quest3_headset'
            
            # Set translation
            pos = headset_data['position']
            transform_msg.transform.translation.x = pos.get('x', 0.0)
            transform_msg.transform.translation.y = pos.get('y', 0.0)
            transform_msg.transform.translation.z = pos.get('z', 0.0)
            
            # Set rotation
            rot = headset_data['rotation']
            transform_msg.transform.rotation.x = rot.get('x', 0.0)
            transform_msg.transform.rotation.y = rot.get('y', 0.0)
            transform_msg.transform.rotation.z = rot.get('z', 0.0)
            transform_msg.transform.rotation.w = rot.get('w', 1.0)
            
            # Publish to topic
            self.headset_transform_pub.publish(transform_msg)
            
            # Publish to TF
            self.tf_broadcaster.sendTransform(transform_msg)
            
            logger.debug(f"Published headset transform: pos({pos.get('x', 0):.3f}, {pos.get('y', 0):.3f}, {pos.get('z', 0):.3f})")
            
        except Exception as e:
            logger.error(f"Error publishing headset data: {e}")
    
    async def publish_controller_data(self, controller):
        """Publish controller transform and button data"""
        try:
            controller_data = self.controller_data[controller]
            if not controller_data['position'] or not controller_data['rotation']:
                return
            
            # Create TransformStamped message
            transform_msg = TransformStamped()
            transform_msg.header.stamp = self.get_clock().now().to_msg()
            transform_msg.header.frame_id = 'quest3_headset'  # Relative to headset
            transform_msg.child_frame_id = f'quest3_{controller}_controller'
            
            # Set translation
            pos = controller_data['position']
            transform_msg.transform.translation.x = pos.get('x', 0.0)
            transform_msg.transform.translation.y = pos.get('y', 0.0)
            transform_msg.transform.translation.z = pos.get('z', 0.0)
            
            # Set rotation
            rot = controller_data['rotation']
            transform_msg.transform.rotation.x = rot.get('x', 0.0)
            transform_msg.transform.rotation.y = rot.get('y', 0.0)
            transform_msg.transform.rotation.z = rot.get('z', 0.0)
            transform_msg.transform.rotation.w = rot.get('w', 1.0)
            
            # Publish transform to topic
            if controller == 'left':
                self.left_controller_transform_pub.publish(transform_msg)
            else:
                self.right_controller_transform_pub.publish(transform_msg)
            
            # Publish to TF
            self.tf_broadcaster.sendTransform(transform_msg)
            
            # Publish button data as Joy message
            await self.publish_controller_buttons(controller, controller_data['buttons'])
            
            logger.debug(f"Published {controller} controller transform: pos({pos.get('x', 0):.3f}, {pos.get('y', 0):.3f}, {pos.get('z', 0):.3f})")
            
        except Exception as e:
            logger.error(f"Error publishing {controller} controller data: {e}")
    
    async def publish_controller_buttons(self, controller, buttons):
        """Publish controller button data as Joy message"""
        try:
            # Debug: Log button publishing attempt (commented out for performance)
            # logger.debug(f"Publishing {controller} controller buttons: {buttons}")
            
            joy_msg = Joy()
            joy_msg.header.stamp = self.get_clock().now().to_msg()
            joy_msg.header.frame_id = f'quest3_{controller}_controller'
            
            # Quest 3 controller button mapping
            # buttons: [trigger, grip, menu, thumbstick_click, a/x, b/y, thumbstick_x, thumbstick_y]
            if controller == 'right':
                # Right controller: A and B buttons
                button4 = buttons.get('a', False)
                button5 = buttons.get('b', False)
            else:
                # Left controller: X and Y buttons
                button4 = buttons.get('x', False)
                button5 = buttons.get('y', False)
            
            button_values = [
                buttons.get('trigger', False),
                buttons.get('grip', False),
                buttons.get('menu', False),
                buttons.get('thumbstick', False),
                button4,
                button5
            ]
            
            # Convert boolean to int (0 or 1) - Joy message expects integers
            joy_msg.buttons = [1 if btn else 0 for btn in button_values]
            
            # Add thumbstick axes - ensure they are floats
            joy_msg.axes = [
                float(buttons.get('thumbstick_x', 0.0)),
                float(buttons.get('thumbstick_y', 0.0))
            ]
            
            # Publish button data
            if controller == 'left':
                self.left_controller_buttons_pub.publish(joy_msg)
            else:
                self.right_controller_buttons_pub.publish(joy_msg)
            
            # Log button presses
            pressed_buttons = [name for name, pressed in buttons.items() if pressed and name not in ['thumbstick_x', 'thumbstick_y']]
            if pressed_buttons:
                logger.info(f"{controller} controller buttons: {', '.join(pressed_buttons)}")
            
            # Log button values for debugging (commented out for performance)
            # logger.debug(f"{controller} controller button values: {button_values}")
            
        except Exception as e:
            logger.error(f"Error publishing {controller} controller buttons: {e}")
    
    async def handle_client(self, websocket, path=None):
        """Handle WebSocket client connection"""
        await self.register_client(websocket)
        try:
            async for message in websocket:
                await self.handle_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            await self.unregister_client(websocket)
    
    def start_http_server(self):
        """Start HTTPS server for WebXR app"""
        try:
            # Create SSL context
            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            try:
                pkg_share_dir = get_package_share_directory('quest3_webxr_ros2')
                cert_file = Path(pkg_share_dir) / 'webxr_app' / 'cert.pem'
                key_file = Path(pkg_share_dir) / 'webxr_app' / 'key.pem'
            except:
                # Fallback to relative path for development
                cert_file = Path(__file__).parent / '..' / 'webxr_app' / 'cert.pem'
                key_file = Path(__file__).parent / '..' / 'webxr_app' / 'key.pem'
            
            if cert_file.exists() and key_file.exists():
                ssl_context.load_cert_chain(str(cert_file), str(key_file))
                
                # Create HTTPS server
                httpd = socketserver.TCPServer((self.host, self.http_port), WebXRHTTPRequestHandler)
                httpd.socket = ssl_context.wrap_socket(httpd.socket, server_side=True)
                
                logger.info(f"HTTPS server started on https://{self.host}:{self.http_port}")
                
                # Start server in thread
                def run_server():
                    httpd.serve_forever()
                
                server_thread = threading.Thread(target=run_server, daemon=True)
                server_thread.start()
                
            else:
                logger.warning("SSL certificates not found, HTTPS server not started")
                
        except Exception as e:
            logger.error(f"Failed to start HTTPS server: {e}")
    
    async def start_websocket_servers(self):
        """Start WebSocket servers (WS and WSS)"""
        # Start regular WebSocket server
        ws_server = await serve(
            self.handle_client,
            self.host,
            self.ws_port,
            ping_interval=30,
            ping_timeout=20
        )
        logger.info(f"WebSocket server started on ws://{self.host}:{self.ws_port}")
        
        # Start secure WebSocket server
        try:
            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            try:
                pkg_share_dir = get_package_share_directory('quest3_webxr_ros2')
                cert_file = Path(pkg_share_dir) / 'webxr_app' / 'cert.pem'
                key_file = Path(pkg_share_dir) / 'webxr_app' / 'key.pem'
            except:
                # Fallback to relative path for development
                cert_file = Path(__file__).parent / '..' / 'webxr_app' / 'cert.pem'
                key_file = Path(__file__).parent / '..' / 'webxr_app' / 'key.pem'
            
            if cert_file.exists() and key_file.exists():
                ssl_context.load_cert_chain(str(cert_file), str(key_file))
                
                wss_server = await serve(
                    self.handle_client,
                    self.host,
                    self.wss_port,
                    ssl=ssl_context,
                    ping_interval=30,
                    ping_timeout=20
                )
                logger.info(f"Secure WebSocket server started on wss://{self.host}:{self.wss_port}")
            else:
                logger.warning("SSL certificates not found, WSS server not started")
                
        except Exception as e:
            logger.warning(f"Could not start WSS server: {e}")
        
        return ws_server
    
    async def run(self):
        """Main server loop"""
        logger.info("Starting Quest 3 WebXR ROS2 Server...")
        
        # Start HTTPS server
        self.start_http_server()
        
        # Start WebSocket servers
        ws_server = await self.start_websocket_servers()
        
        logger.info("All servers started successfully!")
        logger.info(f"WebXR App: https://{self.host}:{self.http_port}/quest3_webxr.html")
        logger.info(f"WebSocket: ws://{self.host}:{self.ws_port} or wss://{self.host}:{self.wss_port}")
        logger.info("Published topics:")
        logger.info("  /quest3/headset/transform")
        logger.info("  /quest3/left_controller/transform")
        logger.info("  /quest3/right_controller/transform")
        logger.info("  /quest3/left_controller/buttons")
        logger.info("  /quest3/right_controller/buttons")
        logger.info("Published TF frames:")
        logger.info("  quest3_headset")
        logger.info("  quest3_left_controller")
        logger.info("  quest3_right_controller")
        
        try:
            # Keep server running
            await asyncio.Future()  # Run forever
        except KeyboardInterrupt:
            logger.info("Shutdown requested")
        finally:
            ws_server.close()
            await ws_server.wait_closed()

async def async_main():
    """Async main function"""
    # Initialize ROS2
    rclpy.init()
    
    # Create and run server
    server = Quest3WebXRServer()
    
    try:
        await server.run()
    finally:
        # Cleanup
        server.destroy_node()
        rclpy.shutdown()

def main():
    """Main function for ROS2 entry point"""
    # Handle graceful shutdown
    def signal_handler(signum, frame):
        logger.info("Received shutdown signal")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Run the server
    asyncio.run(async_main())

if __name__ == '__main__':
    main()
