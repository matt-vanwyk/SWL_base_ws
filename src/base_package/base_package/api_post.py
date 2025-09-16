#!/usr/bin/env python3

import rclpy
import asyncio
import json
import websockets
import threading
from rclpy.node import Node
from swl_shared_interfaces.msg import DroneState

class APIPostNode(Node):
    def __init__(self):
        super().__init__('api_post')

        # Subscribe to drone state
        self.drone_state_subscriber = self.create_subscription(
            DroneState,
            'drone/state',
            self.drone_state_callback,
            10
        )

        # Base station coordinates (PX4 SITL default home)
        self.base_coordinates = {
            "latitude": 47.3977419,
            "longitude": 8.5455938
        }

        # Track state changes
        self.last_drone_state = None
        self.coordinates_sent = False

        # WebSocket server
        self.websocket_server = None

        self.get_logger().info('API Post node started')
        self.get_logger().info(f'Base coordinates: {self.base_coordinates["latitude"]:.6f}, {self.base_coordinates["longitude"]:.6f}')
                
    def drone_state_callback(self, msg):
        """Monitor drone state changes"""
        previous_state = self.last_drone_state.current_state if self.last_drone_state else None
        current_state = msg.current_state

        # Detect transition TO Mission_Uploaded (when mission upload succeeds)
        if (previous_state != 'Mission uploaded' and 
            current_state == 'Mission uploaded' and 
            not self.coordinates_sent):
            
            self.get_logger().info('Drone mission uploaded - sending base coordinates to any connected clients...')
            # We'll send coordinates to any currently connected clients
            # This will be handled in the WebSocket message handler
            self.coordinates_sent = True

        self.last_drone_state = msg

    async def start(self):
        self.websocket_server = await websockets.serve(
            self.handle_websocket_message,
            "0.0.0.0", 
            8766
        )
        self.get_logger().info("WebSocket server started on ws://0.0.0.0:8766 (accessible from network)")

    async def handle_websocket_message(self, websocket):
        """Handle incoming WebSocket connections and messages"""
        self.get_logger().info(f"New WebSocket connection from {websocket.remote_address}")
        
        try:
            # If coordinates already available, send immediately
            if self.coordinates_sent:
                await self.send_coordinates_to_client(websocket)
            else:
                # Send waiting message
                waiting_msg = {
                    "status": "waiting",
                    "message": "Waiting for mission upload to complete..."
                }
                await websocket.send(json.dumps(waiting_msg))
            
            # Keep connection alive and monitor for coordinate sending
            while True:
                try:
                    # Check if coordinates should be sent (non-blocking)
                    if self.coordinates_sent and not hasattr(websocket, 'coordinates_sent'):
                        await self.send_coordinates_to_client(websocket)
                        websocket.coordinates_sent = True  # Mark this socket as having received coordinates
                    
                    # Small delay to prevent tight loop
                    await asyncio.sleep(0.1)
                    
                except websockets.exceptions.ConnectionClosed:
                    break
                except Exception as e:
                    self.get_logger().error(f"Error in message loop: {str(e)}")
                    break
                    
        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info(f"WebSocket connection closed for {websocket.remote_address}")
        except Exception as e:
            self.get_logger().error(f"WebSocket error: {str(e)}")

    async def send_coordinates_to_client(self, websocket):
        """Send base coordinates to a specific client"""
        try:
            coordinates_msg = {
                "status": "coordinates",
                "base_coordinates": self.base_coordinates,
                "message": "Base coordinates received - ready for return_to_base command"
            }
            await websocket.send(json.dumps(coordinates_msg))
            self.get_logger().info(f"Base coordinates sent to {websocket.remote_address}")
        except Exception as e:
            self.get_logger().error(f"Failed to send coordinates to client: {str(e)}")

async def spin(node: Node):
    def _spin_func():
        while rclpy.ok():
            rclpy.spin_once(node)
        node.get_logger().info("ROS 2 spin thread finished")

    spin_thread = threading.Thread(target=_spin_func, daemon=True)
    spin_thread.start()
    try:
        while rclpy.ok():
            await asyncio.sleep(1.0)
    finally:
        node.get_logger().info("Waiting for ROS 2 spin thread to finish")
        spin_thread.join()

def main():
    rclpy.init()
    api_node = APIPostNode()
    
    loop = asyncio.get_event_loop()
    loop.run_until_complete(api_node.start())
      
    try:
        loop.run_until_complete(spin(api_node))
    except KeyboardInterrupt:
        pass
    finally:
        if api_node.websocket_server:
            loop.run_until_complete(api_node.websocket_server.close())
        api_node.destroy_node()
        rclpy.shutdown()
        loop.close()

if __name__ == '__main__':
    main()