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

        # Track state changes and connected clients
        self.last_drone_state = None
        self.coordinates_sent = False
        self.connected_clients = set()  # Track all connected WebSocket clients

        # WebSocket server
        self.websocket_server = None

        self.get_logger().info('API Post node started')
        self.get_logger().info(f'Base coordinates: {self.base_coordinates["latitude"]:.6f}, {self.base_coordinates["longitude"]:.6f}')
                
    def drone_state_callback(self, msg):
        """Monitor drone state changes"""
        previous_state = self.last_drone_state.current_state if self.last_drone_state else None
        current_state = msg.current_state

        # Detect transition TO Mission_Uploaded (capture ground position and send coordinates)
        if (previous_state != 'Mission uploaded' and 
            current_state == 'Mission uploaded' and 
            not self.coordinates_sent):
            
            # Update base coordinates with actual takeoff position (drone still on ground)
            self.base_coordinates = {
                "latitude": msg.latitude,
                "longitude": msg.longitude
            }
            
            self.get_logger().info(f'Mission uploaded - captured base coordinates: {msg.latitude:.6f}, {msg.longitude:.6f}')
            self.get_logger().info('Sending base coordinates to all connected clients...')
            
            # Use run_coroutine_threadsafe to bridge between ROS thread and asyncio loop
            if self.asyncio_loop and not self.asyncio_loop.is_closed():
                asyncio.run_coroutine_threadsafe(
                    self.broadcast_coordinates_to_all_clients(), 
                    self.asyncio_loop
                )
            else:
                self.get_logger().error('No asyncio loop available for broadcasting coordinates')
            
            self.coordinates_sent = True

        self.last_drone_state = msg

    async def broadcast_coordinates_to_all_clients(self):
        """Send coordinates to all currently connected clients"""
        if not self.connected_clients:
            self.get_logger().info('No clients connected to receive coordinates')
            return
            
        clients_to_remove = set()
        
        for websocket in self.connected_clients.copy():  # Copy to avoid modification during iteration
            try:
                await self.send_coordinates_to_client(websocket)
            except websockets.exceptions.ConnectionClosed:
                clients_to_remove.add(websocket)
                self.get_logger().info(f'Removed disconnected client {websocket.remote_address}')
            except Exception as e:
                self.get_logger().error(f'Failed to send coordinates to client: {str(e)}')
                clients_to_remove.add(websocket)
        
        # Clean up disconnected clients
        self.connected_clients -= clients_to_remove


    async def start(self):
        self.asyncio_loop = asyncio.get_event_loop()

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
            # Add client to connected set
            self.connected_clients.add(websocket)
            
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
            
            # Keep connection alive
            async for message in websocket:
                # Handle any incoming messages if needed
                self.get_logger().info(f"Received message from client: {message}")
                    
        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info(f"WebSocket connection closed for {websocket.remote_address}")
        except Exception as e:
            self.get_logger().error(f"WebSocket error: {str(e)}")
        finally:
            # Remove client from connected set
            self.connected_clients.discard(websocket)

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