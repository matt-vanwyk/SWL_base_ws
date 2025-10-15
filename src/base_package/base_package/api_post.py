#!/usr/bin/env python3

import rclpy
import time
import asyncio
import json
import websockets
import threading
from rclpy.node import Node
from swl_shared_interfaces.msg import DroneState, BaseState
from std_msgs.msg import String

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

        self.base_state_subscriber = self.create_subscription(
            BaseState,
            'base/state',
            self.base_state_callback,
            10
        )

        self.hardware_state_subscriber = self.create_subscription(
            String,
            '/arduino/hardware_state',
            self.hardware_state_callback,
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
        self.asyncio_loop = None

        self.current_base_state = "Unknown"
        self.current_system_mode = "manual"
        self.current_hardware_state = "000"
        self.doors_open = False
        self.arms_centered = False
        self.charger_on = False

        # WebSocket server
        self.websocket_server = None

        self.get_logger().info('API Post node started')
        self.get_logger().info(f'Base coordinates: {self.base_coordinates["latitude"]:.6f}, {self.base_coordinates["longitude"]:.6f}')
                
    def drone_state_callback(self, msg):
        """Monitor drone state changes and stream telemetry"""
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

        # Stream telemetry data to all connected clients
        if self.asyncio_loop and not self.asyncio_loop.is_closed():
            asyncio.run_coroutine_threadsafe(
                self.broadcast_telemetry_to_all_clients(msg), 
                self.asyncio_loop
            )

        self.last_drone_state = msg

    def base_state_callback(self, msg):
        """Receive base state machine updates"""
        self.current_base_state = msg.current_state
        self.get_logger().debug(f'Base state updated: {self.current_base_state}')

    def hardware_state_callback(self, msg):
        """Receive hardware state updates from Arduino node"""
        try:
            # Parse hardware state message: "mode:maintenance,state:101"
            parts = msg.data.split(',')
            for part in parts:
                key, value = part.split(':')
                if key == 'mode':
                    self.current_system_mode = value
                elif key == 'state':
                    self.current_hardware_state = value
                    # Parse individual components from bit mask
                    state_int = int(value, 2) if len(value) == 3 and all(c in '01' for c in value) else int(value)
                    self.doors_open = bool(state_int & 0b100)
                    self.arms_centered = bool(state_int & 0b010) 
                    self.charger_on = bool(state_int & 0b001)
                    
            self.get_logger().debug(f'Hardware state updated: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Failed to parse hardware state: {str(e)}')

    async def broadcast_telemetry_to_all_clients(self, drone_state_msg):
        """Stream telemetry data to all connected clients"""
        if not self.connected_clients:
            return
            
        # Convert ROS message to dictionary
        telemetry_data = {
            "status": "telemetry",
            "telemetry": {
                "latitude": drone_state_msg.latitude,
                "longitude": drone_state_msg.longitude,
                "altitude": drone_state_msg.altitude,
                "armed": drone_state_msg.armed,
                "flight_mode": drone_state_msg.flight_mode,
                "is_in_air": drone_state_msg.is_in_air,
                "battery_percentage": drone_state_msg.battery_percentage,
                "num_satellites": drone_state_msg.num_satellites,
                "landed_state": drone_state_msg.landed_state,
                "velocity_x": drone_state_msg.velocity_x,
                "velocity_y": drone_state_msg.velocity_y,
                "velocity_z": drone_state_msg.velocity_z,
                "drone_id": drone_state_msg.drone_id,
                "current_yaw": drone_state_msg.current_yaw,
                "mission_complete": drone_state_msg.mission_complete,
                "current_state": drone_state_msg.current_state
            },
            "base_station": {
                "system_mode": self.current_system_mode,
                "base_state": self.current_base_state,
                "hardware_state": self.current_hardware_state,
                "hardware_components": {
                    "doors_open": self.doors_open,
                    "arms_centered": self.arms_centered,
                    "charger_on": self.charger_on
                }
            },
            "timestamp": self.get_clock().now().to_msg().sec
        }
        
        clients_to_remove = set()
        
        for websocket in self.connected_clients.copy():
            try:
                await websocket.send(json.dumps(telemetry_data))
            except websockets.exceptions.ConnectionClosed:
                clients_to_remove.add(websocket)
                self.get_logger().debug(f'Removed disconnected client {websocket.remote_address}')
            except Exception as e:
                self.get_logger().error(f'Failed to send telemetry to client: {str(e)}')
                clients_to_remove.add(websocket)
        
        # Clean up disconnected clients
        self.connected_clients -= clients_to_remove

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
        
        last_ping = time.time()

        try:
            # Add client to connected set
            self.connected_clients.add(websocket)
            
            # Send initial status
            if self.coordinates_sent:
                await self.send_coordinates_to_client(websocket)
            else:
                # Send waiting message
                waiting_msg = {
                    "status": "waiting",
                    "message": "Waiting for mission upload to complete..."
                }
                await websocket.send(json.dumps(waiting_msg))
            
            # Send current telemetry if available
            if self.last_drone_state:
                telemetry_data = {
                    "status": "telemetry",
                    "telemetry": {
                        "latitude": self.last_drone_state.latitude,
                        "longitude": self.last_drone_state.longitude,
                        "altitude": self.last_drone_state.altitude,
                        "armed": self.last_drone_state.armed,
                        "flight_mode": self.last_drone_state.flight_mode,
                        "is_in_air": self.last_drone_state.is_in_air,
                        "battery_percentage": self.last_drone_state.battery_percentage,
                        "num_satellites": self.last_drone_state.num_satellites,
                        "landed_state": self.last_drone_state.landed_state,
                        "velocity_x": self.last_drone_state.velocity_x,
                        "velocity_y": self.last_drone_state.velocity_y,
                        "velocity_z": self.last_drone_state.velocity_z,
                        "drone_id": self.last_drone_state.drone_id,
                        "current_yaw": self.last_drone_state.current_yaw,
                        "mission_complete": self.last_drone_state.mission_complete,
                        "current_state": self.last_drone_state.current_state
                    },
                    "base_station": {
                        "system_mode": self.current_system_mode,
                        "base_state": self.current_base_state,
                        "hardware_state": self.current_hardware_state,
                        "hardware_components": {
                            "doors_open": self.doors_open,
                            "arms_centered": self.arms_centered,
                            "charger_on": self.charger_on
                        }
                    },
                    "timestamp": self.get_clock().now().to_msg().sec
                }
                await websocket.send(json.dumps(telemetry_data))

            async def ping_client():
                while True:
                    try:
                        pong_waiter = await websocket.ping()
                        await asyncio.wait_for(pong_waiter, timeout=10)
                        self.get_logger().debug(f"Ping successful to {websocket.remote_address}")
                        await asyncio.sleep(5)  # Ping every 5 seconds
                    except asyncio.TimeoutError:
                        self.get_logger().warning(f"Ping timeout to {websocket.remote_address}")
                        break
                    except Exception as e:
                        self.get_logger().error(f"Ping error: {str(e)}")
                        break
        
            # Start ping task
            ping_task = asyncio.create_task(ping_client())
            
            # Keep connection alive
            async for message in websocket:
                # Handle any incoming messages if needed
                last_message_time = time.time()
                self.get_logger().debug(f"Received message from client at {last_message_time}")
                    
        except websockets.exceptions.ConnectionClosed as e:
            self.get_logger().warning(f"WebSocket connection closed: {e.code} - {e.reason}")
        except Exception as e:
            self.get_logger().error(f"WebSocket error: {str(e)}")
        finally:
            # Remove client from connected set
            self.connected_clients.discard(websocket)
            ping_task.cancel()

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