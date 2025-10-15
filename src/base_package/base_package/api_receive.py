#!/usr/bin/env python3

import rclpy
import time
import threading
import asyncio
import json
import websockets
from rclpy.node import Node
from swl_base_interfaces.srv import AppRequest
from swl_shared_interfaces.msg import Waypoint
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future

class APIReceiveNode(Node):
    def __init__(self):
        super().__init__("api_receive")

        self.callback_group = ReentrantCallbackGroup()
        self.app_request_client = self.create_client(AppRequest, "/cloud/app_request", callback_group=self.callback_group)

        self.websocket_server = None

        # Validate the command type
        self.valid_commands = [
            'abort_mission', 'start_mission', 'pan',
            'manual_mode', 'reroute', 'return_to_base', 'reroute_mission',
            'manual_open_hatch', 'manual_close_hatch', 'manual_centre', 'manual_uncentre', 'manual_enable_charge', 'manual_disable_charge',
            'set_system_mode'
        ]

        self.get_logger().info("API Receive node started")

    def call_api_request(self, request):
        
        def call_api_callback(future: Future):
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'Base State Machine received API action request: {response.success}')
                else:
                    self.get_logger().error(f'API service call failed. Action not received by Base State Machine')
            except Exception as e:
                self.get_logger().error(f'Service call failed: {str(e)}')
                #TODO add handle action request failure
            
        future = self.app_request_client.call_async(request)
        future.add_done_callback(call_api_callback)

    async def start(self):
        self.websocket_server = await websockets.serve(
            self.handle_websocket_message,
            "0.0.0.0", 
            8765
        )
        self.get_logger().info("WebSocket server started on ws://0.0.0.0:8765 (accessible from network)")

    async def handle_websocket_message(self, websocket):
        """Handle incoming WebSocket connections and messages"""
        self.get_logger().info(f"New WebSocket connection from {websocket.remote_address}")
        
        last_ping = time.time()

        try:

            async def ping_client():
                while True:
                    try:
                        pong_waiter = await websocket.ping()
                        await asyncio.wait_for(pong_waiter, timeout=10)
                        self.get_logger().debug(f"Ping successful to {websocket.remote_address}")
                        await asyncio.sleep(5)
                    except asyncio.TimeoutError:
                        self.get_logger().warning(f"Ping timeout to {websocket.remote_address}")
                        break
                    except Exception as e:
                        self.get_logger().error(f"Ping error: {str(e)}")
                        break

            ping_task = asyncio.create_task(ping_client())

            async for message in websocket:
                last_message_time = time.time()
                self.get_logger().info(f"Received WebSocket message at {last_message_time}")
                
                try:
                    # Parse the JSON message
                    data = json.loads(message)

                    # Extract and validate command_type
                    command_type = data.get('command_type', '')

                    # Validate command type
                    if not command_type:
                        error_msg = {"error": "Missing command_type field"}
                        await websocket.send(json.dumps(error_msg))
                        self.get_logger().error("Received message without command_type")
                        continue

                    if command_type not in self.valid_commands:
                        error_msg = {"error": f"Unknown command_type: {command_type}"}
                        await websocket.send(json.dumps(error_msg))
                        self.get_logger().error(f"Received unknown command_type: {command_type}")
                        continue
                
                    # Create AppRequest message
                    request = AppRequest.Request()
                    request.command_type = command_type
                    request.mission_id = data.get('mission_id', '')
                    request.target_mode = data.get('target_mode', '')
                    
                    # Handle waypoints if present
                    waypoints_data = data.get('waypoints', [])
                    request.waypoints = []
                    
                    for wp_data in waypoints_data:
                        waypoint = Waypoint()
                        waypoint.latitude = wp_data.get('latitude', 0.0)
                        waypoint.longitude = wp_data.get('longitude', 0.0)
                        waypoint.altitude = float(wp_data.get('altitude', 0.0))
                        request.waypoints.append(waypoint)

                    request.yaw_cw = float(data.get('yaw_cw', 0.0)) 
                    
                    # Make the service call
                    self.get_logger().info(f"Making service call with command: {request.command_type}")
                    self.call_api_request(request)
                    
                    # Send acknowledgment back to WebSocket client
                    response_msg = {
                        "status": "received",
                        "command_type": request.command_type,
                        "mission_id": request.mission_id,
                        "waypoint_count": len(request.waypoints),
                        "yaw_cw": request.yaw_cw
                    }
                    await websocket.send(json.dumps(response_msg))
                    
                except json.JSONDecodeError:
                    error_msg = {"error": "Invalid JSON format"}
                    await websocket.send(json.dumps(error_msg))
                    self.get_logger().error("Received invalid JSON from WebSocket")
                    
                except Exception as e:
                    error_msg = {"error": f"Processing error: {str(e)}"}
                    await websocket.send(json.dumps(error_msg))
                    self.get_logger().error(f"Error processing WebSocket message: {str(e)}")
                    
        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info(f"WebSocket connection closed for {websocket.remote_address}")
        except Exception as e:
            self.get_logger().error(f"WebSocket error: {str(e)}")
        except websockets.exceptions.ConnectionClosed as e:
            self.get_logger().warning(f"WebSocket connection closed: {e.code} - {e.reason}")
        finally:
            ping_task.cancel()

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
    api_node = APIReceiveNode()
    
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
