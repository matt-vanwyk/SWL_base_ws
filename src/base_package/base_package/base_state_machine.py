#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from swl_base_interfaces.srv import AppRequest

class BaseStationStateMachine(Node):
    def __init__(self):
        super().__init__('base_state_machine')

        # Service server for App Request
        self.app_request_service = self.create_service(
            AppRequest,
            "/cloud/app_request",
            self.handle_app_request
        )

        self.get_logger().info('Base Station State Machine node started')

    def handle_app_request(self, request, response):
        """Handle incoming app request service calls"""

        self.get_logger().info('=== Received App Request ===')
        self.get_logger().info(f'Command Type: {request.command_type}')
        self.get_logger().info(f'Mission ID: {request.mission_id}')
        self.get_logger().info(f'Number of Waypoints: {len(request.waypoints)}')

        # Log waypoint details if present
        for i, waypoint in enumerate(request.waypoints):
            self.get_logger().info(f'Waypoint {i+1}: lat={waypoint.latitude}, lon={waypoint.longitude}, alt={waypoint.altitude}')
        
        # Validate the command type
        valid_commands = [
            'abort_mission', 'start_mission', 'turn_left', 'turn_right',
            'manual_mode', 'reroute', 'close_hatch', 'open_hatch',
            'close_centering', 'open_centering', 'start_mission_patrol', 'continue'
        ]
        
        if request.command_type in valid_commands:
            response.success = True
            self.get_logger().info(f'✅ Command "{request.command_type}" accepted')
        else:
            response.success = False
            self.get_logger().warn(f'❌ Unknown command "{request.command_type}"')
            self.get_logger().info(f'Valid commands: {valid_commands}')
        
        self.get_logger().info('=== End Request ===\n')
        
        return response

def main():
    rclpy.init()
    
    test_server = BaseStationStateMachine()
    
    try:
        rclpy.spin(test_server)
    except KeyboardInterrupt:
        test_server.get_logger().info('Service server shutting down...')
    finally:
        test_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()