#!/usr/bin/env python3

import rclpy
import time
import threading
from rclpy.node import Node
from swl_base_interfaces.srv import BaseCommand, AppRequest
from swl_shared_interfaces.srv import DroneCommand
from swl_shared_interfaces.msg import DroneState
from statemachine import StateMachine, State
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future

class BaseStateMachine(StateMachine):
    
    # Define States
    Idle = State(initial=True)
    Home = State()
    Ready_For_Takeoff = State()

    # Define transitions
    station_homed = Idle.to(Home)
    prepare_for_takeoff = Home.to(Ready_For_Takeoff)

    # Methods for 'on entering' new states
    def on_enter_Idle(self):
        """Called when entering Idle state - start homing sequence"""
        self.model.get_logger().info("System boot... homing station")
        # Automatically start homing after a brief delay (handled in boot_sequence)
        
    def on_enter_Home(self):
        """Called when entering Home state - station is ready"""
        self.model.get_logger().info("Homing successful! Entered HOME state - Station homed and ready for missions")

    def on_enter_Ready_For_Takeoff(self):
        """Called when entering Ready_For_Takeoff state - upload mission to drone"""
        self.model.get_logger().info("Entered READY_FOR_TAKEOFF state - Station doors open, arms uncentred, charger off...uploading mission to drone")
        self.model.upload_mission_to_drone()

class BaseStationStateMachine(Node):
    def __init__(self):
        super().__init__('base_state_machine')
        self.state_machine = BaseStateMachine(model=self)

        self.callback_group = ReentrantCallbackGroup()

        # Service server for App Request
        self.app_request_service = self.create_service(
            AppRequest,
            "/cloud/app_request",
            self.handle_app_request
        )

        # Service client for Arduino Node
        self.station_command_client = self.create_client(
            BaseCommand, 
            '/base_station/command',
            callback_group=self.callback_group
        )

        # Service client for Drone State Machine
        self.drone_command_client = self.create_client(
            DroneCommand,
            'drone/command',
            callback_group=self.callback_group
        )

        self.drone_state_subscriber = self.create_subscription(
            DroneState,
            'drone/state',
            self.drone_state_callback,
            10
        )

        # Mission variables
        self.current_drone_state = None
        self.current_mission_id = None
        self.current_waypoints = []

        self.get_logger().info('Base Station State Machine node started')
        self.get_logger().info(f'Initial state: {self.state_machine.current_state.name}')
        
        # Wait for Arduino service to be available
        self.station_command_client.wait_for_service(timeout_sec=10.0)
        
        # Start boot sequence in a separate thread
        self.boot_thread = threading.Thread(target=self.boot_sequence, daemon=True)
        self.boot_thread.start()

    def boot_sequence(self):
        """Boot sequence - automatically start homing if in Idle state"""
        time.sleep(2)  # Wait for everything to initialize
        
        if self.state_machine.current_state.name == 'Idle':
            self.get_logger().info('Starting boot sequence - homing station...')
            self.initiate_station_homing()

    ########################################
    # DRONE STATE CALLBACK
    ########################################
    
    def drone_state_callback(self, msg):
        """Callback to receive and store current drone state"""
        self.current_drone_state = msg

    ########################################
    # ARDUINO SERVICE CLIENTS
    ########################################

    def initiate_station_homing(self):
        """Initiate homing process - called from Idle state or boot sequence"""
        def home_callback(future: Future):
            try:
                response = future.result()
                if response.success:
                    self.state_machine.station_homed()
                else:
                    self.get_logger().error(f'Homing failed! Arduino state: {response.state:03b}')
                    # TODO: Could add retry logic here or transition to error state
                    
            except Exception as e:
                self.get_logger().error(f'Home service call failed: {str(e)}')
        
        # Create and send the request
        request = BaseCommand.Request()
        request.command = 'home_station'
        
        future = self.station_command_client.call_async(request)
        future.add_done_callback(home_callback)

    def initiate_station_preparation_for_takeoff(self):
        """Prepare station for takeoff - called when transitioning to Ready_For_Takeoff"""
        def station_prepare_callback(future: Future):
            try:
                response = future.result()
                if response.success and response.state == 0b100:
                    self.get_logger().info('Base station hardware prepared (state 100) - triggering state transition')
                    # Trigger state machine transition (which will automatically call upload_mission_to_drone)
                    self.state_machine.prepare_for_takeoff()
                else:
                    self.get_logger().error(f'Failed to prepare base station hardware. Expected state 100, got: {response.state:03b}')
                    # TODO: Handle preparation failure - maybe retry or abort mission
                    
            except Exception as e:
                self.get_logger().error(f'Station preparation service call failed: {str(e)}')

        try:
            # Create Arduino command to prepare for takeoff
            station_request = BaseCommand.Request()
            station_request.command = 'prepare_for_takeoff'

            # Make the async service call
            future = self.station_command_client.call_async(station_request)
            future.add_done_callback(station_prepare_callback)

            self.get_logger().info('Station preparation request sent to Arduino...')
            
        except Exception as e:
            self.get_logger().error(f'Failed to create station preparation request: {str(e)}')

    ########################################
    # DRONE SERVICE CLIENTS
    ########################################
                
    def upload_mission_to_drone(self):
        """Upload mission to drone - called automatically when entering Ready_For_Takeoff state"""
        def mission_upload_callback(future: Future):
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'Mission {self.current_mission_id} successfully uploaded to drone!')
                    self.get_logger().info('Next: Ready for drone arming and launch...')
                    # TODO: Next state transition (when we add arm/takeoff states)
                else:
                    self.get_logger().error(f'Mission upload failed for mission {self.current_mission_id}')
                    # TODO: Handle mission upload failure - maybe close station and retry
                    
            except Exception as e:
                self.get_logger().error(f'Mission upload service call failed: {str(e)}')

        try:
            # Create drone command request
            drone_request = DroneCommand.Request()
            drone_request.command_type = 'upload_mission'
            drone_request.waypoints = self.current_waypoints
            drone_request.drone_id = self.current_drone_state.drone_id if self.current_drone_state else "UNKNOWN_DRONE"
            drone_request.base_state = self.state_machine.current_state.name
            drone_request.yaw_cw = False

            future = self.drone_command_client.call_async(drone_request)
            future.add_done_callback(mission_upload_callback)

            self.get_logger().info(f'Mission upload request sent to drone...')

        except Exception as e:
            self.get_logger().error(f'Failed to create mission upload request: {str(e)}')

    ########################################
    # API REQUEST HANDLERS
    ########################################

    def handle_app_request(self, request, response):
        """Handle incoming app request service calls"""
        self.get_logger().info(f'Received app request: {request.command_type}')
        
        if request.command_type == 'start_mission':
            response.success = self.handle_start_mission(request)
        else:
            response.success = True
            self.get_logger().info(f'Command {request.command_type} acknowledged (not yet implemented)')
        
        return response
    
    def handle_start_mission(self, request):
        """Validate mission and initiate mission sequence"""
        
        # Step 1: Validate base station state
        if self.state_machine.current_state.name != 'Home':
            self.get_logger().error(f'Cannot start mission - base station not in Home state. Current state: {self.state_machine.current_state.name}')
            return False
        
        # Step 2: Validate drone state
        if self.current_drone_state is None:
            self.get_logger().error('Cannot start mission - no drone state received. Is drone connected?')
            return False
        
        if self.current_drone_state.current_state != 'Ready to fly':
            self.get_logger().error(f'Cannot start mission - drone not in Ready_To_Fly state. Current drone state: {self.current_drone_state.current_state}')
            return False
        
        # Step 3: Safety checks
        if self.current_drone_state.battery_percentage < 90.0:
            self.get_logger().error(f'Cannot start mission - drone battery too low: {self.current_drone_state.battery_percentage:.1f}%')
            return False
        
        # Step 4: Validate mission data
        if not request.mission_id or len(request.mission_id.strip()) == 0:
            self.get_logger().error('Cannot start mission - missing or empty mission_id')
            return False
        
        if not request.waypoints or len(request.waypoints) == 0:
            self.get_logger().error('Cannot start mission - no waypoints provided')
            return False
        
        # Step 5: Store mission data
        self.current_mission_id = request.mission_id
        self.current_waypoints = request.waypoints

        # Step 6: Log mission details
        self.get_logger().info(f'All validations passed! Starting mission preparation for mission_id: {request.mission_id}')
        
        self.initiate_station_preparation_for_takeoff()
        
        return True

def main():
    rclpy.init()
    
    base_station = BaseStationStateMachine()
    
    try:
        rclpy.spin(base_station)
    except KeyboardInterrupt:
        base_station.get_logger().info('Base Station State Machine shutting down...')
    finally:
        base_station.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()