#!/usr/bin/env python3

import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from swl_base_interfaces.srv import BaseCommand, AppRequest
from swl_shared_interfaces.srv import DroneCommand
from swl_shared_interfaces.msg import DroneState
from statemachine import StateMachine, State
from rclpy.task import Future

class BaseStateMachine(StateMachine):
    
    # Define States
    Idle = State(initial=True)
    Home = State()
    Ready_For_Takeoff = State()
    Mission_In_Progress = State()

    # Define transitions
    station_homed = Idle.to(Home)
    prepare_for_takeoff = Home.to(Ready_For_Takeoff)
    mission_started = Ready_For_Takeoff.to(Mission_In_Progress)

    # Methods for 'on entering' new states
    def on_enter_Idle(self):
        """Called when entering Idle state - start homing sequence"""
        self.model.get_logger().info("System boot... homing station")
        
    def on_enter_Home(self):
        """Called when entering Home state - station is ready"""
        self.model.get_logger().info("State change: IDLE -> HOME")

    def on_enter_Ready_For_Takeoff(self):
        """Called when entering Ready_For_Takeoff state - upload mission to drone"""
        self.model.get_logger().info("State change: HOME -> READY_FOR_TAKEOFF")
        self.model.upload_mission_to_drone()

    def on_enter_Mission_In_Progress(self):
        """Called when entering Mission_In_Progress state - close station doors"""
        self.model.get_logger().info("State change: READY_FOR_TAKEOFF -> MISSION_IN_PROGRESS")
        self.model.secure_station_for_mission()

class BaseStationStateMachine(Node):
    def __init__(self):
        super().__init__('base_state_machine')
        self.state_machine = BaseStateMachine(model=self)

        # Create callback groups for different operations
        self.service_callback_group = ReentrantCallbackGroup()
        self.client_callback_group = ReentrantCallbackGroup()
        self.subscription_callback_group = MutuallyExclusiveCallbackGroup()

        # Service server for App Request (uses reentrant for potential nested calls)
        self.app_request_service = self.create_service(
            AppRequest,
            "/cloud/app_request",
            self.handle_app_request,
            callback_group=self.service_callback_group
        )

        # Service client for Arduino Node (uses reentrant for nested calls)
        self.station_command_client = self.create_client(
            BaseCommand, 
            '/base_station/command',
            callback_group=self.client_callback_group
        )

        # Service client for Drone State Machine (uses reentrant for nested calls)
        self.drone_command_client = self.create_client(
            DroneCommand,
            'drone/command',
            callback_group=self.client_callback_group
        )

        # Drone state subscriber
        self.drone_state_subscriber = self.create_subscription(
            DroneState,
            'drone/state',
            self.drone_state_callback,
            10,
            callback_group=self.subscription_callback_group
        )

        # Mission variables
        self.current_drone_state = None
        self.current_mission_id = None
        self.current_waypoints = []

        self.get_logger().info('Base Station State Machine node started')
        self.get_logger().info(f'Initial state: {self.state_machine.current_state.name}')
        
        # Wait for services to be available
        self.get_logger().info('Waiting for Arduino service...')
        if not self.station_command_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Arduino service not available!')
        else:
            self.get_logger().info('Arduino service connected')

        self.get_logger().info('Waiting for Drone command service...')
        if not self.drone_command_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Drone command service not available!')
        else:
            self.get_logger().info('Drone command service connected')
        
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

        self.check_mission_progress_transition()

    def check_mission_progress_transition(self):
        """Check if we should transition based on drone state (could be used later when returning to the base station)"""
        
        if self.state_machine.current_state.name != 'Ready for takeoff':
            return
        
        if self.current_drone_state is None:
            return

        # Check the drone state and altitude (drone state changes to Mission in progress when higher than 8m)
        if (self.current_drone_state.current_state == 'Mission in progress' and self.current_drone_state.altitude >= 8.0):
            self.get_logger().info('Drone has taken off successfully... Closing station doors')
            self.state_machine.mission_started()

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
                    self.get_logger().info('Station prepared for takeoff!')
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

    def secure_station_for_mission(self):
        """Secure the station when mission starts - close doors and leave arms uncentred and charger off (state 000)"""
        def secure_station_callback(future: Future):
            try:
                response = future.result()
                if response.success and response.state == 0b000:
                    self.get_logger().info('Station secured - doors closed')
                else:
                    self.get_logger().error(f'Failed to secure station. Expected state 000, got: {response.state:03b}')
                    # TODO: Handle securing failure - maybe retry or alert

            except Exception as e:
                self.get_logger().error(f'Station secure service call failed: {str(e)}')

        try:
            station_request = BaseCommand.Request()
            station_request.command = 'secure_station'

            future = self.station_command_client.call_async(station_request)
            future.add_done_callback(secure_station_callback)

        except Exception as e:
            self.get_logger().error(f'Failed to create secure station request: {str(e)}')

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
            response.success = self.handle_start_mission_sync(request)
        elif request.command_type == 'pan':
            response.success = self.handle_pan(request)
        else:
            response.success = True
            self.get_logger().info(f'Command {request.command_type} acknowledged (not yet implemented)')
        
        return response
    
    def handle_start_mission_sync(self, request):
        """Validate mission and initiate mission sequence - synchronous to ensure proper response"""
        
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
        self.get_logger().info(f'Starting mission preparation for {request.mission_id}')
        
        # Step 7: Synchronously prepare station and wait for completion
        return self.initiate_station_preparation_sync()
    
    def initiate_station_preparation_sync(self):
        """Synchronously prepare station for takeoff and wait for completion"""
        
        success_event = threading.Event()
        preparation_success = [False]
        error_message = ['']

        def station_prepare_callback(future: Future):
            try:
                response = future.result()
                if response.success and response.state == 0b100:
                    self.get_logger().info('Base station hardware prepared for takeoff')
                    self.state_machine.prepare_for_takeoff()
                    preparation_success[0] = True
                else:
                    self.get_logger().error(f'Failed to prepare base station hardware. Expected state 100, got: {response.state:03b}')
                    error_message[0] = f'Hardware preparation failed - state: {response.state:03b}'
                    
            except Exception as e:
                self.get_logger().error(f'Station preparation service call failed: {str(e)}')
                error_message[0] = str(e)
            finally:
                success_event.set()

        try:
            # Create Arduino command to prepare for takeoff
            station_request = BaseCommand.Request()
            station_request.command = 'prepare_for_takeoff'

            # Make the async service call
            future = self.station_command_client.call_async(station_request)
            future.add_done_callback(station_prepare_callback)

            self.get_logger().info('Station preparation request sent to Arduino')
            
            # Wait for the async call to complete (with timeout)
            if success_event.wait(timeout=30.0):  # 30 second timeout
                if preparation_success[0]:
                    return True
                else:
                    self.get_logger().error(f'Station preparation failed: {error_message[0]}')
                    return False
            else:
                self.get_logger().error('Station preparation timed out!')
                if not future.done():
                    future.cancel()
                return False
            
        except Exception as e:
            self.get_logger().error(f'Failed to create station preparation request: {str(e)}')
            return False
        
    def handle_pan(self, request):
        """Validate drone state before panning and sending command to drone"""
        # Step 1: Validate drone state
        if self.current_drone_state is None:
            self.get_logger().error('No drone state - drone not connected?')
            return False
        
        #Step 2: Check drone state is Mission_In_Progress
        if self.current_drone_state.current_state != 'Mission in progress':
            self.get_logger().error(f'Drone must be in Mission in Progress state - current: {self.current_drone_state.current_state}')
            return False
        
        # Validate yaw_cw field
        if request.yaw_cw == 0.0:
            self.get_logger().error('Invalid yaw_cw value - must be non-zero')
            return False
        
        # Limit yaw to reasonable range
        if abs(request.yaw_cw) > 180.0:
            self.get_logger().error(f'Yaw value too large: {request.yaw_cw}° (max ±180°)')
            return False
        
        drone_request = DroneCommand.Request()
        drone_request.command_type = request.command_type  # 'pan'
        drone_request.drone_id = self.current_drone_state.drone_id
        drone_request.yaw_cw = request.yaw_cw # positive = right and negative = left

        def pan_callback(future):
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info('Pan command received at drone state machine')
                else:
                    self.get_logger().warn('Failed to send pan command to drone state machine')
            except Exception as e:
                self.get_logger().error(f'Pan service call to drone state machine failed: {str(e)}')
        
        future = self.drone_command_client.call_async(drone_request)
        future.add_done_callback(pan_callback)

        self.get_logger().info(f'Pan command sent to drone: {request.yaw_cw}°')
        return True

def main():
    rclpy.init()
    
    # Create the node
    base_station = BaseStationStateMachine()
    
    # Create MultiThreadedExecutor with multiple threads
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(base_station)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        base_station.get_logger().info('Base Station State Machine shutting down...')
    finally:
        executor.shutdown()
        base_station.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()