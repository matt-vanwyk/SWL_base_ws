#!/usr/bin/env python3

import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from std_msgs.msg import Header
from swl_base_interfaces.srv import BaseCommand, AppRequest
from swl_shared_interfaces.srv import DroneCommand
from swl_shared_interfaces.msg import DroneState, BaseState
from statemachine import StateMachine, State
from rclpy.task import Future

class BaseStateMachine(StateMachine):
    
    # Define States
    Idle = State(initial=True)
    Home = State()
    Ready_For_Takeoff = State()
    Mission_In_Progress = State()
    Prepared_For_Landing = State()
    Charging = State()

    # Define transitions
    station_homed = Idle.to(Home)
    prepare_for_takeoff = Home.to(Ready_For_Takeoff)
    mission_started = Ready_For_Takeoff.to(Mission_In_Progress)
    prepared_for_landing = Mission_In_Progress.to(Prepared_For_Landing)
    station_charging = Prepared_For_Landing.to(Charging)
    prepare_station_for_next_mission = Charging.to(Home)

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

    def on_enter_Prepared_For_Landing(self):
        """Called when entering Prepared_For_Landing state - open station doors"""
        self.model.get_logger().info("State change: MISSION_IN_PROGRESS -> PREPARED_FOR_LANDING")

    def on_enter_Charging(self):
        """Called when entering Charging state - close station doors, centre and charge"""
        self.model.get_logger().info("State change: PREPARED_FOR_LANDING -> CHARGING")

class BaseStationStateMachine(Node):
    def __init__(self):
        super().__init__('base_state_machine')
        self.state_machine = BaseStateMachine(model=self)

        # Create callback groups for different operations
        self.service_callback_group = ReentrantCallbackGroup()
        self.client_callback_group = ReentrantCallbackGroup()
        self.subscription_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()

        # Service server for AppRequest.srv sent from api receive node
        self.app_request_service = self.create_service(
            AppRequest,
            "/cloud/app_request",
            self.handle_app_request,
            callback_group=self.service_callback_group
        )

        # Service client for arduino node BaseCommand.srv
        self.station_command_client = self.create_client(
            BaseCommand, 
            '/base_station/command',
            callback_group=self.client_callback_group
        )

        # Service client for drone state machine DroneCommand.srv
        self.drone_command_client = self.create_client(
            DroneCommand,
            'drone/command',
            callback_group=self.client_callback_group
        )

        # Subscriber for drone state machine DroneState.msg
        self.drone_state_subscriber = self.create_subscription(
            DroneState,
            'drone/state',
            self.drone_state_callback,
            10,
            callback_group=self.subscription_callback_group
        )

        # Publisher for BaseState.msg (to drone state machine)
        self.base_state_publisher = self.create_publisher(
            BaseState,
            'base/state',
            10
        )

        # Timer for publishing BaseState.msg
        self.state_publish_timer = self.create_timer(
            1.0, 
            self.publish_base_state,
            callback_group=self.timer_callback_group
        )

        # Mission variables
        self.current_drone_state = None
        self.current_mission_id = None
        self.current_waypoints = []
        self.station_securing_in_progress = False 
        self.station_landing_prep_in_progress = False
        self.station_landed_prep_in_progress = False
        self.station_charging_complete_in_progress = False

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

# DRONESTATE.MSG CALLBACK FROM DRONE STATE MACHINE
    def drone_state_callback(self, msg):
        """Callback to receive and store current drone state"""
        # Update current_drone_state from DroneState.msg received from drone state machine
        self.current_drone_state = msg

        # Drive state transitions based on DroneState.msg updates
        self.check_mission_progress_transition()

# METHOD TO DRIVE TRANSITIONS IN BASE STATE MACHINE BASED OFF OF DRONE STATE RECEIVED
    def check_mission_progress_transition(self):
        """Check if we should transition based on drone state"""
        if self.current_drone_state is None:
            self.get_logger().warn('No drone state assigned. Is drone connected?')
            return

        # Handle takeoff transition: only when base is in 'Ready for takeoff'
        if self.state_machine.current_state.name == 'Ready for takeoff':
            if (self.current_drone_state.current_state == 'Mission in progress' and 
                self.current_drone_state.altitude >= 8.0 and 
                not self.station_securing_in_progress):
                self.get_logger().info('Drone has taken off successfully... Closing station doors')
                self.station_securing_in_progress = True
                self.secure_station_for_mission()

        # Handle landing preparation: check for Loiter state when base is in Mission in progress
        elif (self.state_machine.current_state.name == 'Mission in progress' and
            self.current_drone_state.current_state == 'Loiter' and
            not self.station_landing_prep_in_progress):
            self.get_logger().info('Drone has arrived near base station. Opening station doors')
            self.station_landing_prep_in_progress = True
            self.prepare_station_for_landing()

        # Handle landed preparation: check for Landed state when base is in Prepared for landing
        elif (self.state_machine.current_state.name == 'Prepared for landing' and
            self.current_drone_state.current_state == 'Landed' and
            self.current_drone_state.landed_state == "ON_GROUND" and 
            not self.station_landed_prep_in_progress):
            self.get_logger().info('Drone has landed on platform. Closing doors, centering arms and turning charger on')
            self.station_landed_prep_in_progress = True
            self.prepare_station_for_charging()

        # Handle charging completion: check for Ready_To_Fly state when base is Charging
        elif (self.state_machine.current_state.name == 'Charging' and
            self.current_drone_state.current_state == 'Ready to fly' and
            not self.station_charging_complete_in_progress):
            self.get_logger().info('Drone charging complete - preparing station for next mission')
            self.station_charging_complete_in_progress = True
            self.state_machine.prepare_station_for_next_mission()

# METHOD TO PUBLISH BASE STATE TO DRONE STATE MACHINE
    def publish_base_state(self):
        msg = BaseState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base"

        msg.current_state = self.state_machine.current_state.name
        self.base_state_publisher.publish(msg)

########################################
# START - BASIC SERVICE CLIENTS TO ARDUINO NODE
########################################

    # METHOD TO HOME STATION ON SYSTEM BOOT
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

    # METHOD TO CLOSE STATION DOORS ONCE DRONE IS IN MISSION_IN_PROGRESS STATE
    def secure_station_for_mission(self):
        """Secure the station when mission starts - close doors and leave arms uncentred and charger off (state 000)"""
        def secure_station_callback(future: Future):
            try:
                response = future.result()
                if response.success and response.state == 0b000:
                    self.get_logger().info('Station secured - doors closed')
                    self.state_machine.mission_started() # Transition to Mission_In_Progress state once station doors are closed
                else:
                    self.get_logger().error(f'Failed to secure station. Expected state 000, got: {response.state:03b}')
                    # TODO: Handle securing failure - maybe retry or alert
            except Exception as e:
                self.get_logger().error(f'Station secure service call failed: {str(e)}')
            finally:
                self.station_securing_in_progress = False

        try:
            station_request = BaseCommand.Request()
            station_request.command = 'secure_station'

            future = self.station_command_client.call_async(station_request)
            future.add_done_callback(secure_station_callback)

        except Exception as e:
            self.get_logger().error(f'Failed to create secure station request: {str(e)}')

    # METHOD TO OPEN STATION DOORS ONCE DRONE IS IN LOITER STATE
    def prepare_station_for_landing(self):
        """Prepare the station for drone landing - open doors and leave arms uncentred and charger off (state 100)"""

        success_event = threading.Event()
        preparation_success = [False]
        error_message = ['']

        def prepare_station_callback(future: Future):
            try:
                response = future.result()
                if response.success and response.state == 0b100:
                    self.get_logger().info('Station prepared for landing - doors open')
                    self.state_machine.prepared_for_landing()  # FIXED TYPO: was prepapred_for_landing
                    preparation_success[0] = True
                else:
                    self.get_logger().error(f'Failed to prepare station for drone landing. Expected state 100, got: {response.state:03b}')
                    error_message[0] = f'Station preparation failed - state: {response.state:03b}'
            except Exception as e:
                self.get_logger().error(f'Station prepare for landing service call failed: {str(e)}')
                error_message[0] = str(e)
            finally:
                success_event.set()

        try:
            station_request = BaseCommand.Request()
            station_request.command = 'prepare_for_landing'

            future = self.station_command_client.call_async(station_request)
            future.add_done_callback(prepare_station_callback)

            self.get_logger().info('Station preparation for landing request sent to Arduino - waiting for completion...')
            
            # Wait for the async call to complete (with timeout)
            if success_event.wait(timeout=30.0):
                if preparation_success[0]:
                    self.get_logger().info('Station landing preparation completed successfully!')
                else:
                    self.get_logger().error(f'Station landing preparation failed: {error_message[0]}')
            else:
                self.get_logger().error('Station landing preparation timed out!')
                if not future.done():
                    future.cancel()
            
        except Exception as e:
            self.get_logger().error(f'Failed to create station landing preparation request: {str(e)}')
        finally:
            # CRITICAL: Reset the flag whether success or failure
            self.station_landing_prep_in_progress = False

    def prepare_station_for_charging(self):
        """Prepare the station for drone landing - open doors and leave arms uncentred and charger off (state 100)"""

        success_event = threading.Event()
        preparation_success = [False]
        error_message = ['']

        def prepare_station_for_charging_callback(future: Future):
            try:
                response = future.result()
                if response.success and response.state == 0b011:
                    self.get_logger().info('Station is charging with doors closed and arms centred')
                    self.state_machine.station_charging()  
                    preparation_success[0] = True
                else:
                    self.get_logger().error(f'Failed to start charging sequence. Expected state 011, got: {response.state:03b}')
                    error_message[0] = f'Station preparation failed - state: {response.state:03b}'
            except Exception as e:
                self.get_logger().error(f'Station start charging service call failed: {str(e)}')
                error_message[0] = str(e)
            finally:
                success_event.set()

        try:
            station_request = BaseCommand.Request()
            station_request.command = 'start_charging'

            future = self.station_command_client.call_async(station_request)
            future.add_done_callback(prepare_station_for_charging_callback)

            self.get_logger().info('Station start charging request sent to Arduino - waiting for completion...')
            
            # Wait for the async call to complete (with timeout)
            if success_event.wait(timeout=30.0):
                if preparation_success[0]:
                    self.get_logger().info('Station started charging successfully!')
                else:
                    self.get_logger().error(f'Station failed to start charging: {error_message[0]}')
            else:
                self.get_logger().error('Station start charging timed out!')
                if not future.done():
                    future.cancel()
            
        except Exception as e:
            self.get_logger().error(f'Failed to create station start charging request: {str(e)}')
        finally:
            # CRITICAL: Reset the flag whether success or failure
            self.station_landed_prep_in_progress = False

########################################
# END - BASIC SERVICE CLIENTS TO ARDUINO NODE
########################################

# HANDLER FOR ALL APP REQUESTS THAT COME FROM API RECIVE NODE
    def handle_app_request(self, request, response):
        """Handle incoming app request service calls"""
        self.get_logger().info(f'Received app request: {request.command_type}')
        
        if request.command_type == 'start_mission':
            response.success = self.handle_start_mission_sync(request)
        elif request.command_type == 'pan':
            response.success = self.handle_pan(request)
        elif request.command_type == 'return_to_base':
            response.success = self.handle_return_to_base_sync(request)
        else:
            response.success = True
            self.get_logger().info(f'Command {request.command_type} acknowledged (not yet implemented)')
        
        return response
    
####################################################
# START - HANDLERS FOR DISTINCT APP REQUESTS (THESE METHODS ALSO ACT AS SERVICE CLIENTS TO DRONE STATE MACHINE
# AND IN SOME CASES, ARDUINO NODE (cases where we need to verify station state transition before performing drone action))
####################################################
    
    ##################################
    # Start - Handler sequence for AppRequest (command_type: start_mission)
    ##################################
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
                    self.state_machine.prepare_for_takeoff() # GOTO on_enter_Read_For_Takeoff...
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
        
    def upload_mission_to_drone(self):
        """Upload mission to drone - called automatically when entering Ready_For_Takeoff state"""
        
        # Use threading event to wait for drone response synchronously
        success_event = threading.Event()
        upload_success = [False]
        error_message = ['']

        def mission_upload_callback(future: Future):
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'Mission {self.current_mission_id} successfully uploaded to drone!')
                    upload_success[0] = True
                else:
                    self.get_logger().error(f'Mission upload failed for mission {self.current_mission_id}')
                    error_message[0] = 'Drone state machine rejected mission upload'
            except Exception as e:
                self.get_logger().error(f'Mission upload service call failed: {str(e)}')
                error_message[0] = str(e)
            finally:
                success_event.set()

        try:
            # Create drone command request
            drone_request = DroneCommand.Request()
            drone_request.command_type = 'upload_mission'
            drone_request.waypoints = self.current_waypoints
            drone_request.drone_id = self.current_drone_state.drone_id if self.current_drone_state else "UNKNOWN_DRONE"
            drone_request.base_state = self.state_machine.current_state.name

            future = self.drone_command_client.call_async(drone_request)
            future.add_done_callback(mission_upload_callback)

            self.get_logger().info(f'Mission upload request sent to drone - waiting for completion...')
            
            # Wait for the async call to complete (with timeout)
            if success_event.wait(timeout=45.0):  # 45 second timeout (longer than drone's 30s)
                if upload_success[0]:
                    self.get_logger().info('Mission upload sequence completed successfully!')
                else:
                    self.get_logger().error(f'Mission upload sequence failed: {error_message[0]}')
                    # TODO: Handle mission upload failure - maybe close station and retry
            else:
                self.get_logger().error('Mission upload sequence timed out!')
                if not future.done():
                    future.cancel()

        except Exception as e:
            self.get_logger().error(f'Failed to create mission upload request: {str(e)}')

    ##################################
    # End - Handler sequence for AppRequest (command_type: start_mission)
    ##################################

    ##################################
    # Start - Handler sequence for AppRequest (command_type: pan)
    ##################################
        
    def handle_pan(self, request):
        """Validate drone state before panning and sending command to drone"""
        # Step 1: Validate drone state
        if self.current_drone_state is None:
            self.get_logger().warn('No current drone state. Is drone connected?')
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

    ##################################
    # End - Handler sequence for AppRequest (command_type: pan)
    ##################################

    ##################################
    # Start - Handler sequence for AppRequest (command_type: return_to_base)
    ##################################

    def handle_return_to_base_sync(self, request):
        """Handle return to base command - validate and initiate RTL sequence"""
        
        # Step 1: Validate base station state (should be Mission_In_Progress)
        if self.state_machine.current_state.name != 'Mission in progress':
            self.get_logger().error(f'Cannot return to base - base station not in Mission_In_Progress state. Current state: {self.state_machine.current_state.name}')
            return False

        # Step 2: Validate drone state  
        if self.current_drone_state is None:
            self.get_logger().error('Cannot return to base - no drone state received. Is drone connected?')
            return False
        
        if self.current_drone_state.current_state != 'Mission in progress':
            self.get_logger().error(f'Cannot return to base - drone not in Mission_In_Progress state. Current drone state: {self.current_drone_state.current_state}')
            return False
        
        # Step 3: Validate RTL waypoints
        if not request.waypoints or len(request.waypoints) == 0:
            self.get_logger().error('Cannot return to base - no RTL waypoints provided')
            return False
        
        # Step 4: Send to drone state machine and wait for completion
        self.get_logger().info('Sending return_to_base command to drone - waiting for completion...')

        # Use threading event to wait for drone response synchronously
        success_event = threading.Event()
        rtl_success = [False]
        error_message = ['']

        self.current_mission_id = request.mission_id
        self.current_waypoints = request.waypoints

        def rtl_callback(future: Future):
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info('Return to base command successfully completed! Drone is returning to base!')
                    rtl_success[0] = True
                else:
                    self.get_logger().error('Return to base command failed at drone')
                    error_message[0] = f'Drone state machine rejected RTL command. Current drone state: {self.current_drone_state.current_state}'
            except Exception as e:
                self.get_logger().error(f'Return to base service call failed: {str(e)}')
                error_message[0] = str(e)
            finally:
                success_event.set()

        try:
            drone_request = DroneCommand.Request()
            drone_request.command_type = 'return_to_base'
            drone_request.waypoints = self.current_waypoints
            drone_request.drone_id = self.current_drone_state.drone_id if self.current_drone_state else "UNKNOWN_DRONE"
            drone_request.base_state = self.state_machine.current_state.name

            future = self.drone_command_client.call_async(drone_request)
            future.add_done_callback(rtl_callback)
            
            # Wait for the async call to complete (with timeout)
            if success_event.wait(timeout=45.0):  # 45 second timeout (longer than drone's 30s)
                if rtl_success[0]:
                    return True
                else:
                    self.get_logger().error(f'RTL sequence failed: {error_message[0]}')
                    return False
            else:
                self.get_logger().error('RTL sequence timed out!')
                if not future.done():
                    future.cancel()
                return False
                
        except Exception as e:
            self.get_logger().error(f'Failed to create RTL request: {str(e)}')
            return False

    ##################################
    # End - Handler sequence for AppRequest (command_type: return_to_base)
    ##################################

####################################################
# END - HANDLERS FOR DISTINCT APP REQUESTS (THESE METHODS ALSO ACT AS SERVICE CLIENTS TO DRONE STATE MACHINE
# AND IN SOME CASES, ARDUINO NODE (cases where we need to verify station state transition before performing drone action))
####################################################

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