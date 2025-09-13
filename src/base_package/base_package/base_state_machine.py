#!/usr/bin/env python3

import rclpy
import time
import threading
from rclpy.node import Node
from swl_base_interfaces.srv import BaseCommand, AppRequest
from swl_shared_interfaces.srv import DroneCommand
from statemachine import StateMachine, State
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future

class BaseStateMachine(StateMachine):
    
    # Define State
    Idle = State(initial=True)
    Home = State()

    # Define transitions
    station_homed = Idle.to(Home)

    # Methods for 'on entering' new states
    def on_enter_Idle(self):
        """Called when entering Idle state"""
        self.model.get_logger().info("System boot... homing station")
        
    def on_enter_Home(self):
        """Called when entering Home state"""
        self.model.get_logger().info("Entered HOME state - Station homed and ready")

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

        # Validate the command type
        self.valid_commands = [
            'abort_mission', 'start_mission', 'turn_left', 'turn_right',
            'manual_mode', 'reroute', 'close_hatch', 'open_hatch',
            'close_centering', 'open_centering', 'enable_charge', 'disable_charge' 'start_mission_patrol', 'continue',
            'manual_open_hatch', 'manual_close_hatch', 'manual_centre', 'manual_uncentre', 'manual_enable_charge', 'manual_disable_charge'
        ]

        self.get_logger().info('Base Station State Machine node started')
        self.get_logger().info(f'Initial state: {self.state_machine.current_state.name}')
        
        # Wait for Arduino service to be available
        self.station_command_client.wait_for_service(timeout_sec=10.0)
        
        # Start boot sequence in a separate thread
        self.boot_thread = threading.Thread(target=self.boot_sequence, daemon=True)
        self.boot_thread.start()

    def boot_sequence(self):
        """Boot sequence - automatically home the station if in Idle state"""
        time.sleep(2)  # Wait for everything to initialize
        
        if self.state_machine.current_state.name == 'Idle':
            self.get_logger().info('Starting boot sequence - homing station...')
            self.home_station_call()

    ########################################
    # START - METHODS FOR SERVICE CALLS TO ARDUINO NODE
    ########################################

    def home_station_call(self):
        """Send home command to Arduino"""
        def home_callback(future: Future):
            try:
                response = future.result()
                if response.success:
                    self.get_logger().info(f'Homing successful!')
                    # Transition to Home state
                    if self.state_machine.current_state.name == 'Idle':
                        self.state_machine.station_homed()
                else:
                    self.get_logger().error(f'Homing failed! Arduino state: {response.state:03b}')
                    # Could add retry logic here
                    
            except Exception as e:
                self.get_logger().error(f'Home service call failed: {str(e)}')
        
        # Create and send the request
        request = BaseCommand.Request()
        request.command = 'home_station'
        
        future = self.station_command_client.call_async(request)
        future.add_done_callback(home_callback)

    ########################################
    # START - METHODS FOR SERVICE CALLS TO DRONE STATE MACHINE
    ########################################

    def drone_command_call(self):
        """Client to send drone commands"""
        def drone_command_callback(future: Future):
            try:
                response = future.result()
                if response.success:# and drone state is Ready_To_Fly:
                    self.get_logger().info(f'Mission uploaded successfully')
                    # TODO transition to base station open state (# 100 : Doors open, arms uncentred, charger off)
                    # arduino needs to be sent command
                    # wait for arduino response 
                    # once station in state 100 then send new drone command 'arm'
                else:
                    self.get_logger().error(f'Mission upload failed! Drone or base station in incorrect state')
                    # TODO Could add retry logic here
                    
            except Exception as e:
                self.get_logger().error(f'Mission upload service call failed: {str(e)}')
        
        # Create and send the request
        request = DroneCommand.Request()
        # request.command will depend on the command sent from the api
        
        future = self.station_command_client.call_async(request)
        future.add_done_callback(drone_command_callback)
        
    ########################################
    # END - METHODS FOR SERVICE CALLS TO DRONE STATE MACHINE
    ########################################
        

    ########################################
    # END - METHODS FOR SERVICE CALLS TO ARDUINO_NODE
    ########################################

    ########################################
    # START - HANDLERS FOR SERVICE CALLS FROM API_RECEIVE
    ########################################

    def handle_app_request(self, request, response):
        """Handle incoming app request service calls"""

        self.get_logger().info(f'Received app request: {request.command}')
        # Here we need to check the state of the drone and base station 
        # Instance, if the command sent is a start_mission command, we should 
        # first check if the base station state is 011 and drone state is Ready_To_Fly
        # before sending the mission to the drone and uploading. Then we need to check 
        # that the base station state is 100 before arming the drone to take off
        # if the command is reroute then we need to check the station is
        
        if request.command in self.valid_commands:
            response.success = True
            self.get_logger().info(f'Command {request.command} accepted')
        else:
            response.success = False
            self.get_logger().warn(f'Unknown command "{request.command}"')
        
        return response
    
    ########################################
    # END - HANDLERS FOR SERVICE CALLS FROM API_RECEIVE
    ########################################

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