#!/usr/bin/env python3

import serial
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from swl_base_interfaces.srv import BaseCommand

class ArduinoNode(Node):
    def __init__(self):
        super().__init__('arduino_node')

# TODO: USE ACTUAL SERIAL CONNECTION TO ARDUINO

        # # Initialize serial connection
        # self.value = "/dev/ttyUSB0"
        # self.ser = serial.Serial(
        #     port=self.value,
        #     baudrate=9600,
        #     bytesize=serial.EIGHTBITS,
        #     parity=serial.PARITY_NONE,
        #     stopbits=serial.STOPBITS_ONE,
        #     timeout=1
        # )
        # self.ser.reset_input_buffer()
        # self.ser.reset_output_buffer()

        # Global state variable to track bit mask state
        self.current_state = 0

        # Service server for base state machine
        self.base_command_server = self.create_service(
            BaseCommand,
            '/base_station/command',
            self.handle_station_command
        )

        self.get_logger().info('Arduino node has started')
        self.get_logger().info(f'Initial state: {self.current_state:03b}')

    #####################################
    # SIMULATION
    #####################################

    def simulate_serial_communication(self, target_state):
        """Simulate sending command to Arduino and receiving response"""
        
        #Test for successful moving of the station
        current_state = target_state

        time.sleep(10.0) # Simulate station movement

        #Test for station moving failure
        # current_state = 100000000000
        return current_state
        
    #####################################
    # SIMULATION
    #####################################

    ########################################
    # START - HANDLERS FOR SERVICE CALLS FROM BASE_STATE_MACHINE
    ########################################

    def handle_station_command(self, request, response):
        """Function to send desired state to Arduino"""
        self.get_logger().info(f'Received command: {request.command}')
        
        try:
            target_state = None
            
            if request.command == 'home_station':
                target_state = 0b011  # Home position
            elif request.command == 'prepare_for_takeoff':
                target_state = 0b100  # Doors open, arms uncentered, charger off
            elif request.command == 'secure_station':
                target_state = 0b000 # Doors closed, arms uncentered, charger off
            elif request.command == 'prepare_for_landing':
                target_state = 0b100 # Doors open, arms uncentered, charger off
            elif request.command == 'start_charging':
                target_state = 0b011 # Doors closed, arms centred, charger on
            
            if target_state is not None:
                # Send desired state to Arduino (simulated)
                received_state = self.simulate_serial_communication(target_state)
                
                response.state = received_state
                response.success = (received_state == target_state)
                
                if response.success:
                    self.get_logger().info(f'Command succeeded - State: {received_state:03b}')
                else:
                    self.get_logger().warn(f'Command failed - Expected: {target_state:03b}, Got: {received_state:03b}')
            else:
                response.state = self.current_state
                response.success = False
                self.get_logger().warn(f'Unknown command: {request.command}')
                
        except Exception as e:
            self.get_logger().error(f'Error handling command: {str(e)}')
            response.state = self.current_state
            response.success = False
            
        return response
    
    ########################################
    # END - HANDLERS FOR SERVICE CALLS FROM BASE_STATE_MACHINE
    ########################################
    
def main():
    rclpy.init()
    
    test_server = ArduinoNode()
    
    try:
        rclpy.spin(test_server)
    except KeyboardInterrupt:
        test_server.get_logger().info('Service server shutting down...')
    finally:
        test_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()