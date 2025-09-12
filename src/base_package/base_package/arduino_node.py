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

        self.current_state = 0

        # Service server for Base State Machine
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
        self.get_logger().info(f'[SIMULATION] Sending command to Arduino: {target_state:03b}')
        
        # Simulate Arduino processing time
        time.sleep(1.0)
        
        # Simulate Arduino response - for simulation, let's say it succeeds 90% of the time
        import random
        success_rate = 0.9  # 90% success rate for testing
        
        if random.random() < success_rate:
            # Simulate successful state change
            self.current_state = target_state
            self.get_logger().info(f'[SIMULATION] Arduino responded: {self.current_state:03b} (SUCCESS)')
            return self.current_state
        else:
            # Simulate failure - Arduino stays in current state
            self.get_logger().warn(f'[SIMULATION] Arduino responded: {self.current_state:03b} (FAILED TO CHANGE)')
            return self.current_state
        
    #####################################
    # SIMULATION
    #####################################

    ########################################
    # START - HANDLERS FOR SERVICE CALLS FROM BASE_STATE_MACHINE
    ########################################

    def handle_station_command(self, request, response):
        """Function to send commands to Arduino"""
        self.get_logger().info(f'Received station command: {request.command}')
        
        try:
            if request.command == 'home_station':
                target_state = 0b011  # 011: Doors closed, arms centred, charger on
                
                self.get_logger().info('Homing station')

                #TODO actually send serial bitmask and wait for response
                # Send serial message to arduino (simulated)
                received_state = self.simulate_serial_communication(target_state)
                
                # Check if received state matches target state
                if received_state == target_state:
                    response.state = received_state
                    response.success = True
                    self.get_logger().info(f'Station homing succeeded! Current state: {received_state:03b}')
                else:
                    response.state = received_state
                    response.success = False
                    self.get_logger().warn(f'Station homing failed! Expected: {target_state:03b}, received: {received_state:03b}')
            
            elif request.command == 'get_status':
                # Just return current state without changing anything
                response.state = self.current_state
                response.success = True
                self.get_logger().info(f'Status request - Current state: {self.current_state:03b}')
                
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
    # START - HANDLERS FOR SERVICE CALLS FROM BASE_STATE_MACHINE
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