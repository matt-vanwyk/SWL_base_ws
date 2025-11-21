#!/usr/bin/env python3

import serial
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from swl_base_interfaces.srv import BaseCommand
from std_msgs.msg import String

class ArduinoNode(Node):
    def __init__(self):
        super().__init__('arduino_node')

        # Initialize serial connection
        try:
            self.value = "/dev/arduino_base"
            self.ser = serial.Serial(
                port=self.value,
                baudrate=9600,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f'Serial connection established on {self.value}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Arduino: {str(e)}')
            self.get_logger().error('Falling back to simulation mode')
            self.ser = None

        self.callback_group = ReentrantCallbackGroup()

        # Track individual component states
        # Bit mapping: Bit 2=doors, Bit 1=arms, Bit 0=charger
        self.doors_open = False      # Bit 2: 0=closed, 1=open
        self.arms_centered = True    # Bit 1: 0=uncentered, 1=centered  
        self.charger_on = True       # Bit 0: 0=off, 1=on

        # Calculate initial combined state
        self.current_state = self._calculate_combined_state()

        # Service server for base state machine
        self.base_command_server = self.create_service(
            BaseCommand,
            '/base_station/command',
            self.handle_station_command
        )

        # Publisher for hardware state (for api_post node)
        self.hardware_state_publisher = self.create_publisher(
            String,
            '/arduino/hardware_state',
            10
        )

        # Timer for publishing hardware state
        self.state_publish_timer = self.create_timer(
            1.0,  # Publish every 1 second
            self.publish_hardware_state,
            callback_group=self.callback_group
        )

        self.current_system_mode = "manual"  # Default

        self.get_logger().info('Arduino node has started')
        self.get_logger().info(f'Initial state: {self.current_state:03b}')

    #####################################
    # SIMULATION
    #####################################

    def simulate_serial_communication(self, target_state):
        """Simulate sending command to Arduino and receiving response"""
        
        #Test for successful moving of the station
        current_state = target_state

        time.sleep(5.0) # Simulate station movement

        #Test for station moving failure
        # current_state = 100000000000
        return current_state
        
    #####################################
    # SIMULATION
    #####################################

    def send_arduino_command(self, target_state):
            """Send command to Arduino and receive response"""
            if self.ser is None:
                self.get_logger().warn('No serial connection - using simulation')
                return self.simulate_serial_communication(target_state)
            
            try:
                # Clear any existing data in the buffer
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                
                # Send target state as a 3-bit binary string
                command = f"{target_state:03b}\n"
                self.ser.write(command.encode())
                self.get_logger().info(f'Sent to Arduino: {command.strip()}')
                
                # Wait for Arduino to process and start movement
                time.sleep(0.5)
                
                # Read all responses until we get the final STATE: response
                final_state = None
                timeout_counter = 0
                max_timeout = 120  # 120 seconds max wait time
                
                while final_state is None and timeout_counter < max_timeout:
                    if self.ser.in_waiting > 0:
                        response = self.ser.readline().decode().strip()
                        self.get_logger().info(f'Arduino: {response}')
                        
                        # Look for the final state response
                        if response.startswith("STATE:"):
                            state_str = response.split(":")[1]
                            if len(state_str) == 3 and all(c in '01' for c in state_str):
                                final_state = int(state_str, 2)
                                self.get_logger().info(f'Arduino movement completed - Final state: {final_state:03b}')
                            else:
                                self.get_logger().error(f'Invalid state format: {state_str}')
                                break
                    else:
                        time.sleep(0.5)
                        timeout_counter += 1
                
                if final_state is not None:
                    return final_state
                else:
                    self.get_logger().error('Arduino communication timeout - falling back to simulation')
                    return self.simulate_serial_communication(target_state)
                
            except Exception as e:
                self.get_logger().error(f'Serial communication error: {str(e)}')
                return self.simulate_serial_communication(target_state)

    def _calculate_combined_state(self):
        """Calculate the combined bit mask state from individual components"""
        return (
            (int(self.doors_open) << 2) |      # Bit 2: doors (0=closed, 1=open)
            (int(self.arms_centered) << 1) |   # Bit 1: arms (0=uncentered, 1=centered)  
            (int(self.charger_on) << 0)        # Bit 0: charger (0=off, 1=on)
        )

    def _update_component_states_from_mask(self, state_mask):
        """Update individual component states from a combined bit mask"""
        self.doors_open = bool(state_mask & 0b100)     # Extract bit 2
        self.arms_centered = bool(state_mask & 0b010)  # Extract bit 1
        self.charger_on = bool(state_mask & 0b001)   

    def publish_hardware_state(self):
        """Publish current hardware state and system mode"""
        # Create state message: "mode:maintenance,state:101"
        state_msg = String()
        state_msg.data = f"mode:{self.current_system_mode},state:{self.current_state:03b}"
        
        self.hardware_state_publisher.publish(state_msg)
        
        # Debug log (optional - can be removed later)
        self.get_logger().debug(f'Published hardware state: {state_msg.data}')

    def update_system_mode_from_command(self, command):
        """Infer system mode from command type"""
        if command.startswith('manual_'):
            self.current_system_mode = "maintenance"
        elif command in ['home_station', 'prepare_for_takeoff', 'secure_station', 
                        'prepare_for_landing', 'start_charging']:
            # These are mission-related commands, likely manual or autonomous mode
            # For now, assume manual mode unless we get explicit mode info
            if self.current_system_mode == "maintenance":
                self.current_system_mode = "manual"  # Exiting maintenance

    ########################################
    # START - HANDLERS FOR SERVICE CALLS FROM BASE_STATE_MACHINE
    ########################################

    def handle_station_command(self, request, response):
        """Function to send desired state to Arduino with stateful component control"""
        self.get_logger().info(f'Received command: {request.command}')
        
        try:
            self.update_system_mode_from_command(request.command)
            # Handle different command types
            if request.command == 'home_station':
                # Set all components to home state
                self.doors_open = False    # Doors closed
                self.arms_centered = True  # Arms centered
                self.charger_on = True     # Charger on
                self.get_logger().info('Setting all components to home state')
                
            elif request.command == 'prepare_for_takeoff':
                # Open doors, keep arms/charger as-is for takeoff
                self.doors_open = True     # Doors open
                self.arms_centered = False # Arms uncentered  
                self.charger_on = False    # Charger off
                self.get_logger().info('Preparing for takeoff: opening doors, uncentering arms, turning off charger')
                
            elif request.command == 'secure_station':
                # Close doors for mission
                self.doors_open = False    # Doors closed
                self.arms_centered = False # Arms uncentered
                self.charger_on = False    # Charger off
                self.get_logger().info('Securing station: closing doors')
                
            elif request.command == 'prepare_for_landing':
                # Open doors for landing
                self.doors_open = True     # Doors open
                self.arms_centered = False # Arms uncentered
                self.charger_on = False    # Charger off
                self.get_logger().info('Preparing for landing: opening doors')
                
            elif request.command == 'start_charging':
                # Setup for charging
                self.doors_open = False    # Doors closed
                self.arms_centered = True  # Arms centered
                self.charger_on = True     # Charger on
                self.get_logger().info('Starting charging: closing doors, centering arms, turning on charger')
                
            # MAINTENANCE COMMANDS - Only modify specific components
            elif request.command == 'manual_open_hatch':
                # Only toggle doors, preserve arms and charger
                self.doors_open = True
                self.get_logger().info(f'Manual command: opening hatch (preserving arms: {self.arms_centered}, charger: {self.charger_on})')
                
            elif request.command == 'manual_close_hatch':
                # Only toggle doors, preserve arms and charger
                self.doors_open = False
                self.get_logger().info(f'Manual command: closing hatch (preserving arms: {self.arms_centered}, charger: {self.charger_on})')
                
            elif request.command == 'manual_centre':
                # Only toggle arms, preserve doors and charger
                self.arms_centered = True
                self.get_logger().info(f'Manual command: centering arms (preserving doors: {self.doors_open}, charger: {self.charger_on})')
                
            elif request.command == 'manual_uncentre':
                # Only toggle arms, preserve doors and charger  
                self.arms_centered = False
                self.get_logger().info(f'Manual command: uncentering arms (preserving doors: {self.doors_open}, charger: {self.charger_on})')
                
            elif request.command == 'manual_enable_charge':
                # Only toggle charger, preserve doors and arms
                self.charger_on = True
                self.get_logger().info(f'Manual command: enabling charger (preserving doors: {self.doors_open}, arms: {self.arms_centered})')
                
            elif request.command == 'manual_disable_charge':
                # Only toggle charger, preserve doors and arms
                self.charger_on = False
                self.get_logger().info(f'Manual command: disabling charger (preserving doors: {self.doors_open}, arms: {self.arms_centered})')
                
            else:
                self.get_logger().error(f'Unknown command: {request.command}')
                response.success = False
                response.state = self.current_state
                return response
            
            # Calculate the target state from individual components
            target_state = self._calculate_combined_state()
            
            self.get_logger().info(f'Target state: {target_state:03b} (doors: {self.doors_open}, arms: {self.arms_centered}, charger: {self.charger_on})')
            
            # Send to Arduino
            result_state = self.send_arduino_command(target_state)
            
            # Update current state and verify success
            self.current_state = result_state
            response.success = (result_state == target_state)
            response.state = result_state
            
            if response.success:
                self.get_logger().info(f'Command executed successfully. Final state: {result_state:03b}')
            else:
                self.get_logger().error(f'Command failed. Expected: {target_state:03b}, Got: {result_state:03b}')
                # Update component states to match actual hardware state
                self._update_component_states_from_mask(result_state)
            
            return response
            
        except Exception as e:
            self.get_logger().error(f'Error executing command {request.command}: {str(e)}')
            response.success = False
            response.state = self.current_state
            return response
    
    ########################################
    # END - HANDLERS FOR SERVICE CALLS FROM BASE_STATE_MACHINE
    ########################################

    def __del__(self):
        """Clean up serial connection"""
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Serial connection closed')
    
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