#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from swl_shared_interfaces.msg import DroneState
from std_msgs.msg import String
import time
import os
from datetime import datetime

class ConnectionMonitor(Node):
    def __init__(self):
        super().__init__('connection_monitor')
        
        # Setup log file
        log_dir = os.path.expanduser('~/SWL_Base_ws/logs')
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_file = open(f'{log_dir}/connection_monitor_{timestamp}.log', 'w', buffering=1)
        self.latest_log_link = f'{log_dir}/latest_connection_monitor.log'
        
        self.log("Connection Monitor Started")
        self.log("=" * 60)
        
        self.last_drone_message_time = None
        self.last_base_message_time = None
        self.message_timeout = 3.0
        
        self.drone_sub = self.create_subscription(
            DroneState,
            'drone/state',
            self.drone_callback,
            10
        )
        
        self.base_sub = self.create_subscription(
            String,
            '/arduino/hardware_state',
            self.base_callback,
            10
        )
        
        self.timer = self.create_timer(1.0, self.check_connection_health)
    
    def log(self, message, level='INFO'):
        """Write to both ROS log and file"""
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        log_line = f"[{timestamp}] [{level}] {message}"
        
        # Write to file
        self.log_file.write(log_line + '\n')
        
        # Also log to ROS
        if level == 'ERROR':
            self.get_logger().error(message)
        elif level == 'WARNING':
            self.get_logger().warning(message)
        elif level == 'DEBUG':
            self.get_logger().debug(message)
        else:
            self.get_logger().info(message)
    
    def drone_callback(self, msg):
        current_time = time.time()
        
        if self.last_drone_message_time is not None:
            latency = current_time - self.last_drone_message_time
            if latency > 2.0:
                self.log(f'High latency from DRONE: {latency:.2f}s', 'WARNING')
            else:
                self.log(f'Drone message OK: {latency:.2f}s', 'DEBUG')
        
        self.last_drone_message_time = current_time
    
    def base_callback(self, msg):
        current_time = time.time()
        
        if self.last_base_message_time is not None:
            latency = current_time - self.last_base_message_time
            if latency > 2.0:
                self.log(f'High latency from BASE: {latency:.2f}s', 'WARNING')
        
        self.last_base_message_time = current_time
    
    def check_connection_health(self):
        current_time = time.time()
        
        # Check drone connection
        if self.last_drone_message_time is None:
            self.log('Waiting for first DRONE message...', 'INFO')
        else:
            time_since_last = current_time - self.last_drone_message_time
            
            if time_since_last > self.message_timeout:
                self.log(
                    f'DRONE CONNECTION LOST! No messages for {time_since_last:.1f}s',
                    'ERROR'
                )
            elif time_since_last > 1.5:
                self.log(
                    f'Slow DRONE connection: {time_since_last:.1f}s since last message',
                    'WARNING'
                )
        
        # Check base connection
        if self.last_base_message_time is None:
            self.log('Waiting for first BASE message...', 'DEBUG')
        else:
            time_since_last = current_time - self.last_base_message_time
            
            if time_since_last > self.message_timeout:
                self.log(
                    f'BASE CONNECTION ISSUE! No hardware updates for {time_since_last:.1f}s',
                    'ERROR'
                )
    
    def __del__(self):
        """Close log file when node shuts down"""
        if hasattr(self, 'log_file'):
            self.log("Connection Monitor Shutting Down")
            self.log_file.close()
            # Create symlink to latest log
            try:
                if os.path.exists(self.latest_log_link):
                    os.remove(self.latest_log_link)
                os.symlink(self.log_file.name, self.latest_log_link)
            except:
                pass

def main():
    rclpy.init()
    node = ConnectionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()