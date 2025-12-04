#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import signal
import time
import requests

class VideoStreamerNode(Node):
    def __init__(self):
        super().__init__('video_streamer_node')
        
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('video_size', '1280x720')
        self.declare_parameter('framerate', '30')
        self.declare_parameter('rtsp_url', 'rtsp://localhost:8554/drone')
        self.declare_parameter('mediamtx_api', 'http://localhost:9997/v3/paths/list')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('restart_on_failure', True)
        self.declare_parameter('health_check_interval', 10.0)  # Check every 10 seconds

        self.video_device = self.get_parameter('video_device').value
        self.video_size = self.get_parameter('video_size').value
        self.framerate = self.get_parameter('framerate').value
        self.rtsp_url = self.get_parameter('rtsp_url').value
        self.mediamtx_api = self.get_parameter('mediamtx_api').value
        self.auto_start = self.get_parameter('auto_start').value
        self.restart_on_failure = self.get_parameter('restart_on_failure').value
        self.health_check_interval = self.get_parameter('health_check_interval').value

        self.ffmpeg_process = None
        self.last_restart_time = 0
        self.restart_count = 0
        self.last_bytes_received = 0
        self.stall_count = 0

        self.status_pub = self.create_publisher(String, 'video_stream_status', 10)
        self.timer = self.create_timer(self.health_check_interval, self.check_stream_health)
        self.get_logger().info('Video Streamer Node initialized')

        if self.auto_start:
            self.start_stream()

    def start_stream(self):
        """Start the FFmpeg streaming process"""
        if self.ffmpeg_process is not None:
            self.get_logger().warn('Stream already running, stopping first...')
            self.stop_stream()
            time.sleep(1)
        
        # Build FFmpeg command
        ffmpeg_cmd = [
            'ffmpeg',
            '-f', 'v4l2',
            '-input_format', 'mjpeg',
            '-video_size', self.video_size,
            '-framerate', str(self.framerate),
            '-i', self.video_device,
            '-c:v', 'libx264',
            '-preset', 'ultrafast',
            '-tune', 'zerolatency',
            '-b:v', '2M',
            '-maxrate', '2M',
            '-bufsize', '4M',
            '-g', '60',
            '-fflags', '+genpts',
            '-rtsp_transport', 'tcp',
            '-timeout', '5000000',  # 5 second timeout for stalled connections
            '-f', 'rtsp',
            self.rtsp_url
        ]
        
        try:
            self.ffmpeg_process = subprocess.Popen(
                ffmpeg_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            self.get_logger().info(f'Started video stream: {self.video_device} -> {self.rtsp_url}')
            self.publish_status('streaming')
            self.restart_count = 0
            self.stall_count = 0
            self.last_bytes_received = 0
        except Exception as e:
            self.get_logger().error(f'Failed to start stream: {str(e)}')
            self.publish_status('error')
    
    def stop_stream(self):
        """Stop the FFmpeg streaming process"""
        if self.ffmpeg_process is None:
            return
        
        try:
            os.killpg(os.getpgid(self.ffmpeg_process.pid), signal.SIGTERM)
            self.ffmpeg_process.wait(timeout=5)
            self.get_logger().info('Stopped video stream')
        except:
            try:
                if self.ffmpeg_process:
                    os.killpg(os.getpgid(self.ffmpeg_process.pid), signal.SIGKILL)
            except:
                pass
        finally:
            self.ffmpeg_process = None
            self.publish_status('stopped')
    
    def is_stream_active(self):
        """Check if MediaMTX is actually receiving data"""
        try:
            response = requests.get(self.mediamtx_api, timeout=2)
            if response.status_code == 200:
                data = response.json()
                for item in data.get('items', []):
                    if item.get('name') == 'drone':
                        is_ready = item.get('ready', False)
                        bytes_received = item.get('bytesReceived', 0)
                        
                        # Check if stream is ready and bytes are increasing
                        if is_ready and bytes_received > self.last_bytes_received:
                            self.last_bytes_received = bytes_received
                            self.stall_count = 0
                            return True
                        elif is_ready and bytes_received == self.last_bytes_received:
                            # Stream is "ready" but no data flowing - stalled!
                            self.stall_count += 1
                            self.get_logger().warn(f'Stream stalled (no new data), count: {self.stall_count}')
                            return self.stall_count < 3  # Allow 3 stalls before restart
                        else:
                            return False
        except Exception as e:
            self.get_logger().warn(f'Failed to check stream status: {str(e)}')
        
        return False
    
    def check_stream_health(self):
        """Check if the FFmpeg process is still running and stream is active"""
        if self.ffmpeg_process is None:
            return
        
        # Check if process crashed
        poll = self.ffmpeg_process.poll()
        if poll is not None:
            self.get_logger().error(f'FFmpeg process died with code {poll}')
            self.ffmpeg_process = None
            self.publish_status('error')
            self.attempt_restart()
            return
        
        # Check if stream is actually sending data to MediaMTX
        if not self.is_stream_active():
            self.get_logger().error('Stream is not active or stalled, restarting...')
            self.stop_stream()
            self.attempt_restart()
    
    def attempt_restart(self):
        """Attempt to restart the stream with exponential backoff"""
        if not self.restart_on_failure:
            return
        
        current_time = time.time()
        if current_time - self.last_restart_time > 2.0:
            self.last_restart_time = current_time
            self.restart_count += 1
            
            wait_time = min(2 ** self.restart_count, 30)
            
            self.get_logger().info(f'Attempting to restart stream in {wait_time}s (attempt #{self.restart_count})...')
            time.sleep(wait_time)
            self.start_stream()
    
    def publish_status(self, status):
        """Publish stream status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
    
    def destroy_node(self):
        """Clean shutdown"""
        self.stop_stream()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VideoStreamerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()