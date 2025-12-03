import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import signal

class VideoStreamerNode(Node):
    def __init__(self):
        super().__init__('video_streamer_node')
        
        self.declare_parameter('video_device', '/dev/video0')
        self.declare_parameter('video_size', '1280x720')
        self.declare_parameter('framerate', '30')
        self.declare_parameter('rtsp_url', 'rtsp://localhost:8554/drone')
        self.declare_parameter('auto_start', True)

        self.video_device = self.get_parameter('video_device').value
        self.video_size = self.get_parameter('video_size').value
        self.framerate = self.get_parameter('framerate').value
        self.rtsp_url = self.get_parameter('rtsp_url').value
        self.auto_start = self.get_parameter('auto_start').value

        self.ffmpeg_process = None

        self.status_pub = self.create_publisher(String, 'video_stream_status', 10)
        self.timer = self.create_timer(5.0, self.check_stream_health)
        self.get_logger().info('Video Streamer Node initialised')

        if self.auto_start:
            self.start_stream()

    def start_stream(self):
            """Start the FFmpeg streaming process"""
            if self.ffmpeg_process is not None:
                self.get_logger().warn('Stream already running')
                return
            
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
                '-g', '60',  # GOP size (keyframe interval)
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
            except Exception as e:
                self.get_logger().error(f'Failed to start stream: {str(e)}')
                self.publish_status('error')
        
    def stop_stream(self):
        """Stop the FFmpeg streaming process"""
        if self.ffmpeg_process is None:
            self.get_logger().warn('No stream running')
            return
        
        try:
            # Send SIGTERM to the process group
            os.killpg(os.getpgid(self.ffmpeg_process.pid), signal.SIGTERM)
            self.ffmpeg_process.wait(timeout=5)
            self.ffmpeg_process = None
            self.get_logger().info('Stopped video stream')
            self.publish_status('stopped')
        except Exception as e:
            self.get_logger().error(f'Error stopping stream: {str(e)}')
    
    def check_stream_health(self):
        """Check if the FFmpeg process is still running"""
        if self.ffmpeg_process is not None:
            poll = self.ffmpeg_process.poll()
            if poll is not None:
                self.get_logger().error(f'FFmpeg process died with code {poll}')
                self.ffmpeg_process = None
                self.publish_status('error')
                
                # Auto-restart
                self.get_logger().info('Attempting to restart stream...')
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