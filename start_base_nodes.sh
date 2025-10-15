#!/bin/bash

# Base Station Multi-Terminal Launch Script (tmux version)
# This script launches each ROS2 node in a separate tmux tab for easier debugging

# Set ROS2 environment variables (matching ~/.bashrc)
export ROS_DOMAIN_ID=42
# Use default RMW implementation (FastDDS) - same as bashrc
unset RMW_IMPLEMENTATION

# Kill existing session if it exists
tmux kill-session -t base_system 2>/dev/null

# Create new tmux session with first window/tab
tmux new-session -d -s base_system -n "API_Receive" "cd ~/SWL_Base_ws && source /opt/ros/humble/setup.bash && source ~/SWL_Base_ws/install/setup.bash && echo '================================================' && echo 'API RECEIVE NODE (WebSocket Receiver)' && echo '================================================' && echo '' && ros2 run base_package api_receive; bash"

# Create second window/tab
tmux new-window -t base_system -n "API_Post" "cd ~/SWL_Base_ws && source /opt/ros/humble/setup.bash && source ~/SWL_Base_ws/install/setup.bash && echo '================================================' && echo 'API POST NODE (WebSocket Publisher)' && echo '================================================' && echo '' && ros2 run base_package api_post; bash"

# Create third window/tab
tmux new-window -t base_system -n "StateMachine" "cd ~/SWL_Base_ws && source /opt/ros/humble/setup.bash && source ~/SWL_Base_ws/install/setup.bash && echo '================================================' && echo 'BASE STATE MACHINE' && echo '================================================' && echo '' && ros2 run base_package base_state_machine; bash"

# Create fourth window/tab
tmux new-window -t base_system -n "Arduino" "cd ~/SWL_Base_ws && source /opt/ros/humble/setup.bash && source ~/SWL_Base_ws/install/setup.bash && echo '================================================' && echo 'ARDUINO NODE (Hardware Interface)' && echo '================================================' && echo '' && ros2 run base_package arduino_node; bash"

# Select first window
tmux select-window -t base_system:0

# Auto-attach to the session (only if running interactively)
if [ -t 0 ]; then
    echo "Attaching to tmux session..."
    tmux attach -t base_system
else
    echo "Running in background (systemd mode)"
    echo "Use 'tmux attach -t base_system' to view logs"
fi
