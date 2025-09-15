# SWL_base_ws
ROS2 system for base station

# ROS2 Service Creation Guide
**Complete Step-by-Step Instructions for Adding New Commands to Your Drone System**

## Table of Contents
1. [Service Pattern Decision Tree](#service-pattern-decision-tree)
2. [Sequential Pattern Guide](#sequential-pattern-guide)
3. [Simple Pattern Guide](#simple-pattern-guide)
4. [Complete Code Templates](#complete-code-templates)
5. [Testing Your New Service](#testing-your-new-service)
6. [Troubleshooting Common Issues](#troubleshooting-common-issues)

---

## Service Pattern Decision Tree

**Before adding any new command, ask yourself:**

### Use **Sequential Pattern** When:
- ✅ Multiple dependent steps must complete in order
- ✅ API caller needs to know the final result of the entire operation
- ✅ Each step depends on the previous step succeeding
- ✅ Critical operations that must be confirmed (mission start, abort, land)
- ✅ Hardware state changes that must be verified

**Examples:** `start_mission`, `abort_mission`, `return_to_base`, `land_drone`

### Use **Simple Pattern** When:
- ✅ Single independent action
- ✅ User can verify success visually (camera feed, telemetry)
- ✅ Real-time interactive controls
- ✅ Don't need to wait for operation completion
- ✅ Immediate response expected

**Examples:** `turn_left`, `turn_right`, `manual_mode`, `adjust_altitude`, `pan_camera`

---

## Sequential Pattern Guide
*For critical operations that require step-by-step confirmation*

### Step 1: Add Command to API Receive
**File:** `src/base_package/base_package/api_receive.py`

```python
# Add to valid_commands list
self.valid_commands = [
    'abort_mission',      # Add your new command here
    'start_mission', 
    'turn_left', 'turn_right',
    # ... existing commands
]
```

### Step 2: Create Service Handler in Base State Machine
**File:** `src/base_package/base_package/base_state_machine.py`

```python
def handle_app_request(self, request, response):
    """Route commands to appropriate handlers"""
    self.get_logger().info(f'Received app request: {request.command_type}')
    
    if request.command_type == 'start_mission':
        response.success = self.handle_start_mission_sync(request)
    elif request.command_type == 'abort_mission':  # Add your command here
        response.success = self.handle_abort_mission_sync(request)
    else:
        response.success = True
        self.get_logger().info(f'Command {request.command_type} acknowledged')
    
    return response

def handle_abort_mission_sync(self, request):
    """Sequential pattern implementation"""
    
    # Step 1: Validate current state
    if self.state_machine.current_state.name not in ['Ready_For_Takeoff', 'Mission_In_Progress']:
        self.get_logger().error('Cannot abort - invalid state')
        return False
    
    # Step 2: Validate drone state
    if self.current_drone_state is None:
        self.get_logger().error('Cannot abort - no drone connection')
        return False
    
    # Step 3: Execute sequential steps
    self.get_logger().info('Starting abort sequence...')
    
    # Call the synchronous chain
    return self.complete_abort_sequence_sync()

def complete_abort_sequence_sync(self):
    """Execute abort sequence with proper waiting"""
    
    # Threading events for synchronization
    drone_abort_event = threading.Event()
    arduino_secure_event = threading.Event()
    drone_success = [False]
    arduino_success = [False]
    error_messages = []

    # Step 1: Stop drone mission
    def drone_abort_callback(future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Drone abort command successful')
                drone_success[0] = True
                # Trigger next step: secure station
                self.start_station_secure_sync(arduino_secure_event, arduino_success, error_messages)
            else:
                error_messages.append('Drone abort failed')
        except Exception as e:
            error_messages.append(f'Drone abort service failed: {str(e)}')
        finally:
            drone_abort_event.set()

    def start_station_secure_sync(self, arduino_event, arduino_result, errors):
        """Secure station after drone abort"""
        def arduino_secure_callback(future):
            try:
                response = future.result()
                if response.success and response.state == 0b000:  # Closed and secured
                    self.get_logger().info('Station secured successfully')
                    arduino_result[0] = True
                    # Trigger state transition
                    self.state_machine.mission_aborted()  # You'll need to add this transition
                else:
                    errors.append(f'Station secure failed - state: {response.state:03b}')
            except Exception as e:
                errors.append(f'Arduino secure failed: {str(e)}')
            finally:
                arduino_event.set()

        try:
            # Send secure command to Arduino
            arduino_request = BaseCommand.Request()
            arduino_request.command = 'secure_station'  # New Arduino command
            
            future = self.station_command_client.call_async(arduino_request)
            future.add_done_callback(arduino_secure_callback)
            
        except Exception as e:
            errors.append(f'Failed to create Arduino secure request: {str(e)}')
            arduino_event.set()

    try:
        # Start the abort sequence
        drone_request = DroneCommand.Request()
        drone_request.command_type = 'abort_mission'
        drone_request.drone_id = self.current_drone_state.drone_id if self.current_drone_state else "UNKNOWN"
        
        future = self.drone_command_client.call_async(drone_request)
        future.add_done_callback(drone_abort_callback)
        
        self.get_logger().info('Waiting for drone abort...')
        
        # Wait for drone abort (which triggers Arduino secure)
        if not drone_abort_event.wait(timeout=30.0):
            self.get_logger().error('Drone abort timed out!')
            return False
        
        if not drone_success[0]:
            self.get_logger().error(f'Drone abort failed: {", ".join(error_messages)}')
            return False
        
        self.get_logger().info('Drone abort success - waiting for station secure...')
        
        # Wait for station secure
        if not arduino_secure_event.wait(timeout=30.0):
            self.get_logger().error('Station secure timed out!')
            return False
        
        if not arduino_success[0]:
            self.get_logger().error(f'Station secure failed: {", ".join(error_messages)}')
            return False
        
        self.get_logger().info('COMPLETE abort sequence successful!')
        return True
        
    except Exception as e:
        self.get_logger().error(f'Abort sequence failed: {str(e)}')
        return False
```

### Step 3: Add Handler in Drone State Machine
**File:** `src/drone_pkg/drone_pkg/drone_state_machine.py`

```python
def handle_drone_command(self, request, response):
    """Route drone commands"""
    self.get_logger().info(f'Received drone command: {request.command_type}')
    
    if request.command_type == 'upload_mission':
        response.success = self.handle_mission_upload_sync(request)
    elif request.command_type == 'abort_mission':  # Add your command
        response.success = self.handle_abort_mission_sync(request)
    else:
        response.success = False
        self.get_logger().warn(f'Unknown drone command: {request.command_type}')
    
    return response

def handle_abort_mission_sync(self, request):
    """Handle abort with MAVSDK confirmation"""
    
    # Validate drone can abort
    current_state = self.state_machine.current_state.name
    if current_state not in ['Mission_In_Progress', 'Mission_Uploaded']:
        self.get_logger().error(f'Cannot abort from state: {current_state}')
        return False
    
    # Send to MAVSDK and wait for confirmation
    return self.forward_abort_to_mavsdk_sync()

def forward_abort_to_mavsdk_sync(self):
    """Send abort to MAVSDK with synchronous waiting"""
    
    success_event = threading.Event()
    abort_success = [False]
    error_message = ['']

    def mavsdk_abort_callback(future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('MAVSDK abort successful')
                abort_success[0] = True
                # Trigger state transition
                self.state_machine.mission_aborted()  # Add this transition to your state machine
            else:
                error_message[0] = 'MAVSDK abort failed'
        except Exception as e:
            error_message[0] = str(e)
        finally:
            success_event.set()

    try:
        # Create MAVSDK abort request (you'll need to create this service)
        mavsdk_request = AbortMission.Request()  # New service type needed
        
        future = self.mavsdk_abort_client.call_async(mavsdk_request)
        future.add_done_callback(mavsdk_abort_callback)
        
        self.get_logger().info('Abort forwarded to MAVSDK - waiting for response...')
        
        if success_event.wait(timeout=30.0):
            if abort_success[0]:
                return True
            else:
                self.get_logger().error(f'MAVSDK abort failed: {error_message[0]}')
                return False
        else:
            self.get_logger().error('MAVSDK abort timed out!')
            return False
            
    except Exception as e:
        self.get_logger().error(f'Failed to send abort to MAVSDK: {str(e)}')
        return False
```

### Step 4: Add MAVSDK Implementation
**File:** `src/drone_pkg/drone_pkg/mavsdk_node.py`

```python
def __init__(self):
    # Add new service server
    self.abort_mission_server = self.create_service(
        AbortMission,  # You'll need to create this service type
        'drone/abort_mission',
        self.handle_abort_mission
    )

def handle_abort_mission(self, request, response):
    """Handle abort mission request"""
    future = asyncio.run_coroutine_threadsafe(self.abort_mission(), self.loop)
    result = future.result()
    response.success = result['success']
    response.message = result['message']
    return response

async def abort_mission(self):
    """Execute abort mission sequence"""
    try:
        # Stop current mission
        await self.drone.mission.pause_mission()
        self.get_logger().info("Mission paused")
        
        # Return to launch
        await self.drone.action.return_to_launch()
        self.get_logger().info("Returning to launch")
        
        return {"success": True, "message": "Mission aborted successfully"}
    except Exception as e:
        return {"success": False, "message": f"Abort failed: {str(e)}"}
```

---

## Simple Pattern Guide
*For real-time controls and non-critical commands*

### Step 1: Add Command to API Receive
*(Same as Sequential Pattern Step 1)*

### Step 2: Create Simple Handler in Base State Machine
**File:** `src/base_package/base_package/base_state_machine.py`

```python
def handle_app_request(self, request, response):
    """Route commands to appropriate handlers"""
    self.get_logger().info(f'Received app request: {request.command_type}')
    
    if request.command_type == 'start_mission':
        response.success = self.handle_start_mission_sync(request)  # Sequential
    elif request.command_type in ['turn_left', 'turn_right']:
        response.success = self.handle_yaw_command(request)         # Simple
    elif request.command_type == 'manual_mode':
        response.success = self.handle_manual_mode(request)         # Simple
    else:
        response.success = True
    
    return response

def handle_yaw_command(self, request):
    """Simple pattern - fire and forget yaw control"""
    
    # Quick validation only
    if self.current_drone_state is None:
        self.get_logger().error('No drone state - cannot send yaw command')
        return False
    
    if self.current_drone_state.current_state != 'Mission_In_Progress':
        self.get_logger().error('Drone not in mission - yaw command ignored')
        return False
    
    # Send to drone - don't wait for response
    drone_request = DroneCommand.Request()
    drone_request.command_type = request.command_type  # 'turn_left' or 'turn_right'
    drone_request.drone_id = self.current_drone_state.drone_id
    
    def yaw_callback(future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Yaw command executed: {request.command_type}')
            else:
                self.get_logger().warn(f'Yaw command failed: {request.command_type}')
        except Exception as e:
            self.get_logger().error(f'Yaw service call failed: {str(e)}')
    
    future = self.drone_command_client.call_async(drone_request)
    future.add_done_callback(yaw_callback)
    
    return True  # Return immediately - don't wait

def handle_manual_mode(self, request):
    """Simple pattern - switch to manual mode"""
    
    # Quick validation
    if self.current_drone_state is None:
        return False
    
    # Send command
    drone_request = DroneCommand.Request()
    drone_request.command_type = 'manual_mode'
    drone_request.drone_id = self.current_drone_state.drone_id
    
    def manual_callback(future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Manual mode activated')
            else:
                self.get_logger().warn('Manual mode activation failed')
        except Exception as e:
            self.get_logger().error(f'Manual mode service call failed: {str(e)}')
    
    future = self.drone_command_client.call_async(drone_request)
    future.add_done_callback(manual_callback)
    
    return True
```

### Step 3: Add Simple Handler in Drone State Machine
**File:** `src/drone_pkg/drone_pkg/drone_state_machine.py`

```python
def handle_drone_command(self, request, response):
    """Route drone commands"""
    if request.command_type == 'upload_mission':
        response.success = self.handle_mission_upload_sync(request)  # Sequential
    elif request.command_type in ['turn_left', 'turn_right']:
        response.success = self.handle_yaw_command(request)          # Simple
    elif request.command_type == 'manual_mode':
        response.success = self.handle_manual_mode(request)          # Simple
    else:
        response.success = False
    
    return response

def handle_yaw_command(self, request):
    """Simple yaw control - immediate response"""
    
    # Determine yaw direction
    yaw_clockwise = (request.command_type == 'turn_right')
    
    # Send to MAVSDK - don't wait
    yaw_request = SetYaw.Request()  # You'll need to create this service
    yaw_request.yaw_cw = yaw_clockwise
    yaw_request.yaw_degrees = 45.0  # Or get from request
    
    def yaw_mavsdk_callback(future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'MAVSDK yaw executed: {request.command_type}')
            else:
                self.get_logger().warn(f'MAVSDK yaw failed: {request.command_type}')
        except Exception as e:
            self.get_logger().error(f'MAVSDK yaw service failed: {str(e)}')
    
    future = self.mavsdk_yaw_client.call_async(yaw_request)
    future.add_done_callback(yaw_mavsdk_callback)
    
    return True  # Return immediately

def handle_manual_mode(self, request):
    """Switch to manual flight mode"""
    
    # Send to MAVSDK
    mode_request = SetFlightMode.Request()
    mode_request.mode = 'MANUAL'
    
    def mode_callback(future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Manual mode set successfully')
                # Could trigger state transition if needed
            else:
                self.get_logger().warn('Manual mode setting failed')
        except Exception as e:
            self.get_logger().error(f'Manual mode service failed: {str(e)}')
    
    future = self.mavsdk_mode_client.call_async(mode_request)
    future.add_done_callback(mode_callback)
    
    return True
```

### Step 4: Add MAVSDK Implementation
**File:** `src/drone_pkg/drone_pkg/mavsdk_node.py`

```python
def __init__(self):
    # Add simple service servers
    self.set_yaw_server = self.create_service(
        SetYaw,
        'drone/set_yaw',
        self.handle_set_yaw
    )
    
    self.set_flight_mode_server = self.create_service(
        SetFlightMode,
        'drone/set_flight_mode',
        self.handle_set_flight_mode
    )

def handle_set_yaw(self, request, response):
    """Handle yaw command"""
    future = asyncio.run_coroutine_threadsafe(
        self.set_yaw(request.yaw_degrees, request.yaw_cw), 
        self.loop
    )
    result = future.result()
    response.success = result['success']
    response.message = result['message']
    return response

async def set_yaw(self, degrees, clockwise):
    """Execute yaw command"""
    try:
        # Get current heading
        async for heading in self.drone.telemetry.heading():
            current_yaw = heading.heading_deg
            break
        
        # Calculate target yaw
        if clockwise:
            target_yaw = (current_yaw + degrees) % 360
        else:
            target_yaw = (current_yaw - degrees) % 360
        
        # Send yaw command
        await self.drone.action.goto_location(
            latitude_deg=None,  # Keep current position
            longitude_deg=None,
            absolute_altitude_m=None,
            yaw_deg=target_yaw
        )
        
        return {"success": True, "message": f"Yaw set to {target_yaw}°"}
    except Exception as e:
        return {"success": False, "message": f"Yaw failed: {str(e)}"}

def handle_set_flight_mode(self, request, response):
    """Handle flight mode change"""
    future = asyncio.run_coroutine_threadsafe(
        self.set_flight_mode(request.mode), 
        self.loop
    )
    result = future.result()
    response.success = result['success']
    response.message = result['message']
    return response

async def set_flight_mode(self, mode):
    """Set flight mode"""
    try:
        if mode == 'MANUAL':
            # Switch to manual mode (implementation depends on your setup)
            await self.drone.manual_control.start_position_control()
        elif mode == 'MISSION':
            # Switch back to mission mode
            await self.drone.mission.start_mission()
        
        return {"success": True, "message": f"Flight mode set to {mode}"}
    except Exception as e:
        return {"success": False, "message": f"Mode change failed: {str(e)}"}
```

---

## Complete Code Templates

### Service Definition Template
**Create new .srv files in appropriate interface packages**

```bash
# For base station services: src/swl_base_interfaces/srv/
# For drone services: src/swl_drone_interfaces/srv/
# For shared services: src/swl_shared_interfaces/srv/
```

**Example: NewCommand.srv**
```
# Request
string command_parameter
float64 value_parameter
---
# Response
bool success
string message
int32 result_code
```

### Callback Group Template
```python
# In node __init__
self.service_callback_group = ReentrantCallbackGroup()
self.client_callback_group = ReentrantCallbackGroup()

# For service servers
self.my_service = self.create_service(
    ServiceType,
    'service_name',
    self.handler_method,
    callback_group=self.service_callback_group
)

# For service clients
self.my_client = self.create_client(
    ServiceType,
    'service_name',
    callback_group=self.client_callback_group
)
```

---

## Testing Your New Service

### 1. Build and Source
```bash
cd ~/your_workspace
colcon build --packages-select base_package drone_pkg swl_base_interfaces swl_shared_interfaces
source install/setup.bash
```

### 2. Test via Command Line
```bash
# Test service directly
ros2 service call /cloud/app_request swl_base_interfaces/srv/AppRequest \
  "{command_type: 'your_new_command', mission_id: 'TEST', waypoints: []}"

# Test drone service directly  
ros2 service call /drone/command swl_shared_interfaces/srv/DroneCommand \
  "{command_type: 'your_new_command', drone_id: 'TEST'}"
```

### 3. Test via WebSocket
```json
{
  "command_type": "your_new_command",
  "mission_id": "TEST_001",
  "waypoints": []
}
```

### 4. Monitor Logs
```bash
# Watch all logs
ros2 launch your_launch_file.py

# Watch specific node
ros2 run base_package base_state_machine
```

---

## Troubleshooting Common Issues

### Service Not Found
```bash
# Check if service exists
ros2 service list | grep your_service

# Check service type
ros2 service type /your/service/name
```

### Threading Issues
- **Deadlock**: Check callback groups are ReentrantCallbackGroup
- **Timeout**: Increase timeout values or check service availability
- **Race Conditions**: Ensure proper threading.Event usage

### State Machine Issues
- **Invalid Transitions**: Check state machine allows your transition
- **State Validation**: Ensure state checks are correct before commands

### Service Interface Issues
```bash
# Rebuild interfaces
colcon build --packages-select swl_base_interfaces swl_shared_interfaces swl_drone_interfaces

# Check interface generation
ros2 interface show swl_base_interfaces/srv/YourService
```

### Quick Debug Checklist
1. ✅ Command added to `valid_commands` in api_receive.py
2. ✅ Handler added to `handle_app_request()` in base_state_machine.py
3. ✅ Handler added to `handle_drone_command()` in drone_state_machine.py
4. ✅ MAVSDK implementation added if needed
5. ✅ Service interfaces built and sourced
6. ✅ MultiThreadedExecutor configured
7. ✅ Proper callback groups assigned

---

## Summary

**Sequential Pattern:** Use for critical operations requiring step-by-step confirmation
**Simple Pattern:** Use for real-time controls with immediate response

Both patterns follow the same API → Base → Drone → MAVSDK flow, but differ in how they handle waiting for responses. Choose based on whether you need confirmation of completion or just confirmation that the command was sent.

**Remember:** User visual feedback (camera, telemetry) can replace service response confirmation for many interactive commands!
