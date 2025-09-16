# ROS2 Synchronous Service Chain Template

**Complete guide for creating reliable synchronous service chains through the drone system architecture.**

## Architecture Flow
```
WebSocket Client → API Receive → Base State Machine → Drone State Machine → MAVSDK Node
                                        ↓ (waits)        ↓ (waits)         ↓
                  API Response ← Base Response ← Drone Response ← MAVSDK Response
```

---

## Step 1: Add Command to API Receive

**File:** `src/base_package/base_package/api_receive.py`

```python
# Add new command to valid_commands list
self.valid_commands = [
    'abort_mission', 'start_mission', 'pan', 'return_to_base',
    'your_new_command',  # ADD YOUR COMMAND HERE
    'manual_mode', 'reroute'
    # ... existing commands
]
```

**Test:** Command validation passes, no "Unknown command_type" errors

---

## Step 2: Add Handler in Base State Machine

**File:** `src/base_package/base_package/base_state_machine.py`

### 2a. Add Route in handle_app_request()

```python
def handle_app_request(self, request, response):
    """Handle incoming app request service calls"""
    self.get_logger().info(f'Received app request: {request.command_type}')
    
    if request.command_type == 'start_mission':
        response.success = self.handle_start_mission_sync(request)
    elif request.command_type == 'return_to_base':
        response.success = self.handle_return_to_base_sync(request)
    elif request.command_type == 'your_new_command':  # ADD ROUTE
        response.success = self.handle_your_new_command_sync(request)
    else:
        response.success = True
        self.get_logger().info(f'Command {request.command_type} acknowledged (not yet implemented)')
    
    return response
```

### 2b. Create Synchronous Handler Method

```python
def handle_your_new_command_sync(self, request):
    """Handle your new command - validate and forward to drone"""
    
    # Step 1: Validate base station state
    valid_states = ['Mission in progress', 'Home']  # Define valid states
    if self.state_machine.current_state.name not in valid_states:
        self.get_logger().error(f'Cannot execute command - invalid base state: {self.state_machine.current_state.name}')
        return False

    # Step 2: Validate drone state
    if self.current_drone_state is None:
        self.get_logger().error('Cannot execute command - no drone state received')
        return False
    
    valid_drone_states = ['Mission in progress', 'Loiter']  # Define valid drone states
    if self.current_drone_state.current_state not in valid_drone_states:
        self.get_logger().error(f'Cannot execute command - invalid drone state: {self.current_drone_state.current_state}')
        return False
    
    # Step 3: Validate request data (customize as needed)
    if hasattr(request, 'waypoints') and (not request.waypoints or len(request.waypoints) == 0):
        self.get_logger().error('Cannot execute command - no waypoints provided')
        return False
    
    # Step 4: Log command details
    self.get_logger().info(f'Executing {request.command_type} command...')
    
    # Step 5: Forward to drone with synchronous waiting
    return self.forward_command_to_drone_sync(request)

def forward_command_to_drone_sync(self, request):
    """Forward command to drone and wait for completion - SYNCHRONOUS"""
    
    # Use threading event for synchronous waiting
    success_event = threading.Event()
    command_success = [False]
    error_message = ['']

    def drone_callback(future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'{request.command_type} command completed successfully!')
                command_success[0] = True
            else:
                self.get_logger().error(f'{request.command_type} command failed at drone')
                error_message[0] = f'Drone rejected command. Current state: {self.current_drone_state.current_state}'
        except Exception as e:
            self.get_logger().error(f'{request.command_type} service call failed: {str(e)}')
            error_message[0] = str(e)
        finally:
            success_event.set()  # CRITICAL: Always signal completion

    try:
        # Create drone command request
        drone_request = DroneCommand.Request()
        drone_request.command_type = request.command_type
        
        # Copy relevant fields from original request
        if hasattr(request, 'waypoints'):
            drone_request.waypoints = request.waypoints
        if hasattr(request, 'yaw_cw'):
            drone_request.yaw_cw = request.yaw_cw
            
        drone_request.drone_id = self.current_drone_state.drone_id
        drone_request.base_state = self.state_machine.current_state.name

        # Make async call
        future = self.drone_command_client.call_async(drone_request)
        future.add_done_callback(drone_callback)

        self.get_logger().info(f'{request.command_type} command sent to drone - waiting for completion...')
        
        # SYNCHRONOUS WAIT - This is the key part
        if success_event.wait(timeout=45.0):  # Timeout > drone timeout (30s)
            if command_success[0]:
                return True
            else:
                self.get_logger().error(f'{request.command_type} failed: {error_message[0]}')
                return False
        else:
            self.get_logger().error(f'{request.command_type} timed out!')
            if not future.done():
                future.cancel()
            return False
            
    except Exception as e:
        self.get_logger().error(f'Failed to create {request.command_type} request: {str(e)}')
        return False
```

**Test:** Base state machine logs show proper sequence with "waiting for completion" before "completed successfully"

---

## Step 3: Add Handler in Drone State Machine

**File:** `src/drone_pkg/drone_pkg/drone_state_machine.py`

### 3a. Add Route in handle_drone_command()

```python
def handle_drone_command(self, request, response):
    """Handle commands from base station"""
    self.get_logger().info(f'Received drone command: {request.command_type}')
    
    if request.command_type == 'upload_mission':
        response.success = self.handle_mission_upload_sync(request)
    elif request.command_type == 'return_to_base':
        response.success = self.handle_return_to_base_sync(request)
    elif request.command_type == 'your_new_command':  # ADD ROUTE
        response.success = self.handle_your_new_command_sync(request)
    elif request.command_type == 'pan':
        response.success = self.handle_pan(request)  # Simple async pattern
    else:
        response.success = False
        self.get_logger().warn(f'Unknown drone command: {request.command_type}')
    
    return response
```

### 3b. Create Synchronous Handler Method

```python
def handle_your_new_command_sync(self, request):
    """Handle your command synchronously - waits for MAVSDK response"""
    
    # Step 1: Validate drone state
    valid_states = ['Mission in progress', 'Loiter', 'Pan']
    current_state = self.state_machine.current_state.name
    if current_state not in valid_states:
        self.get_logger().error(f'Cannot execute command from state: {current_state}')
        return False
    
    # Step 2: Process request data (customize as needed)
    self.get_logger().info(f'Processing {request.command_type} with drone {request.drone_id}')
    
    # Step 3: Forward to MAVSDK with synchronous waiting
    return self.forward_to_mavsdk_sync(request)

def forward_to_mavsdk_sync(self, request):
    """Forward command to MAVSDK and wait for completion - SYNCHRONOUS"""
    
    # Use threading event for synchronous waiting
    success_event = threading.Event()
    mavsdk_success = [False]
    error_message = ['']

    def mavsdk_callback(future: Future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'MAVSDK {request.command_type} successful!')
                
                # Optional: Trigger state machine transition
                # self.state_machine.your_transition_method()
                
                mavsdk_success[0] = True
            else:
                self.get_logger().error(f'MAVSDK {request.command_type} failed: {response.message}')
                error_message[0] = response.message
        except Exception as e:
            self.get_logger().error(f'MAVSDK service call failed: {str(e)}')
            error_message[0] = str(e)
        finally:
            success_event.set()  # CRITICAL: Always signal completion

    try:
        # Create MAVSDK request (customize based on your service)
        mavsdk_request = YourMAVSDKService.Request()
        
        # Copy relevant fields
        if hasattr(request, 'waypoints'):
            mavsdk_request.waypoints = request.waypoints
        # Add other fields as needed
        
        # Make async call to MAVSDK
        future = self.your_mavsdk_client.call_async(mavsdk_request)
        future.add_done_callback(mavsdk_callback)
        
        self.get_logger().info(f'{request.command_type} forwarded to MAVSDK - waiting for response...')
        
        # SYNCHRONOUS WAIT - This ensures proper ordering
        if success_event.wait(timeout=30.0):
            if mavsdk_success[0]:
                return True
            else:
                self.get_logger().error(f'MAVSDK {request.command_type} failed: {error_message[0]}')
                return False
        else:
            self.get_logger().error(f'MAVSDK {request.command_type} timed out!')
            if not future.done():
                future.cancel()
            return False
            
    except Exception as e:
        self.get_logger().error(f'Failed to forward {request.command_type} to MAVSDK: {str(e)}')
        return False
```

**Test:** Drone state machine logs show "forwarded to MAVSDK" before "MAVSDK successful"

---

## Step 4: Add MAVSDK Implementation (if needed)

**File:** `src/drone_pkg/drone_pkg/mavsdk_node.py`

### 4a. Add Service Server

```python
def __init__(self):
    # ... existing code ...
    
    # Add new service server
    self.your_service_server = self.create_service(
        YourMAVSDKService,  # Create this service type in interfaces
        'drone/your_service',
        self.handle_your_service
    )
```

### 4b. Create Service Handler

```python
def handle_your_service(self, request, response):
    """Handle your service request"""
    future = asyncio.run_coroutine_threadsafe(
        self.execute_your_command(request), 
        self.loop
    )
    result = future.result()
    response.success = result['success']
    response.message = result['message']
    return response

async def execute_your_command(self, request):
    """Execute your command using MAVSDK"""
    try:
        # Your MAVSDK implementation here
        # Examples:
        # await self.drone.action.some_action()
        # await self.drone.mission.some_mission_command()
        
        self.get_logger().info("Your command executed successfully")
        return {"success": True, "message": "Command completed"}
        
    except Exception as e:
        self.get_logger().error(f"Your command failed: {str(e)}")
        return {"success": False, "message": f"Error: {str(e)}"}
```

**Test:** MAVSDK node logs show successful execution

---

## Step 5: Create Service Interface (if needed)

**File:** `src/swl_drone_interfaces/srv/YourMAVSDKService.srv`

```
# Request fields
string parameter1
float64 parameter2
swl_shared_interfaces/Waypoint[] waypoints
---
# Response fields  
bool success
string message
```

**Build interfaces:**
```bash
colcon build --packages-select swl_drone_interfaces swl_shared_interfaces
source install/setup.bash
```

---

## Step 6: Testing

### 6a. WebSocket Test Client

```python
#!/usr/bin/env python3
import asyncio
import websockets
import json

async def test_your_command():
    uri = "ws://your_ip:8765"
    
    command_msg = {
        "command_type": "your_new_command",
        "parameter1": "test_value",
        "parameter2": 42.0
        # Add other fields as needed
    }
    
    try:
        async with websockets.connect(uri) as websocket:
            print("Sending command...")
            await websocket.send(json.dumps(command_msg))
            
            response = await websocket.recv()
            response_data = json.loads(response)
            
            if response_data.get("status") == "received":
                print("✅ Command sent successfully!")
            else:
                print(f"❌ Command failed: {response_data}")
                
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    asyncio.run(test_your_command())
```

### 6b. Direct Service Test

```bash
# Test the service directly
ros2 service call /cloud/app_request swl_base_interfaces/srv/AppRequest \
  "{command_type: 'your_new_command', waypoints: []}"
```

### 6c. Expected Log Sequence

```
[api_receive]: Making service call with command: your_new_command
[base_state_machine]: Received app request: your_new_command
[base_state_machine]: your_new_command command sent to drone - waiting for completion...
[drone_state_machine]: Received drone command: your_new_command
[drone_state_machine]: your_new_command forwarded to MAVSDK - waiting for response...
[mavsdk_node]: Your command executed successfully
[drone_state_machine]: MAVSDK your_new_command successful!
[base_state_machine]: your_new_command command completed successfully!
[api_receive]: Base State Machine received API action request: True
```

---

## Key Principles

### ✅ Synchronous Chain Pattern
- Each layer waits for the next layer to complete
- Uses `threading.Event()` for synchronization
- Timeouts increase as you go up the chain (30s → 45s)

### ✅ Error Handling
- Always call `success_event.set()` in finally blocks
- Cancel futures on timeout
- Clear state on failure

### ✅ Logging Strategy
- "Sending/forwarding..." before wait
- "Successful/failed..." after response  
- "Completed successfully!" at each level

### ✅ State Validation
- Check both base and drone states
- Validate request data
- Fail fast with clear error messages

### ❌ Common Mistakes to Avoid
- Fire-and-forget async calls (breaks timing)
- Missing `finally: success_event.set()`
- Timeout shorter than downstream timeout
- Not canceling futures on timeout
- Forgetting to add command to valid_commands

---

## Service Patterns

### Pattern A: Full Chain (WebSocket → Base → Drone → MAVSDK)
Use for: Mission commands, RTL, abort sequences
Example: `start_mission`, `return_to_base`

### Pattern B: Direct to Drone (WebSocket → Base → Drone)  
Use for: Real-time controls, non-critical commands
Example: `pan` (fire-and-forget to MAVSDK)

### Pattern C: Base Only (WebSocket → Base → Arduino)
Use for: Station hardware control
Example: Manual station commands

Choose the pattern based on whether you need confirmation of completion vs just confirmation that the command was sent.