# Troubleshooting Guide: Robot Arm Teleoperation Migration from ROS 2 Humble to Jazzy

This document records the investigation and solution for the robotic arm (Kinova Gen3) teleoperation issues, specifically **violent oscillations** and **accumulated latency backlog** encountered after migrating from ROS 2 Humble to Jazzy.

---

## 🔍 Issue Details
After migrating to ROS 2 Jazzy, the teleoperation system exhibited the following symptoms:
1. **Severe Oscillation**: The slave robotic arm vibrated and oscillated violently back and forth when driven by the master device.
2. **Latency Backlog**: The robot responded to the master's movements with a delay of several seconds, which grew worse over time.
3. **Gripper Unresponsive**: Gripper open/close commands from the master device failed to actuate the slave gripper in the simulation/RViz.

---

## 🛠️ Analysis & Root Causes

### 1. Parallel Gripper Action Type Mismatch in ROS 2 Jazzy
* **Cause**: In ROS 2 Jazzy, the parallel gripper controller (`parallel_gripper_action_controller`) uses a new action type: **`control_msgs/action/ParallelGripperCommand`**.
* **Legacy Code**: The teleoperation code (`kinova.py`) was still using the old ROS 2 Humble action type: **`control_msgs/action/GripperCommand`**.
* **Impact**: Due to the type mismatch, the ROS 2 DDS middleware could not connect the action client and the action server. The gripper commands from the master were silently ignored.

### 2. Blocking Call in the Main Control Loop (1-Second Delay)
* **Cause**: The teleoperation code in `kinova.py` checked the gripper action server's availability in every loop iteration (50 Hz):
  ```python
  if not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
      return
  ```
* **Impact**: Because the gripper action server did not respond (due to the type mismatch), `wait_for_server` **blocked the main callback thread for a full 1.0 second** on every iteration.
* **Latency Accumulation**: Blocking a 50 Hz loop (20 ms interval) for 1000 ms starved the ROS 2 SingleThreadedExecutor. The incoming joint states `/joint_states` piled up in the subscriber queue, creating a latency backlog of several seconds.

### 3. Error Accumulation Leading to Loop Instability
* **Cause**: The joint velocity commands sent to the slave are calculated using a hybrid velocity and proportional position controller:
  $$\text{Commanded Velocity} = v_{\text{master}} + K_p \times (\theta_{\text{master}} - \theta_{\text{slave}})$$
* **Impact**: Due to the 1-second lag in the loop, the received slave position state ($\theta_{\text{slave}}$) was stale. This caused the tracking error ($\theta_{\text{master}} - \theta_{\text{slave}}$) to accumulate to a very large value. When the callback finally executed, the P controller with a high gain ($K_p = 1.5$) commanded a massive correction velocity, resulting in a large overshoot, lag, and violent oscillations.

---

## 💡 Solutions Implemented

### Step 1: Set wait_for_server Timeout to Non-blocking
We reduced the `timeout_sec` in `wait_for_server` within [kinova.py](file:///home/an/workspace/harvest_ws/src/harvest_teleop/harvest_teleop/arms/kinova_gen3_7dof/kinova.py) to a non-blocking value (`0.001` seconds). This ensures the control loop is never blocked if the gripper is disconnected or not yet active:
```python
# BEFORE
if not self.gripper_action_client.wait_for_server(timeout_sec=1.0):

# AFTER (NON-BLOCKING)
if not self.gripper_action_client.wait_for_server(timeout_sec=0.001):
```

### Step 2: Migrate Action Client to ParallelGripperCommand
We modified the imports and the goal message structure in [kinova.py](file:///home/an/workspace/harvest_ws/src/harvest_teleop/harvest_teleop/arms/kinova_gen3_7dof/kinova.py) to use `ParallelGripperCommand` along with the dynamically prefixed joint name:
```python
from control_msgs.action import ParallelGripperCommand

# Client Initialization
self.gripper_action_client = ActionClient(self.ctx, ParallelGripperCommand, 'robotiq_gripper_controller/gripper_cmd')

# Goal Message Construction in send_gripper_goal()
ns = self.ctx.get_namespace().strip('/')
clean_ns = ns.rstrip('_')
prefix = f"{clean_ns}_" if clean_ns else ""
joint_name = f"{prefix}robotiq_85_left_knuckle_joint"

goal_msg = ParallelGripperCommand.Goal()
goal_msg.command.name = [joint_name]
goal_msg.command.position = [float(target_position)]
goal_msg.command.effort = [100.0]
self.gripper_action_client.send_goal_async(goal_msg)
```

---

## 📈 Diagnostic Tips: Detecting Latency in Control Loops

To detect whether a control loop is being blocked by serial port I/O or network service calls in the future, measure the loop execution duration inside the timer callback ([teleop_node.py](file:///home/an/workspace/harvest_ws/src/harvest_teleop/harvest_teleop/teleop_node.py)):

```python
def teleop_callback(self):
    start_time = self.get_clock().now()
    
    # ... Teleoperation Logic ...
    
    # Compute loop duration in milliseconds
    duration = (self.get_clock().now() - start_time).nanoseconds / 1e6
    if self._debug_tick % 50 == 0:
        self.get_logger().info(f"DEBUG: teleop_callback took {duration:.2f} ms")
```

> [!IMPORTANT]
> If the control loop is configured to run at $f$ Hz, the callback duration **must be lower** than the cycle time $T = \frac{1000}{f}$ ms.
> * Example: A 50 Hz loop has a budget of **20 ms**. If log durations exceed 20 ms, there is a blocking call that must be refactored to be asynchronous or non-blocking.
