#!/usr/bin/env python3
"""
HARVEST Framework: Hand-admittance and Robotic Direct Trajectory System for Low-Cost Teleoperation
Author: Ari Hartawan
Description: Multi-arm compatible core node. Direct position trajectory mirroring using
             continuous spline interpolation in JointTrajectoryController.
             Supports unrecorded live teleop engagement before dataset recording starts,
             and auto-homes Master & Slave BEFORE dataset recording stops.
"""

import os
import sys
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

# Ensure proper modules routing across the ROS 2 package share paths
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__)) 
SRC_DIR = os.path.abspath(os.path.join(CURRENT_DIR, '..'))
if SRC_DIR not in sys.path:
    sys.path.append(SRC_DIR)

try:
    from ament_index_python.packages import get_package_share_directory
    pkg_share = get_package_share_directory('harvest_teleop')
    if pkg_share not in sys.path:
        sys.path.append(pkg_share)
except Exception:
    pass

try:
    from hardware.stservo.scservo_sdk.port_handler import PortHandler
    from hardware.stservo.scservo_sdk.sms_sts import sms_sts
    from hardware.stservo.scservo_sdk.scservo_def import *
except ImportError as e:
    print(f"Error: Feetech SCServo SDK modules not found. Check local paths: {e}")
    sys.exit(1)

class HarvestTeleopCoreNode(Node):
    def __init__(self):
        super().__init__(
            'harvest_teleop_core_node',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )
        
        self.get_logger().info("HARVEST Framework: Loading parameters dynamically for Direct Trajectory Teleop...")

        # Unpack configurations directly from the Parameter Server
        self.BACKEND_TYPE = self.get_parameter('control.arm_backend_type').value
        self.PORT_NAME = self.get_parameter('hardware.port_name').value
        self.BAUDRATE = self.get_parameter('hardware.baudrate').value
        self.SERVO_IDS = self.get_parameter('hardware.servo_ids').value
        self.SERVO_CENTER = self.get_parameter('hardware.servo_center').value
        self.LIVE_TORQUE_LIMIT = self.get_parameter('hardware.live_torque_limit').value
        self.STIFF_TORQUE_LIMIT = self.get_parameter('hardware.stiff_torque_limit').value
        self.deadband = self.get_parameter('hardware.deadband').value
        
        self.ARM_JOINT_NAMES = self.get_parameter('kinematics.arm_joint_names').value
        self.JOINT_SIGNS = self.get_parameter('kinematics.joint_signs').value
        self.HOME_RADS = self.get_parameter('kinematics.home_radians').value
        
        self.ENCODER_CLOSE = self.get_parameter('gripper.encoder_close').value
        self.ENCODER_OPEN = self.get_parameter('gripper.encoder_open').value
        self.ROBOTIQ_CLOSE = self.get_parameter('gripper.robotiq_close').value
        self.ROBOTIQ_OPEN = self.get_parameter('gripper.robotiq_open').value
        
        self.CONTINUOUS_JOINTS = self.get_parameter('control.continuous_joints').value
        self.admittance_gain = self.get_parameter('control.admittance_gain').value
        
        self.RAD_PER_UNIT = (2 * math.pi) / 4096.0

        ns = self.get_namespace().strip('/')
        clean_ns = ns.rstrip('_')
        prefix = f"{clean_ns}_" if clean_ns else ""
        self.ARM_JOINT_NAMES = [f"{prefix}{name}" for name in self.ARM_JOINT_NAMES]
        
        # State estimation buffers
        self.virtual_targets = {}  
        self.prev_master_rads = list(self.HOME_RADS)
        self.raw_master_rads = list(self.HOME_RADS) 
        self.current_slave_positions = list(self.HOME_RADS)
        
        # Safety state: 0=Standby, 2=Homed & Stiff, 3=Teleop Active (Unrecorded), 4=Teleop Active (Recorded)
        self.safety_state = 0
        self.is_hardware_connected = False
        self.is_homing_in_progress = False
        
        # ROS 2 Interfaces
        # 1. Publish Master Device State
        self.master_state_publisher = self.create_publisher(JointState, 'master_joint_states', 10)
        # 2. Subscribe to Robot Slave States
        self.joint_state_subscriber = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        
        # Trigger Publisher & Subscriber (Global Topic)
        self.trigger_publisher = self.create_publisher(String, '/harvest_safety_trigger', 10)
        self.trigger_subscriber = self.create_subscription(String, '/harvest_safety_trigger', self.safety_trigger_callback, 10)
        
        # Dynamic Backend Selection
        if self.BACKEND_TYPE == "kinova":
            from .arms.kinova_gen3_7dof.kinova import KinovaBackend
            self.arm_backend = KinovaBackend(self)
        else:
            self.get_logger().error(f"Selected backend '{self.BACKEND_TYPE}' is unsupported.")
            sys.exit(1)

        self.initialize_hardware()
        
        # Trigger control loop (50 Hz / 20 ms)
        self.timer = self.create_timer(0.02, self.teleop_callback)
        self.show_menu_prompt()

    def joint_state_callback(self, msg):
        ns = self.get_namespace().strip('/')
        prefix = f"{ns}_" if ns else ""

        for i, joint_name in enumerate(self.ARM_JOINT_NAMES):
            full_joint_name = f"{prefix}{joint_name}"
            
            if full_joint_name in msg.name:
                idx = msg.name.index(full_joint_name)
                self.current_slave_positions[i] = msg.position[idx]
            elif joint_name in msg.name:
                idx = msg.name.index(joint_name)
                self.current_slave_positions[i] = msg.position[idx]

    def initialize_hardware(self):
        self.port_handler = PortHandler(self.PORT_NAME)
        self.packet_handler = sms_sts(self.port_handler)
        if not self.port_handler.openPort() or not self.port_handler.setBaudRate(self.BAUDRATE):
            self.get_logger().error('CRITICAL: Failed to bind serial link to Master device.')
            sys.exit(1)

        for s_id in self.SERVO_IDS:
            pos, _, res, _ = self.packet_handler.ReadPosSpeed(s_id)
            self.virtual_targets[s_id] = pos if res == COMM_SUCCESS else self.SERVO_CENTER
            self.packet_handler.write1ByteTxRx(s_id, 40, 1)
            self.packet_handler.write2ByteTxRx(s_id, 48, self.STIFF_TORQUE_LIMIT)
            self.packet_handler.WritePosEx(s_id, int(self.virtual_targets[s_id]), 1500, 50)
        self.is_hardware_connected = True
        self.get_logger().info('Master device link up and successfully bounded.')

    def show_menu_prompt(self):
        if self.safety_state == 0:
            print("\n" + "="*60)
            print(" [HARVEST SYSTEM] CURRENT: STATE 0 -> STANDBY")
            print(" -> Press [SPACE] in keyboard_node to run INITIAL HOMING.")
            print("="*60)

    def get_shortest_angle_error(self, target_rad, current_rad):
        error = target_rad - current_rad
        return math.atan2(math.sin(error), math.cos(error))

    def safety_trigger_callback(self, msg):
        command = msg.data.strip()
        
        # --- INITIAL HOMING (FIRST STARTUP BEFORE RECORDING) ---
        if command in ['INITIAL_HOME', 'STEP_1']:
            if self.safety_state == 0:
                self.get_logger().info("Triggering INITIAL HOMING for Master & Slave...")
                self.is_homing_in_progress = True
                self.arm_backend.home_both_master_and_slave()
                self.is_homing_in_progress = False
                
                # Lock Master Device with Stiff Torque
                for s_id in self.SERVO_IDS:
                    self.packet_handler.write1ByteTxRx(s_id, 40, 1)
                    self.packet_handler.write2ByteTxRx(s_id, 48, self.STIFF_TORQUE_LIMIT)

                self.safety_state = 2  # State 2 = Homed & Stiff at Home
                self.get_logger().info("✅ INITIAL HOMING COMPLETE! System Homed & Locked (Hold handle & press SPACE to engage Live Teleop)")

        # --- ENGAGE LIVE TELEOP (UNRECORDED - SOFT TORQUE / ADMITTANCE ON) ---
        elif command in ['ENGAGE_TELEOP', 'STEP_3']:
            if self.safety_state == 2:
                self.get_logger().info("Triggering ENGAGE LIVE TELEOP (Unrecorded Mode)... 🕹️")
                
                # Enable Soft Torque Limit for Master Admittance
                for s_id in self.SERVO_IDS:
                    self.packet_handler.write1ByteTxRx(s_id, 40, 1)
                    self.packet_handler.write2ByteTxRx(s_id, 48, self.LIVE_TORQUE_LIMIT)

                self.safety_state = 3  # State 3 = Live Teleop Active (Unrecorded)
                self.get_logger().info("🕹️ LIVE TELEOP ENGAGED (Not recording yet). Position hands & press SPACE to START RECORDING 🔴")

        # --- START RECORDING ---
        elif command == 'START_RECORDING':
            if self.safety_state == 3:
                self.get_logger().info("Triggering START RECORDING... 🔴")
                self.safety_state = 4  # State 4 = Teleop Active + Recording On

        # --- AUTO HOME BEFORE STOPPING DATASET RECORDING ---
        elif command == 'AUTO_HOME_AND_STOP':
            if self.safety_state == 4 and not self.is_homing_in_progress:
                self.get_logger().info("Triggering AUTO-HOME WHILE RECORDING... (Returning Master & Slave to Home position) 🏠")
                self.is_homing_in_progress = True

                # Lock Master Device to Stiff Torque during homing
                for s_id in self.SERVO_IDS:
                    self.packet_handler.write1ByteTxRx(s_id, 40, 1)
                    self.packet_handler.write2ByteTxRx(s_id, 48, self.STIFF_TORQUE_LIMIT)

                # Execute concurrent Master & Slave homing (~2.0 seconds duration)
                # safety_state remains 4 during homing, so logger records the return trajectory to Home!
                self.arm_backend.home_both_master_and_slave()
                
                self.is_homing_in_progress = False

                # Send STOP_RECORDING trigger now that robot & master are safely at Home position!
                self.get_logger().info("Arrived at Home Position! Sending STOP_RECORDING to save dataset... 💾")
                stop_msg = String()
                stop_msg.data = 'STOP_RECORDING'
                self.trigger_publisher.publish(stop_msg)

                self.safety_state = 2  # Return to Homed & Stiff (State 2) for next episode
                self.get_logger().info("✅ Episode Saved & Homed! Locked at Home Position for Next Episode.")

        # --- STOP RECORDING ACKNOWLEDGEMENT ---
        elif command == 'STOP_RECORDING':
            if self.safety_state in [3, 4]:
                self.safety_state = 2
                for s_id in self.SERVO_IDS:
                    self.packet_handler.write1ByteTxRx(s_id, 40, 1)
                    self.packet_handler.write2ByteTxRx(s_id, 48, self.STIFF_TORQUE_LIMIT)

        # --- CANCEL RECORDING ---
        elif command == 'CANCEL_RECORDING':
            if self.safety_state in [3, 4]:
                self.get_logger().warn("Triggering CANCEL RECORDING... Discarding data and returning to Home position.")
                self.is_homing_in_progress = True

                for s_id in self.SERVO_IDS:
                    self.packet_handler.write1ByteTxRx(s_id, 40, 1)
                    self.packet_handler.write2ByteTxRx(s_id, 48, self.STIFF_TORQUE_LIMIT)

                self.arm_backend.home_both_master_and_slave()
                self.is_homing_in_progress = False
                self.safety_state = 2

    def teleop_callback(self):
        # 🔑 Execute teleop loop if in State 3 or State 4 (and not currently performing homing)
        if self.safety_state < 3 or self.is_homing_in_progress:
            return

        if not hasattr(self, '_debug_tick'):
            self._debug_tick = 0
        self._debug_tick += 1

        native_master_velocities = [0.0] * len(self.ARM_JOINT_NAMES)
        
        # --- 1. MASTER ADMITTANCE & HARDWARE READ LOOP ---
        for i, s_id in enumerate(self.SERVO_IDS):
            pos, speed, res, _ = self.packet_handler.ReadPosSpeed(s_id)
            if res == COMM_SUCCESS:
                error = pos - self.virtual_targets[s_id]
                if abs(error) > self.deadband:
                    self.virtual_targets[s_id] += (error * self.admittance_gain)
                    self.packet_handler.WritePosEx(s_id, int(self.virtual_targets[s_id]), 2400, 0)
                
                if s_id == 8:
                    min_encoder = min(self.ENCODER_CLOSE, self.ENCODER_OPEN)
                    max_encoder = max(self.ENCODER_CLOSE, self.ENCODER_OPEN)
                    
                    bounded_pos = max(min_encoder, min(float(pos), max_encoder))
                    slope = (self.ROBOTIQ_OPEN - self.ROBOTIQ_CLOSE) / (self.ENCODER_OPEN - self.ENCODER_CLOSE)
                    gripper_target_cmd = self.ROBOTIQ_CLOSE + slope * (bounded_pos - self.ENCODER_CLOSE)
                    self.arm_backend.send_gripper_goal(gripper_target_cmd)
                else:
                    native_master_velocities[i] = float(speed) * self.RAD_PER_UNIT * self.JOINT_SIGNS[i]
                    self.raw_master_rads[i] = (pos - self.SERVO_CENTER) * self.RAD_PER_UNIT * self.JOINT_SIGNS[i]
            else:
                if s_id != 8:
                    self.raw_master_rads[i] = self.prev_master_rads[i]

        # Publish Master Device State for Logging & Visualization
        master_msg = JointState()
        master_msg.header.stamp = self.get_clock().now().to_msg()
        master_msg.name = self.ARM_JOINT_NAMES
        master_msg.position = list(self.raw_master_rads)
        master_msg.velocity = list(native_master_velocities)
        self.master_state_publisher.publish(master_msg)

        # --- 2. POSITION MIRRORING CALCULATION ---
        target_q = [0.0] * len(self.ARM_JOINT_NAMES)

        for j in range(len(self.ARM_JOINT_NAMES)):
            raw_master_pos = self.raw_master_rads[j]
            curr_slave_pos = self.current_slave_positions[j]

            if j in self.CONTINUOUS_JOINTS:
                shortest_err = self.get_shortest_angle_error(raw_master_pos, curr_slave_pos)
                joint_target = curr_slave_pos + shortest_err
            else:
                joint_target = raw_master_pos

            target_q[j] = joint_target

        self.prev_master_rads = list(self.raw_master_rads)

        # --- 3. STREAM DIRECT POSITION TRAJECTORY POINT ---
        self.arm_backend.send_teleop_trajectory_point(target_q, lookahead_sec=0.06)

def main(args=None):
    rclpy.init(args=args)
    node = HarvestTeleopCoreNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.arm_backend.shutdown_procedure()
        if node.is_hardware_connected:
            for s_id in node.SERVO_IDS:
                node.packet_handler.write1ByteTxRx(s_id, 40, 0)
        node.port_handler.closePort()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()