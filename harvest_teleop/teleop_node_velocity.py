#!/usr/bin/env python3
"""
HARVEST Framework: Hand-admittance and Robotic Velocity-driven System for Low-Cost Teleoperation
Author: Ari Hartawan
Description: Multi-arm compatible core node. Purely parameter-driven via external 
             YAML configuration files without hardcoded default values.
             Updated with safe controller switching on stop to prevent arm sagging.
"""

import os
import sys
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
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
        
        self.get_logger().info("HARVEST Framework: Loading parameters dynamically from external YAML...")

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
        
        self.VELOCITY_SMOOTHING_FACTOR = self.get_parameter('control.velocity_smoothing_factor').value
        self.MAX_CORRECTION_VEL = self.get_parameter('control.max_correction_velocity').value
        self.KP_POSITION_CORRECTOR = self.get_parameter('control.kp_position_corrector').value
        self.MAX_JOINT_VELOCITIES_DEG = self.get_parameter('control.max_joint_velocities_deg').value
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
        self.current_filtered_vel = [0.0] * len(self.ARM_JOINT_NAMES)
        self.current_slave_positions = [0.0] * len(self.ARM_JOINT_NAMES)
        
        self.safety_state = 0
        self.is_hardware_connected = False
        
        # ROS 2 Interfaces
        # 1. Publish Calculated Velocities for Slave Robot
        self.velocity_publisher = self.create_publisher(Float64MultiArray, 'joint_group_velocity_controller/commands', 10)
        # 2. Publish Master Device State
        self.master_state_publisher = self.create_publisher(JointState, 'master_joint_states', 10)
        # 3. Subscribe to Robot Slave States
        self.joint_state_subscriber = self.create_subscription(JointState, 'joint_states', self.joint_state_callback, 10)
        
        # External Safety UI Trigger Subscriber (Global Topic)
        self.trigger_subscriber = self.create_subscription(String, '/harvest_safety_trigger', self.safety_trigger_callback, 10)
        
        # Dynamic Backend Selection
        if self.BACKEND_TYPE == "kinova":
            from .arms.kinova_gen3_7dof.kinova import KinovaBackend
            self.arm_backend = KinovaBackend(self)
        else:
            self.get_logger().error(f"Selected backend '{self.BACKEND_TYPE}' is unsupported.")
            sys.exit(1)

        self.initialize_hardware()
        
        # Trigger control loop
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
            print(" -> Run 'keyboard_node' or 'harvest_gui_logger' to send safety triggers.")
            print("="*60)

    def get_shortest_angle_error(self, target_rad, current_rad):
        error = target_rad - current_rad
        return math.atan2(math.sin(error), math.cos(error))

    def safety_trigger_callback(self, msg):
        command = msg.data.strip()
        
        # --- STEP 1: HOME MASTER ---
        if command in ['STEP_1', '0']:
            if self.safety_state == 0:
                self.get_logger().info("Triggering STEP 1: Master Homing...")
                self.safety_state = 1
                self.arm_backend.move_master_to_home()

        # --- STEP 2: HOME ROBOT SLAVE ---
        elif command == 'STEP_2':
            if self.safety_state == 1:
                self.get_logger().info("Triggering STEP 2: Robot Slave Homing...")
                self.safety_state = 2
                self.arm_backend.send_robot_to_home()

        # --- STEP 3: SWITCH CONTROLLER & DIRECTLY ENGAGE LIVE TELEOP (UNRECORDED) ---
        elif command == 'STEP_3':
            if self.safety_state == 2:
                self.get_logger().info("Triggering STEP 3: Transitioning Controllers & ENABLING LIVE TELEOP...")
                if self.arm_backend.switch_controllers():
                    # Set Soft Torque Limit for Master
                    for s_id in self.SERVO_IDS:
                        self.packet_handler.write1ByteTxRx(s_id, 40, 1)
                        self.packet_handler.write2ByteTxRx(s_id, 48, self.LIVE_TORQUE_LIMIT)
                    
                    # 🚀 ENGAGE LIVE TELEOP DIRECTLY (State 3 = Live Teleop Active, Logging Off)
                    self.safety_state = 3
                    self.get_logger().info("🕹️ LIVE TELEOP IS NOW ACTIVE! (Waiting for SPACE to start logging)")
                else:
                    self.get_logger().error("Controller switch failed!")

        # --- STEP 4: START HDF5 LOGGING (SEAMLESS TRANSITION) ---
        elif command == 'START_RECORDING':
            if self.safety_state == 3:
                self.get_logger().info("Triggering STEP 4: START HDF5 LOGGING DATASET! 🔴")
                self.safety_state = 4  # State 4 = Live Teleop Active + Logging On

        # --- STEP 5: STOP RECORDING, SAVE HDF5, & LOCK ROBOT ---
        elif command in ['STOP_RECORDING', 'CANCEL_RECORDING']:
            self.get_logger().info("Triggering STEP 5: Restoring Trajectory Controller & Stiff Torque...")
            
            # Stop velocity movement immediately
            zero_msg = Float64MultiArray()
            zero_msg.data = [0.0] * len(self.ARM_JOINT_NAMES)
            self.velocity_publisher.publish(zero_msg)

            # Lock Master Device
            for s_id in self.SERVO_IDS:
                self.packet_handler.write1ByteTxRx(s_id, 40, 1)
                self.packet_handler.write2ByteTxRx(s_id, 48, self.STIFF_TORQUE_LIMIT)

            # Switch back to Trajectory Controller to lock robot position safely
            traj_controller = "joint_trajectory_controller"
            vel_controller = "joint_group_velocity_controller"
            cm_path = self.arm_backend.get_active_controller_manager()
            
            cmd = [
                "ros2", "control", "switch_controllers",
                "--controller-manager", cm_path,
                "--deactivate", vel_controller,
                "--activate", traj_controller,
                "--strict"
            ]
            try:
                import subprocess
                subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
                self.get_logger().info(f"SUCCESS: Trajectory controller restored. Robot locked safely!")
            except Exception as e:
                self.get_logger().warn(f"Note on restoring trajectory controller: {e}")

            self.safety_state = 0
            self.show_menu_prompt()

    def teleop_callback(self):
        # 🔑 Only allow velocity loop to execute if in State 3 or State 4!
        if self.safety_state < 3:
            return

        if not hasattr(self, '_debug_tick'):
            self._debug_tick = 0
        self._debug_tick += 1

        start_time = self.get_clock().now()

        native_master_velocities = [0.0] * len(self.ARM_JOINT_NAMES)
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
                    if i == 0 and self._debug_tick % 50 == 0:
                        self.get_logger().info(f"DEBUG: Joint 0 - Raw Speed Reg: {speed}, Calc Vel: {native_master_velocities[i]:.4f}, Pos: {self.raw_master_rads[i]:.4f}")
            else:
                if s_id != 8:
                    self.raw_master_rads[i] = self.prev_master_rads[i]

        master_msg = JointState()
        master_msg.header.stamp = self.get_clock().now().to_msg()
        master_msg.name = self.ARM_JOINT_NAMES
        master_msg.position = list(self.raw_master_rads)
        master_msg.velocity = list(native_master_velocities)
        self.master_state_publisher.publish(master_msg)

        final_commands = [0.0] * len(self.ARM_JOINT_NAMES)
        for j in range(len(self.ARM_JOINT_NAMES)):
            raw_velocity = native_master_velocities[j]
            self.current_filtered_vel[j] = (self.VELOCITY_SMOOTHING_FACTOR * raw_velocity) + \
                                           ((1.0 - self.VELOCITY_SMOOTHING_FACTOR) * self.current_filtered_vel[j])
            position_error = self.raw_master_rads[j] - self.current_slave_positions[j]
            if j in self.CONTINUOUS_JOINTS:
                position_error = math.atan2(math.sin(position_error), math.cos(position_error))
            
            # P controller active again for troubleshooting
            correction_vel = self.KP_POSITION_CORRECTOR[j] * position_error
            correction_vel = max(-self.MAX_CORRECTION_VEL, min(correction_vel, self.MAX_CORRECTION_VEL))
            raw_final_cmd = self.current_filtered_vel[j] + correction_vel

            if abs(raw_final_cmd) < 0.015:
                raw_final_cmd = 0.0
            
            joint_limit_rad = math.radians(self.MAX_JOINT_VELOCITIES_DEG[j])
            final_commands[j] = max(-joint_limit_rad, min(raw_final_cmd, joint_limit_rad))

        self.prev_master_rads = list(self.raw_master_rads)
        vel_msg = Float64MultiArray()
        vel_msg.data = final_commands
        self.velocity_publisher.publish(vel_msg)

        duration = (self.get_clock().now() - start_time).nanoseconds / 1e6
        if self._debug_tick % 50 == 0:
            self.get_logger().info(f"DEBUG: teleop_callback took {duration:.2f} ms")

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
