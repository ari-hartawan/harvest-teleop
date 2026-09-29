#!/usr/bin/env python3
"""Kinova Gen3 Handshake and Action Server Communication Plugin Module."""

import time
import subprocess
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import ParallelGripperCommand
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from hardware.stservo.scservo_sdk.scservo_def import COMM_SUCCESS

class KinovaBackend:
    def __init__(self, node_context):
        self.ctx = node_context
        # FIX: Remove leading '/' to respect active Namespace (right_ or left_)
        self.arm_trajectory_publisher = self.ctx.create_publisher(JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10)
        self.gripper_action_client = ActionClient(self.ctx, ParallelGripperCommand, 'robotiq_gripper_controller/gripper_cmd')

    def get_active_controller_manager(self):
        """Gets the controller manager path dynamically based on the active namespace."""
        ns = self.ctx.get_namespace().strip('/')
        if ns:
            return f"/{ns}/controller_manager"
        return "/controller_manager"

    def send_robot_to_home(self):
        """Sends the Kinova robot to home position using the safest shortest-path calculation."""
        arm_msg = JointTrajectory()
        arm_msg.header.stamp = self.ctx.get_clock().now().to_msg()
        
        # Use ARM_JOINT_NAMES which already contains the correct prefixes automatically.
        arm_msg.joint_names = self.ctx.ARM_JOINT_NAMES
        
        # Calculate safest shortest path to target Home to prevent unpredicted massive loops
        safe_home_positions = []
        for i in range(len(self.ctx.ARM_JOINT_NAMES)):
            current_pos = self.ctx.current_slave_positions[i]
            target_home = self.ctx.HOME_RADS[i]
            
            # Use the core's atan2 helper to calculate shortest path
            shortest_err = self.ctx.get_shortest_angle_error(target_home, current_pos)
            safe_home_positions.append(current_pos + shortest_err)

        arm_point = JointTrajectoryPoint()
        arm_point.positions = safe_home_positions
        arm_point.time_from_start = Duration(sec=2, nanosec=0)
        arm_msg.points.append(arm_point)
        
        self.arm_trajectory_publisher.publish(arm_msg)
        self.send_gripper_goal(0.5)

    def switch_controllers(self):
        # Use standard controller names without additional prefixes
        traj_controller = "joint_trajectory_controller"
        vel_controller = "joint_group_velocity_controller"
        
        # Call the controller manager corresponding to the active namespace dynamically (e.g., /right_/controller_manager)
        cm_path = self.get_active_controller_manager()
        cmd = [
            "ros2", "control", "switch_controllers", 
            "--controller-manager", cm_path,
            "--deactivate", traj_controller, 
            "--activate", vel_controller
        ]
        try:
            subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
            return True
        except subprocess.CalledProcessError:
            return False

    def move_master_to_home(self):
        """Moves the master device to home coordinates taking the absolute shortest path to prevent cable stress."""
        for i, s_id in enumerate(self.ctx.SERVO_IDS[:7]):
            pos, _, res, _ = self.ctx.packet_handler.ReadPosSpeed(s_id)
            
            if res == COMM_SUCCESS:
                # Convert feedback to actual joint physical radians
                current_rad = (pos - self.ctx.SERVO_CENTER) * self.ctx.RAD_PER_UNIT * self.ctx.JOINT_SIGNS[i]
                target_home_rad = self.ctx.HOME_RADS[i]
                
                # Compute shortest trajectory delta to eliminate wrapping/spinning loops
                shortest_err = self.ctx.get_shortest_angle_error(target_home_rad, current_rad)
                safe_target_rad = current_rad + shortest_err
                
                # Re-convert back to safe encoder coordinates (clipping boundaries for hardware protection)
                raw_target_encoder = self.ctx.SERVO_CENTER + int((safe_target_rad * self.ctx.JOINT_SIGNS[i]) / self.ctx.RAD_PER_UNIT)
                target_encoder = max(0, min(raw_target_encoder, 4095))
            else:
                # Fallback to absolute parameter target if serial query fails
                target_encoder = self.ctx.SERVO_CENTER + int((self.ctx.HOME_RADS[i] * self.ctx.JOINT_SIGNS[i]) / self.ctx.RAD_PER_UNIT)

            self.ctx.virtual_targets[s_id] = target_encoder
            self.ctx.packet_handler.write2ByteTxRx(s_id, 48, self.ctx.STIFF_TORQUE_LIMIT)
            self.ctx.packet_handler.WritePosEx(s_id, int(target_encoder), 1500, 50)
            
        target_robotiq = 0.5
        slope = (self.ctx.ROBOTIQ_OPEN - self.ctx.ROBOTIQ_CLOSE) / (self.ctx.ENCODER_OPEN - self.ctx.ENCODER_CLOSE)
        home_encoder_gripper = int(self.ctx.ENCODER_CLOSE + (target_robotiq - self.ctx.ROBOTIQ_CLOSE) / slope)

        self.ctx.virtual_targets[8] = home_encoder_gripper
        self.ctx.packet_handler.write2ByteTxRx(8, 48, self.ctx.STIFF_TORQUE_LIMIT)
        self.ctx.packet_handler.WritePosEx(8, home_encoder_gripper, 1500, 50)
        
        # Fast master device homing delay (1.5 seconds)
        time.sleep(1.5)

    def send_gripper_goal(self, target_position):
        if not self.gripper_action_client.wait_for_server(timeout_sec=0.001):
            return
            
        ns = self.ctx.get_namespace().strip('/')
        clean_ns = ns.rstrip('_')
        prefix = f"{clean_ns}_" if clean_ns else ""
        joint_name = f"{prefix}robotiq_85_left_knuckle_joint"

        goal_msg = ParallelGripperCommand.Goal()
        goal_msg.command.name = [joint_name]
        goal_msg.command.position = [float(target_position)]
        goal_msg.command.effort = [100.0]
        self.gripper_action_client.send_goal_async(goal_msg)

    def shutdown_procedure(self):
        from std_msgs.msg import Float64MultiArray
        try:
            zero_msg = Float64MultiArray()
            zero_msg.data = [0.0] * len(self.ctx.ARM_JOINT_NAMES)
            self.ctx.velocity_publisher.publish(zero_msg)
        except Exception:
            pass
            
        # Use standard controller names without additional prefixes
        traj_controller = "joint_trajectory_controller"
        vel_controller = "joint_group_velocity_controller"
        
        # Call the controller manager corresponding to the active namespace dynamically (e.g., /right_/controller_manager)
        cm_path = self.get_active_controller_manager()
        cmd = [
            "ros2", "control", "switch_controllers",
            "--controller-manager", cm_path,
            "--deactivate", vel_controller,
            "--activate", traj_controller
        ]
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)