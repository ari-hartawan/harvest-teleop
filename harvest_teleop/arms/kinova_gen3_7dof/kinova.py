#!/usr/bin/env python3
"""Kinova Gen3 Handshake and Action Server Communication Plugin Module (Direct Position Trajectory Mode)."""

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
        # Joint trajectory controller publisher
        self.arm_trajectory_publisher = self.ctx.create_publisher(
            JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10
        )
        self.gripper_action_client = ActionClient(
            self.ctx, ParallelGripperCommand, 'robotiq_gripper_controller/gripper_cmd'
        )

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
        arm_msg.joint_names = self.ctx.ARM_JOINT_NAMES
        
        safe_home_positions = []
        for i in range(len(self.ctx.ARM_JOINT_NAMES)):
            current_pos = self.ctx.current_slave_positions[i]
            target_home = self.ctx.HOME_RADS[i]
            shortest_err = self.ctx.get_shortest_angle_error(target_home, current_pos)
            safe_home_positions.append(current_pos + shortest_err)

        arm_point = JointTrajectoryPoint()
        arm_point.positions = safe_home_positions
        arm_point.time_from_start = Duration(sec=2, nanosec=0)
        arm_msg.points.append(arm_point)
        
        self.arm_trajectory_publisher.publish(arm_msg)
        self.send_gripper_goal(0.5)

    def switch_controllers(self):
        """
        In Direct Trajectory Mode, joint_trajectory_controller stays active continuously.
        No ros2 control switch_controllers subprocess call is needed.
        """
        self.ctx.get_logger().info("Direct Trajectory Mode: Skipping controller switch (joint_trajectory_controller stays active).")
        return True

    def send_teleop_trajectory_point(self, target_positions, target_velocities=None, lookahead_sec=0.06):
        """Publishes streaming position trajectory points to joint_trajectory_controller.
        Note: velocities are left empty by default to prevent JTC from rejecting non-zero end-velocity points.
        """
        arm_msg = JointTrajectory()
        arm_msg.header.stamp = self.ctx.get_clock().now().to_msg()
        arm_msg.joint_names = self.ctx.ARM_JOINT_NAMES

        arm_point = JointTrajectoryPoint()
        arm_point.positions = [float(p) for p in target_positions]
        
        # Only attach velocities if explicitly enabled and non-empty
        if target_velocities is not None and len(target_velocities) == len(target_positions) and getattr(self.ctx, 'send_velocities_in_traj', False):
            arm_point.velocities = [float(v) for v in target_velocities]
        
        arm_point.time_from_start = Duration(sec=0, nanosec=int(lookahead_sec * 1e9))
        arm_msg.points.append(arm_point)

        self.arm_trajectory_publisher.publish(arm_msg)

    def move_master_to_home(self):
        """Moves the master device to home coordinates taking the absolute shortest path."""
        for i, s_id in enumerate(self.ctx.SERVO_IDS[:7]):
            pos, _, res, _ = self.ctx.packet_handler.ReadPosSpeed(s_id)
            
            if res == COMM_SUCCESS:
                current_rad = (pos - self.ctx.SERVO_CENTER) * self.ctx.RAD_PER_UNIT * self.ctx.JOINT_SIGNS[i]
                target_home_rad = self.ctx.HOME_RADS[i]
                
                shortest_err = self.ctx.get_shortest_angle_error(target_home_rad, current_rad)
                safe_target_rad = current_rad + shortest_err
                
                raw_target_encoder = self.ctx.SERVO_CENTER + int((safe_target_rad * self.ctx.JOINT_SIGNS[i]) / self.ctx.RAD_PER_UNIT)
                target_encoder = max(0, min(raw_target_encoder, 4095))
            else:
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
        
        time.sleep(1.5)

    def home_both_master_and_slave(self):
        """Homes both the master device and the robot slave concurrently to HOME_RADS."""
        self.ctx.get_logger().info("Homing both Master and Slave arms to HOME_RADS...")
        self.send_robot_to_home()
        self.move_master_to_home()
        time.sleep(0.5)
        self.ctx.get_logger().info("Master and Slave arms homing complete!")

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
        """Safely locks the robot in current position by sending zero velocity trajectory point."""
        try:
            if hasattr(self.ctx, 'current_slave_positions') and len(self.ctx.current_slave_positions) == len(self.ctx.ARM_JOINT_NAMES):
                hold_positions = list(self.ctx.current_slave_positions)
                self.send_teleop_trajectory_point(hold_positions, lookahead_sec=0.1)
        except Exception:
            pass