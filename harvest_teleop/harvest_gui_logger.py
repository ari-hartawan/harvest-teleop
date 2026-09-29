#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
HARVEST Framework: Async Direct-Pull Data Collector & GUI Logger (IR Grayscale Clean)
- RealSense D435i uses Infrared Stream (Y8 1-channel) to save USB bandwidth.
- IR Laser Emitter is disabled to keep the image clean without laser dots.
- Automatically duplicated to 3-channel Grayscale in memory so HDF5 ACT format (480, 640, 3) remains unchanged.
- Ensenso N46 runs independently in its own background thread.
- RealSense & Joint State acquisition runs consistently at a target of 10 FPS.
"""

import sys
import os
import time
import threading
from collections import deque
import numpy as np
import cv2
try:
    import pyrealsense2 as rs
    REALSENSE_AVAILABLE = True
except ImportError:
    REALSENSE_AVAILABLE = False
import h5py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

try:
    from nxlib import NxLib, NxLibException, NxLibCommand, NxLibItem
    from nxlib.constants import *
    ENSENSO_AVAILABLE = True
except ImportError:
    ENSENSO_AVAILABLE = False

os.environ['QT_LOGGING_RULES'] = '*.debug=false;*.warning=false'

CAMERA_SERIAL_N46 = "260143"  
SN_LEFT = "233722071722"
SN_RIGHT = "233522076188"


class DirectHarvestNode(Node):
    def __init__(self):
        super().__init__('harvest_direct_logger_node')
        
        self.declare_parameter('dataset_dir', '~/harvest_dataset')
        self.declare_parameter('record_fps', 10.0)
        self.declare_parameter('webcam_device_id', 'auto')
        
        self.dataset_dir = os.path.expanduser(self.get_parameter('dataset_dir').value)
        os.makedirs(self.dataset_dir, exist_ok=True)
        self.target_fps = self.get_parameter('record_fps').value
        self.target_dt = 1.0 / self.target_fps

        self.episode_idx = self.get_auto_index(self.dataset_dir)
        
        self.is_recording = False
        self.is_saving = False
        self.save_success = False
        self.save_success_timer = 0.0
        self.last_saved_filename = ""
        self.last_saved_size_mb = 0.0

        self.latest_left_qpos = [0.0] * 7
        self.latest_left_qvel = [0.0] * 7
        self.latest_left_gripper = 0.0
        self.latest_right_qpos = [0.0] * 7
        self.latest_right_qvel = [0.0] * 7
        self.latest_right_gripper = 0.0

        # We do not use unwrapping in the logger anymore (reverted to RAW)
        # unwrap_hdf5.py will handle offline dataset fixes.

        # Current Frame Buffer (3-Channel for ACT compatibility)
        self.current_top_img = np.zeros((480, 640, 3), dtype=np.uint8)
        self.current_left_img = np.zeros((480, 640, 3), dtype=np.uint8)
        self.current_right_img = np.zeros((480, 640, 3), dtype=np.uint8)
        self.current_front_img = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Lock for Ensenso & Webcam Thread Safety
        self.ensenso_lock = threading.Lock()
        self.webcam_lock = threading.Lock()

        self.reset_episode_buffer()

        self.create_subscription(JointState, '/left_/joint_states', self.left_joint_callback, 10)
        self.create_subscription(JointState, '/right_/joint_states', self.right_joint_callback, 10)
        self.create_subscription(String, '/harvest_safety_trigger', self.safety_trigger_callback, 10)

        self.init_cameras()

        self.running = True

        # Ensenso Dedicated Thread
        if ENSENSO_AVAILABLE and self.ensenso_node is not None:
            self.ensenso_thread = threading.Thread(target=self.ensenso_capture_loop, daemon=True)
            self.ensenso_thread.start()

        # Front Webcam Dedicated Thread
        if hasattr(self, 'cap_front') and self.cap_front is not None and self.cap_front.isOpened():
            self.webcam_thread = threading.Thread(target=self.webcam_capture_loop, daemon=True)
            self.webcam_thread.start()

        # Main Logging Loop Thread (10 Hz)
        self.capture_thread = threading.Thread(target=self.acquisition_loop, daemon=True)
        self.capture_thread.start()

        self.create_timer(0.033, self.gui_render_callback)

        self.get_logger().info("=" * 60)
        self.get_logger().info(f" HARVEST IR-Clean Logger Active | Target: {self.target_fps} Hz")
        self.get_logger().info("=" * 60)

    def get_auto_index(self, dataset_dir):
        for i in range(2000):
            if not os.path.isfile(os.path.join(dataset_dir, f"episode_{i}.hdf5")):
                return i
        return 0

    def reset_episode_buffer(self):
        self.episode_frames = []
        self.episode_qpos = []
        self.episode_qvel = []
        self.episode_action = []
        self.dt_history = []
        self.last_loop_time = None

    def safety_trigger_callback(self, msg):
        cmd = msg.data.strip()
        if cmd == 'START_RECORDING' and not self.is_saving:
            self.start_recording()
        elif cmd == 'STOP_RECORDING':
            self.stop_and_save_recording()
        elif cmd == 'CANCEL_RECORDING':
            self.is_recording = False
            self.is_saving = False
            self.reset_episode_buffer()
            self.get_logger().warn("🗑️ [RECORDING CANCELLED] Episode buffer cleared.")

    def parse_joint_msg(self, msg, is_right=True):
        temp_qpos = [0.0] * 7
        temp_qvel = [0.0] * 7
        gripper_val = 0.0
        for idx, (name, pos) in enumerate(zip(msg.name, msg.position)):
            vel = msg.velocity[idx] if len(msg.velocity) > idx else 0.0
            for i in range(1, 8):
                if f'_joint_{i}' in name:
                    temp_qpos[i - 1] = float(pos)
                    temp_qvel[i - 1] = float(vel)
                    break
            if 'knuckle_joint' in name:
                gripper_val = float(pos)
        
        if is_right:
            self.latest_right_qpos = temp_qpos
            self.latest_right_qvel = temp_qvel
            self.latest_right_gripper = gripper_val
        else:
            self.latest_left_qpos = temp_qpos
            self.latest_left_qvel = temp_qvel
            self.latest_left_gripper = gripper_val

    def left_joint_callback(self, msg): self.parse_joint_msg(msg, is_right=False)
    def right_joint_callback(self, msg): self.parse_joint_msg(msg, is_right=True)

    def find_front_webcam(self):
        v4l_dir = '/dev/v4l/by-id'
        if os.path.exists(v4l_dir):
            for filename in os.listdir(v4l_dir):
                if 'index0' in filename and ('Brio' in filename or 'Logitech' in filename or ('Intel' not in filename and 'RealSense' not in filename)):
                    return os.path.join(v4l_dir, filename)
        return None

    def init_cameras(self):
        if not REALSENSE_AVAILABLE:
            self.rs_left = None
            self.rs_right = None
            self.get_logger().warn("⚠️ pyrealsense2 module not installed; RealSense cameras disabled.")
        else:
            # --- REALSENSE LEFT (INFRARED + EMITTER OFF) ---
            self.rs_left = rs.pipeline()
            cfg_l = rs.config()
            cfg_l.enable_device(SN_LEFT)
            cfg_l.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
            try:
                profile_l = self.rs_left.start(cfg_l)
                depth_sensor_l = profile_l.get_device().first_depth_sensor()
                if depth_sensor_l.supports(rs.option.emitter_enabled):
                    depth_sensor_l.set_option(rs.option.emitter_enabled, 0)  # Disable laser dots
                self.get_logger().info(f"✅ RealSense Left ({SN_LEFT}) Online [IR Clean Grayscale].")
            except Exception as e:
                self.rs_left = None
                self.get_logger().error(f"❌ RealSense Left Failed: {e}")

            # --- REALSENSE RIGHT (INFRARED + EMITTER OFF) ---
            self.rs_right = rs.pipeline()
            cfg_r = rs.config()
            cfg_r.enable_device(SN_RIGHT)
            cfg_r.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
            try:
                profile_r = self.rs_right.start(cfg_r)
                depth_sensor_r = profile_r.get_device().first_depth_sensor()
                if depth_sensor_r.supports(rs.option.emitter_enabled):
                    depth_sensor_r.set_option(rs.option.emitter_enabled, 0)  # Disable laser dots
                self.get_logger().info(f"✅ RealSense Right ({SN_RIGHT}) Online [IR Clean Grayscale].")
            except Exception as e:
                self.rs_right = None
                self.get_logger().error(f"❌ RealSense Right Failed: {e}")

        self.ensenso_node = None
        if ENSENSO_AVAILABLE:
            try:
                self.nx_ctx = NxLib()
                root = NxLibItem()
                cam_node = root[ITM_CAMERAS][CAMERA_SERIAL_N46]
                if cam_node.exists():
                    try:
                        with NxLibCommand(CMD_OPEN) as cmd:
                            cmd.parameters()[ITM_CAMERAS] = CAMERA_SERIAL_N46
                            cmd.execute()
                    except Exception:
                        pass
                    
                    cap = cam_node[ITM_PARAMETERS][ITM_CAPTURE]
                    cap[ITM_MODE] = "Rectified"
                    cap["TriggerMode"] = "Software"
                    cap["Projector"] = False
                    self.ensenso_node = cam_node
                    self.get_logger().info(f"✅ Ensenso N46 ({CAMERA_SERIAL_N46}) Connected.")
            except Exception as e:
                self.get_logger().error(f"❌ Ensenso Init Failed: {e}")

        # --- WEBCAM FRONT (FRONT VIEW) ---
        webcam_id_param = self.get_parameter('webcam_device_id').value
        if isinstance(webcam_id_param, str) and webcam_id_param.lower() in ['auto', '']:
            auto_path = self.find_front_webcam()
            if auto_path:
                self.get_logger().info(f"🔍 Auto-detected front webcam path: {auto_path}")
                self.webcam_device_id = auto_path
            else:
                self.get_logger().warn("⚠️ Could not auto-detect front webcam. Falling back to default index 12.")
                self.webcam_device_id = 12
        else:
            try:
                self.webcam_device_id = int(webcam_id_param)
            except (ValueError, TypeError):
                self.webcam_device_id = webcam_id_param

        self.cap_front = cv2.VideoCapture(self.webcam_device_id)
        if self.cap_front.isOpened():
            self.get_logger().info(f"✅ Webcam Front (Device {self.webcam_device_id}) Online.")
            self.cap_front.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap_front.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        else:
            self.cap_front = None
            self.get_logger().error(f"❌ Webcam Front (Device {self.webcam_device_id}) Failed to Open.")

    def webcam_capture_loop(self):
        while self.running and rclpy.ok():
            if self.cap_front is not None and self.cap_front.isOpened():
                ret, frame = self.cap_front.read()
                if ret and frame is not None:
                    if frame.shape != (480, 640, 3):
                        frame = cv2.resize(frame, (640, 480))
                    with self.webcam_lock:
                        self.current_front_img = frame
            time.sleep(0.01)

    def ensenso_capture_loop(self):
        while self.running and rclpy.ok():
            try:
                with NxLibCommand(CMD_CAPTURE) as cmd:
                    cmd.parameters()[ITM_CAMERAS] = CAMERA_SERIAL_N46
                    cmd.execute()
                with NxLibCommand(CMD_RECTIFY_IMAGES) as cmd:
                    cmd.execute()

                tex_node = self.ensenso_node[ITM_IMAGES][ITM_RECTIFIED_TEXTURE][ITM_LEFT]
                if not tex_node.exists() or tex_node.count() == 0:
                    tex_node = self.ensenso_node[ITM_IMAGES][ITM_RAW][ITM_LEFT]
                
                if tex_node.exists():
                    raw_img = tex_node.get_binary_data()
                    if len(raw_img.shape) == 2 or raw_img.shape[2] == 1:
                        img_3ch = cv2.cvtColor(raw_img, cv2.COLOR_GRAY2BGR)
                    else:
                        img_3ch = raw_img[:, :, :3].copy()
                    
                    if img_3ch.dtype != np.uint8:
                        img_3ch = img_3ch.astype(np.uint8)
                    
                    frame_resized = cv2.resize(img_3ch, (640, 480))
                    
                    with self.ensenso_lock:
                        self.current_top_img = frame_resized
            except Exception:
                pass
            time.sleep(0.01)

    def pull_realsense_frame(self, pipeline):
        if not pipeline:
            return np.zeros((480, 640, 3), dtype=np.uint8)
        try:
            frames = pipeline.poll_for_frames()
            if frames:
                ir_frame = frames.get_infrared_frame(1)
                if ir_frame:
                    gray = np.asanyarray(ir_frame.get_data())
                    if gray is not None and gray.size > 0:
                        bgr_3ch = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                        return cv2.resize(bgr_3ch, (640, 480))
        except Exception:
            pass
        return np.zeros((480, 640, 3), dtype=np.uint8)

    def start_recording(self):
        self.reset_episode_buffer()
        self.save_success = False
        self.is_recording = True
        self.last_loop_time = time.time()
        self.get_logger().info(f"🔴 [RECORDING STARTED] Episode #{self.episode_idx}")

    def acquisition_loop(self):
        while self.running and rclpy.ok():
            loop_start = time.time()

            rs_l_img = self.pull_realsense_frame(self.rs_left)
            rs_r_img = self.pull_realsense_frame(self.rs_right)

            if rs_l_img is not None and rs_l_img.shape == (480, 640, 3): 
                self.current_left_img = rs_l_img
            if rs_r_img is not None and rs_r_img.shape == (480, 640, 3): 
                self.current_right_img = rs_r_img

            with self.ensenso_lock:
                top_img = self.current_top_img.copy()

            with self.webcam_lock:
                front_img = self.current_front_img.copy()

            if self.is_recording and not self.is_saving:
                t0 = time.time()
                if self.last_loop_time is not None:
                    self.dt_history.append(t0 - self.last_loop_time)
                self.last_loop_time = t0

                l_qpos = list(self.latest_left_qpos)
                l_qvel = list(self.latest_left_qvel)
                l_grip = self.latest_left_gripper
                
                r_qpos = list(self.latest_right_qpos)
                r_qvel = list(self.latest_right_qvel)
                r_grip = self.latest_right_gripper

                qpos_16dof = l_qpos + [l_grip] + r_qpos + [r_grip]
                qvel_16dof = l_qvel + [0.0] + r_qvel + [0.0]
                action_16dof = qpos_16dof.copy()

                self.episode_qpos.append(qpos_16dof)
                self.episode_qvel.append(qvel_16dof)
                self.episode_action.append(action_16dof)

                self.episode_frames.append({
                    'top': top_img,
                    'left': self.current_left_img.copy(),
                    'right': self.current_right_img.copy(),
                    'front': front_img
                })

            elapsed = time.time() - loop_start
            time.sleep(max(0.0, self.target_dt - elapsed))

    def stop_and_save_recording(self):
        if not self.is_recording: return
        self.is_recording = False
        self.is_saving = True
        self.save_success = False

        total_frames = len(self.episode_qpos)
        if total_frames == 0:
            self.get_logger().warn("⚠️ No frames recorded!")
            self.is_saving = False
            return

        dataset_name = f"episode_{self.episode_idx}.hdf5"
        filename = os.path.join(self.dataset_dir, dataset_name)

        try:
            t_start = time.time()
            self.get_logger().info(f"⏳ Batch-writing {total_frames} frames to HDF5...")

            cam_top_arr = np.zeros((total_frames, 480, 640, 3), dtype=np.uint8)
            cam_left_arr = np.zeros((total_frames, 480, 640, 3), dtype=np.uint8)
            cam_right_arr = np.zeros((total_frames, 480, 640, 3), dtype=np.uint8)
            cam_front_arr = np.zeros((total_frames, 480, 640, 3), dtype=np.uint8)

            for idx, frame_dict in enumerate(self.episode_frames):
                cam_top_arr[idx] = frame_dict['top']
                cam_left_arr[idx] = frame_dict['left']
                cam_right_arr[idx] = frame_dict['right']
                cam_front_arr[idx] = frame_dict.get('front', np.zeros((480, 640, 3), dtype=np.uint8))

            qpos_arr = np.array(self.episode_qpos, dtype=np.float64)
            qvel_arr = np.array(self.episode_qvel, dtype=np.float64)
            action_arr = np.array(self.episode_action, dtype=np.float64)

            with h5py.File(filename, 'w') as root:
                root.attrs['sim'] = False
                obs = root.create_group('observations')
                image = root.create_group('images')
                
                image.create_dataset('cam_top', data=cam_top_arr, chunks=(1, 480, 640, 3), compression="gzip", compression_opts=4)
                image.create_dataset('cam_left_wrist', data=cam_left_arr, chunks=(1, 480, 640, 3), compression="gzip", compression_opts=4)
                image.create_dataset('cam_right_wrist', data=cam_right_arr, chunks=(1, 480, 640, 3), compression="gzip", compression_opts=4)
                image.create_dataset('cam_front', data=cam_front_arr, chunks=(1, 480, 640, 3), compression="gzip", compression_opts=4)
                
                obs.create_dataset('qpos', data=qpos_arr)
                obs.create_dataset('qvel', data=qvel_arr)
                root.create_dataset('action', data=action_arr)

            self.last_saved_filename = dataset_name
            self.last_saved_size_mb = os.path.getsize(filename) / (1024 * 1024)
            self.get_logger().info(f"✅ {dataset_name} Successfully Saved! ({self.last_saved_size_mb:.1f} MB)")

            self.save_success = True
            self.save_success_timer = time.time()
            self.episode_idx = self.get_auto_index(self.dataset_dir)
        except Exception as e:
            self.get_logger().error(f"❌ Failed to save HDF5: {e}")
        finally:
            self.is_saving = False

    def gui_render_callback(self):
        dashboard = np.zeros((800, 1280, 3), dtype=np.uint8)

        disp_left = cv2.resize(self.current_left_img, (426, 320))
        disp_top = cv2.resize(self.current_top_img, (428, 320))
        disp_right = cv2.resize(self.current_right_img, (426, 320))

        with self.webcam_lock:
            front_img = self.current_front_img.copy()
        disp_front = cv2.resize(front_img, (426, 320))

        dashboard[0:320, 0:426] = disp_left
        dashboard[0:320, 426:854] = disp_top
        dashboard[0:320, 854:1280] = disp_right

        cv2.putText(dashboard, 'LEFT WRIST (IR CLEAN)', (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(dashboard, 'TOP (ENSENSO ASYNC)', (441, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(dashboard, 'RIGHT WRIST (IR CLEAN)', (869, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.rectangle(dashboard, (0, 320), (1280, 800), (25, 25, 30), -1)

        # Overlay Front Webcam Image & Border on top of dark gray background
        dashboard[400:720, 800:1226] = disp_front
        cv2.rectangle(dashboard, (799, 399), (1227, 721), (0, 255, 255), 1)
        cv2.putText(dashboard, 'FRONT WEBCAM', (800, 390), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        curr_time = time.time()
        if self.is_saving:
            cv2.rectangle(dashboard, (20, 340), (450, 410), (0, 140, 255), -1)
            cv2.putText(dashboard, '⏳ WRITING HDF5 TO DISK...', (30, 368), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        elif self.save_success and (curr_time - self.save_success_timer) < 2.5:
            cv2.rectangle(dashboard, (20, 340), (450, 410), (0, 180, 0), -1)
            cv2.putText(dashboard, '✅ SAVE SUCCESSFUL!', (30, 368), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        elif self.is_recording:
            fps = 1.0 / np.mean(self.dt_history[-10:]) if len(self.dt_history) > 5 and np.mean(self.dt_history[-10:]) > 0 else 0.0
            cv2.rectangle(dashboard, (20, 340), (450, 410), (0, 0, 200), -1)
            cv2.putText(dashboard, f'🔴 RECORDING | {fps:.1f} FPS (Target: {self.target_fps:.0f})', (30, 368), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(dashboard, f"Frames: {len(self.episode_qpos)} | EP #{self.episode_idx}", (30, 395), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)
        else:
            cv2.rectangle(dashboard, (20, 340), (450, 410), (100, 80, 0), -1)
            cv2.putText(dashboard, f'READY (NEXT: episode_{self.episode_idx}.hdf5)', (30, 368), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        cv2.putText(dashboard, 'JOINT    LEFT (RAD)    RIGHT (RAD)', (20, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        for i in range(7):
            cv2.putText(dashboard, f' J{i+1} :     {self.latest_left_qpos[i]:+.3f}       {self.latest_right_qpos[i]:+.3f}', (20, 465 + i * 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 200), 1)
        cv2.putText(dashboard, f' GRP :     {self.latest_left_gripper:+.3f}       {self.latest_right_gripper:+.3f}', (20, 465 + 7 * 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        try:
            cv2.imshow('HARVEST Direct-Pull Logger', dashboard)
            cv2.waitKey(1)
        except Exception:
            pass

    def destroy_node(self):
        self.running = False
        if self.rs_left: self.rs_left.stop()
        if self.rs_right: self.rs_right.stop()
        if hasattr(self, 'cap_front') and self.cap_front is not None:
            self.cap_front.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    if ENSENSO_AVAILABLE:
        with NxLib():
            node = DirectHarvestNode()
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                pass
            finally:
                node.destroy_node()
                rclpy.shutdown()
    else:
        node = DirectHarvestNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()