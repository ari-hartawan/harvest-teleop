#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
HARVEST Framework: Native LeRobot Dataset Logger & GUI
- Listens to /harvest_safety_trigger to start/stop recording.
- Replaces harvest_webcam_gui_logger.py by writing directly to LeRobot format (Parquet + MP4).
- Preserves the exact original safety flow and controller switching on stop.
- Uses LeRobot's OpenCVCamera with persistent serial mapping.
- Supports Bimanual (`both`) and Single Arm (`right_only`, `left_only`) data collection.
"""

import sys
import os
os.environ["QT_LOGGING_RULES"] = "*.debug=false;*.warning=false"
import time
import subprocess
import threading
import numpy as np
import cv2
import torch

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# LeRobot imports
try:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.cameras.opencv import OpenCVCamera, OpenCVCameraConfig
    from lerobot.cameras import ColorMode
    from lerobot.cameras.configs import Cv2Backends
    HAS_LEROBOT = True
except ImportError as e:
    print(f"DEBUG: Import error details: {e}")
    import traceback
    traceback.print_exc()
    HAS_LEROBOT = False

# Persistent Logitech Camera Serial Mapping (Default Fallback)
CAMERA_SERIALS = {
    'top': 'BEDDD37F',
    'left_wrist': '9328796F',
    'right_wrist': '6B24796F',
    'front': 'F1ADD37F'
}


def load_camera_config():
    """
    Loads webcam_config.json and returns (active_camera_serials_dict, config_data).
    Maps role ('top', 'left_wrist', 'right_wrist', 'front') -> hardware serial number.
    """
    import json
    config_paths = [
        os.path.expanduser('~/workspace/harvest_ws/src/harvest_teleop/config/webcam_config.json'),
        os.path.expanduser('~/workspace/config/webcam_config.json'),
        os.path.expanduser('~/workspace/webcam_config.json'),
        os.path.join(os.getcwd(), 'src', 'harvest_teleop', 'config', 'webcam_config.json'),
        os.path.join(os.getcwd(), 'config', 'webcam_config.json'),
        os.path.join(os.getcwd(), 'webcam_config.json')
    ]
    serials = dict(CAMERA_SERIALS)
    config_data = None

    for path in config_paths:
        if os.path.exists(path):
            try:
                with open(path, 'r') as f:
                    config_data = json.load(f)
                
                # Extract role to serial
                if 'role_to_serial' in config_data and isinstance(config_data['role_to_serial'], dict):
                    for role, ser in config_data['role_to_serial'].items():
                        if ser and role in serials:
                            serials[role] = ser
                elif 'camera_serials' in config_data and isinstance(config_data['camera_serials'], dict):
                    cam_ser = config_data['camera_serials']
                    for k, v in cam_ser.items():
                        if k in serials and isinstance(v, str): # role -> serial
                            serials[k] = v
                        elif isinstance(v, str) and v in serials: # serial -> role
                            serials[v] = k

                print(f"📖 Loaded webcam_config.json from: {path}")
                break
            except Exception as e:
                print(f"⚠️ Warning loading webcam_config.json from {path}: {e}")

    return serials, config_data


def find_webcam_by_serial(serial_str):
    """Find persistent /dev/v4l/by-id path for a specific hardware serial number."""
    v4l_dir = '/dev/v4l/by-id'
    if os.path.exists(v4l_dir):
        for filename in os.listdir(v4l_dir):
            if 'index0' in filename and serial_str in filename:
                return os.path.join(v4l_dir, filename)
    return None


def apply_focus_parameters(dev_path, serial, role=None, config=None):
    """Loads webcam_config.json and applies the calibrated manual focus value to the V4L2 device."""
    if config is None:
        _, config = load_camera_config()
        
    focus_val = None
    if config:
        focus_by_serial = config.get('focus_by_serial', {})
        focus_by_role = config.get('focus_by_role', {})
        focus_values = config.get('focus_values', {})

        if serial in focus_by_serial:
            focus_val = focus_by_serial[serial]
        elif role and role in focus_by_role:
            focus_val = focus_by_role[role]
        elif serial in focus_values:
            focus_val = focus_values[serial]
        elif role and role in focus_values:
            focus_val = focus_values[role]
        else:
            devices = config.get('devices_detected', [])
            for i, dev in enumerate(devices):
                if dev.get('serial') == serial:
                    cam_key = f"Cam {i+1}"
                    focus_val = focus_values.get(cam_key)
                    break

    if focus_val is not None:
        role_label = f" ({role.upper()})" if role else ""
        print(f"🔧 Applying calibrated manual focus {focus_val} to camera {serial}{role_label} at {dev_path}...")
        try:
            subprocess.run(
                ['v4l2-ctl', '-d', dev_path, '-c', 'focus_automatic_continuous=0'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False
            )
            subprocess.run(
                ['v4l2-ctl', '-d', dev_path, '-c', f'focus_absolute={focus_val}'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False
            )
        except Exception:
            pass
    else:
        # Fallback to default disabling autofocus
        try:
            subprocess.run(
                ['v4l2-ctl', '-d', dev_path, '-c', 'focus_automatic_continuous=0'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False
            )
        except Exception:
            pass


class HarvestLeRobotLoggerNode(Node):
    def __init__(self):
        super().__init__('harvest_lerobot_logger_node')
        
        if not HAS_LEROBOT:
            self.get_logger().error("❌ LeRobot library not found! Please run inside 'lerobot_env' environment.")
            sys.exit(1)

        self.declare_parameter('dataset_dir', '~/harvest_lerobot_dataset')
        self.declare_parameter('record_fps', 30.0)
        self.declare_parameter('has_front', True)
        self.declare_parameter('task_description', 'perform teleoperation demonstration')
        self.declare_parameter('arm_mode', 'both')
        
        self.dataset_dir = os.path.expanduser(self.get_parameter('dataset_dir').value)
        self.target_fps = self.get_parameter('record_fps').value
        self.target_dt = 1.0 / self.target_fps
        self.has_front = self.get_parameter('has_front').value
        self.task_description = self.get_parameter('task_description').value
        self.arm_mode = str(self.get_parameter('arm_mode').value).lower()
        if self.arm_mode not in ['both', 'right_only', 'left_only']:
            self.get_logger().error(f"❌ Invalid arm_mode '{self.arm_mode}'. Options: both, right_only, left_only. Defaulting to 'both'.")
            self.arm_mode = 'both'

        self.is_recording = False
        self.saving_episodes_count = 0
        self.save_success = False
        self.save_success_timer = 0.0
        self.episode_frames_count = 0
        self.current_episode_idx = 0
        self.dataset_write_lock = threading.Lock()

        # Joint states buffers
        self.latest_left_qpos = [0.0] * 7
        self.latest_left_gripper = 0.4
        self.latest_right_qpos = [0.0] * 7
        self.latest_right_gripper = 0.4

        # Thread safety
        self.cam_lock = threading.Lock()
        self.episode_qpos = []
        self.episode_images = {'top': [], 'left_wrist': [], 'right_wrist': [], 'front': []}
        
        # Cameras dictionaries
        self.cameras = {}
        self.camera_frames = {
            'top': np.zeros((480, 640, 3), dtype=np.uint8),
            'left_wrist': np.zeros((480, 640, 3), dtype=np.uint8),
            'right_wrist': np.zeros((480, 640, 3), dtype=np.uint8),
            'front': np.zeros((480, 640, 3), dtype=np.uint8)
        }

        # Real-time FPS trackers for webcams
        self.cam_fps = {key: 0.0 for key in ['top', 'left_wrist', 'right_wrist', 'front']}
        self.cam_fps_timer = {key: time.time() for key in ['top', 'left_wrist', 'right_wrist', 'front']}
        self.cam_fps_counter = {key: 0 for key in ['top', 'left_wrist', 'right_wrist', 'front']}

        # Real-time recording loop rate tracker
        self.rec_fps = 0.0
        self.rec_fps_timer = time.time()
        self.rec_fps_counter = 0

        # Subscriptions based on arm_mode
        if self.arm_mode in ['both', 'left_only']:
            self.create_subscription(JointState, '/left_/joint_states', self.left_joint_callback, 10)
        if self.arm_mode in ['both', 'right_only']:
            self.create_subscription(JointState, '/right_/joint_states', self.right_joint_callback, 10)
        self.create_subscription(String, '/harvest_safety_trigger', self.safety_trigger_callback, 10)

        # Initialize Dataset
        self.init_dataset()

        # Initialize Cameras
        self.running = True
        self.init_cameras()

        # Main logging loop thread
        self.acquisition_thread = threading.Thread(target=self.acquisition_loop, daemon=True)
        self.acquisition_thread.start()

        # Timer for GUI rendering (30 FPS)
        self.create_timer(0.033, self.gui_render_callback)

        self.get_logger().info("=" * 65)
        self.get_logger().info(f" HARVEST LeRobot Live Logger Active | Target: {self.target_fps} Hz")
        self.get_logger().info(f" Arm Mode: {self.arm_mode.upper()} | State Dim: {'16D' if self.arm_mode == 'both' else '8D'}")
        self.get_logger().info(f" Saving to: {self.dataset_dir}")
        self.get_logger().info("=" * 65)

    def init_dataset(self):
        """Create or load the LeRobot dataset structure."""
        state_dim = 16 if self.arm_mode == 'both' else 8
        robot_type = "kinova_gen3_dual" if self.arm_mode == 'both' else "kinova_gen3_single"

        features = {
            "observation.state": {"dtype": "float32", "shape": (state_dim,)},
            "action": {"dtype": "float32", "shape": (state_dim,)},
            "observation.images.cam_top": {"dtype": "video", "shape": (3, 480, 640), "names": ["channels", "height", "width"]},
        }
        if self.arm_mode in ['both', 'left_only']:
            features["observation.images.cam_left_wrist"] = {"dtype": "video", "shape": (3, 480, 640), "names": ["channels", "height", "width"]}
        if self.arm_mode in ['both', 'right_only']:
            features["observation.images.cam_right_wrist"] = {"dtype": "video", "shape": (3, 480, 640), "names": ["channels", "height", "width"]}
        if self.has_front:
            features["observation.images.cam_front"] = {"dtype": "video", "shape": (3, 480, 640), "names": ["channels", "height", "width"]}

        # If dataset metadata already exists, we load it; otherwise, we create a new one.
        info_path = os.path.join(self.dataset_dir, "meta/info.json")
        if os.path.exists(info_path):
            self.get_logger().info(f"📂 Loading existing LeRobot dataset at {self.dataset_dir}")
            repo_name = os.path.basename(self.dataset_dir)
            self.dataset = LeRobotDataset.resume(repo_id=repo_name, root=self.dataset_dir)
            if hasattr(self.dataset, 'writer') and self.dataset.writer is not None:
                if hasattr(self.dataset.writer, '_rgb_encoder') and self.dataset.writer._rgb_encoder is not None:
                    self.dataset.writer._rgb_encoder.g = 30
            self.current_episode_idx = self.dataset.num_episodes
        else:
            if os.path.exists(self.dataset_dir):
                self.get_logger().info(f"🧹 Cleaning up incomplete dataset folder from previous failed run: {self.dataset_dir}")
                import shutil
                try:
                    shutil.rmtree(self.dataset_dir)
                except Exception as e:
                    self.get_logger().warn(f"Warning: could not remove directory: {e}")
            from lerobot.configs.video import RGBEncoderConfig
            rgb_enc = RGBEncoderConfig(vcodec="h264_nvenc", g=30)
            self.get_logger().info(f"✨ Creating new LeRobot dataset at {self.dataset_dir} (Robot: {robot_type}, Dim: {state_dim}D)")
            self.dataset = LeRobotDataset.create(
                repo_id=self.dataset_dir,
                fps=int(self.target_fps),
                robot_type=robot_type,
                features=features,
                use_videos=True,
                rgb_encoder=rgb_enc,
                encoder_threads=2,
            )
            self.current_episode_idx = 0

    def init_cameras(self):
        active_serials, config_data = load_camera_config()
        self.active_camera_serials = active_serials
        self.get_logger().info(f"📷 Active Camera Serials loaded: {self.active_camera_serials}")

        roles = [('top', self.active_camera_serials['top'])]
        if self.arm_mode in ['both', 'left_only']:
            roles.append(('left_wrist', self.active_camera_serials['left_wrist']))
        if self.arm_mode in ['both', 'right_only']:
            roles.append(('right_wrist', self.active_camera_serials['right_wrist']))
        if self.has_front:
            roles.append(('front', self.active_camera_serials['front']))

        for key, serial in roles:
            dev_path = find_webcam_by_serial(serial)
            if dev_path:
                apply_focus_parameters(dev_path, serial, role=key, config=config_data)

                cam_cfg = OpenCVCameraConfig(
                    index_or_path=dev_path,
                    fps=30,
                    width=640,
                    height=480,
                    color_mode=ColorMode.RGB,
                    fourcc="MJPG",
                    backend=Cv2Backends.V4L2
                )
                try:
                    cam = OpenCVCamera(cam_cfg)
                    cam.connect()
                    self.cameras[key] = cam
                    self.get_logger().info(f"✅ OpenCVCamera [{key.upper()}] Bound -> {dev_path}")
                    
                    # Dedicated per-camera parallel worker thread running async_read
                    t = threading.Thread(target=self._camera_read_loop, args=(key, cam), daemon=True)
                    t.start()
                except Exception as e:
                    self.get_logger().error(f"❌ Failed to connect OpenCVCamera [{key.upper()}]: {e}")
            else:
                self.get_logger().error(f"❌ Camera [{key.upper()}] serial {serial} NOT found in /dev/v4l/by-id/")

    def _camera_read_loop(self, key, cam):
        while self.running and rclpy.ok():
            try:
                # Parallel async_read waits for new hardware frame signal independently
                img = cam.async_read(timeout_ms=35)
                if img is not None:
                    now = time.time()
                    self.cam_fps_counter[key] += 1
                    if now - self.cam_fps_timer[key] >= 1.0:
                        self.cam_fps[key] = self.cam_fps_counter[key] / (now - self.cam_fps_timer[key])
                        self.cam_fps_counter[key] = 0
                        self.cam_fps_timer[key] = now

                    with self.cam_lock:
                        self.camera_frames[key] = img
            except Exception:
                pass
            time.sleep(0.001)

    def safety_trigger_callback(self, msg):
        cmd = msg.data.strip()
        if cmd == 'START_RECORDING':
            self.start_recording()
        elif cmd == 'STOP_RECORDING':
            self.stop_and_save_recording()
        elif cmd == 'CANCEL_RECORDING':
            self.cancel_recording()

    def left_joint_callback(self, msg):
        self.parse_joint_msg(msg, is_right=False)

    def right_joint_callback(self, msg):
        self.parse_joint_msg(msg, is_right=True)

    def parse_joint_msg(self, msg, is_right=True):
        temp_qpos = [0.0] * 7
        gripper_val = 0.4
        for idx, (name, pos) in enumerate(zip(msg.name, msg.position)):
            for i in range(1, 8):
                if f'joint_{i}' in name:
                    temp_qpos[i - 1] = float(pos)
                    break
            if 'knuckle_joint' in name:
                gripper_val = float(pos)
        if is_right:
            self.latest_right_qpos = temp_qpos
            self.latest_right_gripper = gripper_val
        else:
            self.latest_left_qpos = temp_qpos
            self.latest_left_gripper = gripper_val

    def start_recording(self):
        self.episode_qpos = []
        self.episode_images = {'top': [], 'left_wrist': [], 'right_wrist': [], 'front': []}
        self.episode_frames_count = 0
        self.save_success = False
        self.is_recording = True
        
        # Reset recording FPS counter
        self.rec_fps = 0.0
        self.rec_fps_timer = time.time()
        self.rec_fps_counter = 0
        
        self.get_logger().info(f"🔴 [LEROBOT RECORDING STARTED] Episode #{self.current_episode_idx} ({self.arm_mode.upper()} Mode)")

    def cancel_recording(self):
        if not self.is_recording:
            return
        self.is_recording = False
        self.episode_qpos = []
        self.episode_images = {'top': [], 'left_wrist': [], 'right_wrist': [], 'front': []}
        self.episode_frames_count = 0
        self.get_logger().warn(f"🗑️ [LEROBOT RECORDING CANCELLED] Discarded current episode #{self.current_episode_idx}")

    def acquisition_loop(self):
        next_tick = time.perf_counter()
        while self.running and rclpy.ok():
            now = time.perf_counter()
            sleep_dur = next_tick - now
            if sleep_dur > 0:
                time.sleep(sleep_dur)
            next_tick += self.target_dt

            # Acquisition loop running at 30 Hz using parallel camera worker frames

            if self.is_recording:
                # Capture current joint space state
                if self.arm_mode == 'both':
                    state_vec = np.zeros(16, dtype=np.float32)
                    state_vec[0:7] = self.latest_left_qpos
                    state_vec[7] = self.latest_left_gripper
                    state_vec[8:15] = self.latest_right_qpos
                    state_vec[15] = self.latest_right_gripper
                elif self.arm_mode == 'right_only':
                    state_vec = np.zeros(8, dtype=np.float32)
                    state_vec[0:7] = self.latest_right_qpos
                    state_vec[7] = self.latest_right_gripper
                elif self.arm_mode == 'left_only':
                    state_vec = np.zeros(8, dtype=np.float32)
                    state_vec[0:7] = self.latest_left_qpos
                    state_vec[7] = self.latest_left_gripper

                # Action matches current state in Joint Space teleoperation datasets
                action_vec = state_vec.copy()

                with self.cam_lock:
                    img_top = self.camera_frames['top'].copy()
                    img_left = self.camera_frames['left_wrist'].copy() if self.arm_mode in ['both', 'left_only'] else None
                    img_right = self.camera_frames['right_wrist'].copy() if self.arm_mode in ['both', 'right_only'] else None
                    img_front = self.camera_frames['front'].copy() if self.has_front else None

                # Store frames in RGB format directly
                self.episode_qpos.append((state_vec, action_vec))
                self.episode_images['top'].append(img_top)
                if self.arm_mode in ['both', 'left_only']:
                    self.episode_images['left_wrist'].append(img_left)
                if self.arm_mode in ['both', 'right_only']:
                    self.episode_images['right_wrist'].append(img_right)
                if self.has_front:
                    self.episode_images['front'].append(img_front)

                self.episode_frames_count += 1

                # Compute actual recording loop rate
                self.rec_fps_counter += 1
                rec_now = time.time()
                if rec_now - self.rec_fps_timer >= 1.0:
                    self.rec_fps = self.rec_fps_counter / (rec_now - self.rec_fps_timer)
                    self.rec_fps_counter = 0
                    self.rec_fps_timer = rec_now

    def stop_and_save_recording(self):
        if not self.is_recording:
            return
        self.is_recording = False
        self.save_success = False

        if self.episode_frames_count == 0:
            self.get_logger().warn("⚠️ No frames recorded!")
            return

        # Snapshot active buffers (Double Buffering)
        episode_data = {
            'qpos': self.episode_qpos,
            'images': {k: list(v) for k, v in self.episode_images.items()},
            'frames_count': self.episode_frames_count,
            'episode_idx': self.current_episode_idx
        }

        # Reset active recording buffers immediately for the next episode
        self.episode_qpos = []
        self.episode_images = {'top': [], 'left_wrist': [], 'right_wrist': [], 'front': []}
        self.episode_frames_count = 0
        self.current_episode_idx += 1

        self.saving_episodes_count += 1
        t = threading.Thread(target=self._write_episode_to_lerobot, args=(episode_data,))
        t.start()

    def _write_episode_to_lerobot(self, ep_data):
        # Set background thread nice priority to lower priority so it never starves live acquisition threads
        with self.dataset_write_lock:
            try:
                frames_count = ep_data['frames_count']
                episode_idx = ep_data['episode_idx']
                self.get_logger().info(f"⏳ Writing {frames_count} frames for Episode {episode_idx} directly to LeRobot dataset ({self.arm_mode.upper()})...")

                # Extract states and actions to arrays for batch unwrapping & alignment
                states = np.array([item[0] for item in ep_data['qpos']], dtype=np.float32)
                actions = np.array([item[1] for item in ep_data['qpos']], dtype=np.float32)

                # Reference HOME position for continuous joint alignment
                HOME_LEFT = [0.00632, 0.19647, -3.13367, -2.09887, -0.01613, -0.84450, 1.57919]
                HOME_RIGHT = [0.00126, 0.18837, -3.11828, -2.10992, -0.03365, -0.83863, 1.60368]

                if self.arm_mode == 'both':
                    HOME_REF = HOME_LEFT + [0.3] + HOME_RIGHT + [0.3]
                    cont_indices = [0, 2, 4, 6, 8, 10, 12, 14]
                    arm_joint_indices = [i for i in range(16) if i not in (7, 15)]
                elif self.arm_mode == 'right_only':
                    HOME_REF = HOME_RIGHT + [0.3]
                    cont_indices = [0, 2, 4, 6]
                    arm_joint_indices = [0, 1, 2, 3, 4, 5, 6]
                elif self.arm_mode == 'left_only':
                    HOME_REF = HOME_LEFT + [0.3]
                    cont_indices = [0, 2, 4, 6]
                    arm_joint_indices = [0, 1, 2, 3, 4, 5, 6]

                # Align continuous joint starting angles to HOME_REF and unwrap time-series
                for idx in cont_indices:
                    ref = HOME_REF[idx]
                    diff_st = states[0, idx] - ref
                    turns_st = np.round(diff_st / (2.0 * np.pi))
                    states[:, idx] = states[:, idx] - turns_st * (2.0 * np.pi)

                    diff_act = actions[0, idx] - ref
                    turns_act = np.round(diff_act / (2.0 * np.pi))
                    actions[:, idx] = actions[:, idx] - turns_act * (2.0 * np.pi)

                    states[:, idx] = np.unwrap(states[:, idx], axis=0)
                    actions[:, idx] = np.unwrap(actions[:, idx], axis=0)



                for idx in range(frames_count):
                    state = states[idx]
                    action = actions[idx]
                    
                    # Retrieve RGB frames and transpose to channel-first [C, H, W] for LeRobot
                    img_top = ep_data['images']['top'][idx].transpose(2, 0, 1)

                    frame_payload = {
                        "observation.state": state,
                        "action": action,
                        "observation.images.cam_top": img_top,
                        "task": self.task_description,
                    }

                    if self.arm_mode in ['both', 'left_only']:
                        img_left = ep_data['images']['left_wrist'][idx].transpose(2, 0, 1)
                        frame_payload["observation.images.cam_left_wrist"] = img_left

                    if self.arm_mode in ['both', 'right_only']:
                        img_right = ep_data['images']['right_wrist'][idx].transpose(2, 0, 1)
                        frame_payload["observation.images.cam_right_wrist"] = img_right

                    if self.has_front:
                        img_front = ep_data['images']['front'][idx].transpose(2, 0, 1)
                        frame_payload["observation.images.cam_front"] = img_front

                    # Add to dataset
                    self.dataset.add_frame(frame_payload)

                # Save the episode safely without parallel NVENC process collisions
                self.dataset.save_episode(parallel_encoding=False)
                self.get_logger().info(f"✅ LeRobot Episode {episode_idx} Saved successfully!")
                
                self.save_success = True
                self.save_success_timer = time.time()
            except Exception as e:
                self.get_logger().error(f"❌ Failed to save Episode {episode_idx} to LeRobot: {e}")
            finally:
                self.saving_episodes_count -= 1

    def gui_render_callback(self):
        # 1280x800 Dashboard
        dashboard = np.zeros((800, 1280, 3), dtype=np.uint8)

        with self.cam_lock:
            left_img = self.camera_frames['left_wrist'].copy()
            top_img = self.camera_frames['top'].copy()
            right_img = self.camera_frames['right_wrist'].copy()
            front_img = self.camera_frames['front'].copy()

        disp_left = cv2.cvtColor(cv2.resize(left_img, (426, 320)), cv2.COLOR_RGB2BGR) if self.arm_mode in ['both', 'left_only'] else np.zeros((320, 426, 3), dtype=np.uint8)
        disp_top = cv2.cvtColor(cv2.resize(top_img, (428, 320)), cv2.COLOR_RGB2BGR)
        disp_right = cv2.cvtColor(cv2.resize(right_img, (426, 320)), cv2.COLOR_RGB2BGR) if self.arm_mode in ['both', 'right_only'] else np.zeros((320, 426, 3), dtype=np.uint8)
        disp_front = cv2.cvtColor(cv2.resize(front_img, (426, 320)), cv2.COLOR_RGB2BGR) if self.has_front else np.zeros((320, 426, 3), dtype=np.uint8)

        if self.arm_mode == 'right_only':
            cv2.putText(disp_left, 'DISABLED (RIGHT ARM MODE)', (40, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 1)
        elif self.arm_mode == 'left_only':
            cv2.putText(disp_right, 'DISABLED (LEFT ARM MODE)', (40, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 100), 1)

        dashboard[0:320, 0:426] = disp_left
        dashboard[0:320, 426:854] = disp_top
        dashboard[0:320, 854:1280] = disp_right

        lbl_left = f'LEFT WRIST ({self.cam_fps["left_wrist"]:.1f} FPS)' if self.arm_mode in ['both', 'left_only'] else 'LEFT WRIST (OFF)'
        lbl_top = f'TOP WEBCAM ({self.cam_fps["top"]:.1f} FPS)'
        lbl_right = f'RIGHT WRIST ({self.cam_fps["right_wrist"]:.1f} FPS)' if self.arm_mode in ['both', 'right_only'] else 'RIGHT WRIST (OFF)'

        cv2.putText(dashboard, lbl_left, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255) if self.arm_mode in ['both', 'left_only'] else (100, 100, 100), 2 if self.arm_mode in ['both', 'left_only'] else 1)
        cv2.putText(dashboard, lbl_top, (441, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        cv2.putText(dashboard, lbl_right, (869, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255) if self.arm_mode in ['both', 'right_only'] else (100, 100, 100), 2 if self.arm_mode in ['both', 'right_only'] else 1)

        cv2.rectangle(dashboard, (0, 320), (1280, 800), (25, 25, 30), -1)

        dashboard[400:720, 800:1226] = disp_front
        cv2.rectangle(dashboard, (799, 399), (1227, 721), (0, 255, 255) if self.has_front else (80, 80, 80), 1)
        lbl_front = f'FRONT WEBCAM ({self.cam_fps["front"]:.1f} FPS)' if self.has_front else 'FRONT WEBCAM (OFF)'
        cv2.putText(dashboard, lbl_front, (800, 390), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255) if self.has_front else (100, 100, 100), 2 if self.has_front else 1)

        curr_time = time.time()
        
        # 1. Recording / Node State Box (x: 20 to 450)
        mode_str = f"MODE: {self.arm_mode.upper()}"
        if self.is_recording:
            cv2.rectangle(dashboard, (20, 340), (450, 410), (0, 0, 200), -1)
            cv2.putText(dashboard, f'🔴 REC EP #{self.current_episode_idx} [{mode_str}]', (30, 368), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            cv2.putText(dashboard, f"Frames: {self.episode_frames_count} | Actual: {self.rec_fps:.1f} Hz (Target: {self.target_fps} Hz)", (30, 395), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (220, 220, 220), 1)
        else:
            cv2.rectangle(dashboard, (20, 340), (450, 410), (0, 100, 0), -1)
            cv2.putText(dashboard, f'🟢 READY / IDLE [{mode_str}]', (30, 368), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            cv2.putText(dashboard, f"Next Ep Index: {self.current_episode_idx}", (30, 395), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)

        # 2. Asynchronous Saving Queue Box (x: 480 to 780)
        if self.saving_episodes_count > 0:
            cv2.rectangle(dashboard, (480, 340), (780, 410), (0, 140, 255), -1)
            cv2.putText(dashboard, f'⏳ SAVING QUEUE', (490, 368), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(dashboard, f"Pending: {self.saving_episodes_count} episodes", (490, 395), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)
        elif self.save_success and (curr_time - self.save_success_timer) < 3.0:
            cv2.rectangle(dashboard, (480, 340), (780, 410), (0, 160, 0), -1)
            cv2.putText(dashboard, '✅ SAVE SUCCESSFUL', (490, 368), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(dashboard, "All files written to disk", (490, 395), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)
        else:
            cv2.rectangle(dashboard, (480, 340), (780, 410), (30, 30, 35), -1)
            cv2.rectangle(dashboard, (480, 340), (780, 410), (80, 80, 80), 1)
            cv2.putText(dashboard, '💤 QUEUE EMPTY', (490, 368), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 120, 120), 1)
            cv2.putText(dashboard, "Idle - No pending saves", (490, 395), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 100, 100), 1)

        # Telemetry Display (Joint Angles)
        if self.arm_mode == 'both':
            cv2.putText(dashboard, 'JOINT VALUES (RAD)      LEFT ARM          RIGHT ARM', (20, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1)
            for i in range(7):
                val_l = self.latest_left_qpos[i]
                val_r = self.latest_right_qpos[i]
                cv2.putText(dashboard, f' JOINT {i+1} :              {val_l:+.4f}           {val_r:+.4f}', 
                            (20, 465 + i * 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 200), 1)
            cv2.putText(dashboard, f' GRIPPER :              {self.latest_left_gripper:+.4f}           {self.latest_right_gripper:+.4f}', 
                        (20, 619), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 0), 1)
        elif self.arm_mode == 'right_only':
            cv2.putText(dashboard, 'JOINT VALUES (RAD)      RIGHT ARM ONLY', (20, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1)
            for i in range(7):
                val_r = self.latest_right_qpos[i]
                cv2.putText(dashboard, f' JOINT {i+1} :              {val_r:+.4f}', 
                            (20, 465 + i * 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 200), 1)
            cv2.putText(dashboard, f' GRIPPER :              {self.latest_right_gripper:+.4f}', 
                        (20, 619), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 0), 1)
        elif self.arm_mode == 'left_only':
            cv2.putText(dashboard, 'JOINT VALUES (RAD)      LEFT ARM ONLY', (20, 440), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 1)
            for i in range(7):
                val_l = self.latest_left_qpos[i]
                cv2.putText(dashboard, f' JOINT {i+1} :              {val_l:+.4f}', 
                            (20, 465 + i * 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 200), 1)
            cv2.putText(dashboard, f' GRIPPER :              {self.latest_left_gripper:+.4f}', 
                        (20, 619), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 0), 1)

        try:
            cv2.imshow('HARVEST LeRobot Live Logger', dashboard)
            cv2.waitKey(1)
        except Exception:
            pass

    def shutdown_procedure(self):
        self.running = False
        
        # Wait for all background saves to finish
        if hasattr(self, 'saving_episodes_count') and self.saving_episodes_count > 0:
            self.get_logger().info(f"⏳ Waiting for {self.saving_episodes_count} background save(s) to complete before shutting down...")
            while self.saving_episodes_count > 0:
                time.sleep(0.5)

        # Disconnect LeRobot cameras
        for key, cam in self.cameras.items():
            cam.disconnect()
            self.get_logger().info(f"Released OpenCVCamera: {key}")
        self.cameras.clear()

        # Finalize the dataset
        if hasattr(self, 'dataset') and self.dataset is not None:
            self.get_logger().info("⏳ Finalizing LeRobot Dataset (writing stats, compressing MP4s)...")
            self.dataset.finalize()
            self.get_logger().info(f"🎉 Dataset saved & finalized at: {self.dataset_dir}")

        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = HarvestLeRobotLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_procedure()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
