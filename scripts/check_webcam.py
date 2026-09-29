#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
HARVEST Framework: 4-Point USB Webcam Diagnostic & Alignment Tool
- Discovers connected USB webcams (/dev/v4l/by-id/ and /dev/video*).
- Identifies cameras by persistent hardware serial numbers (F1ADD37F=Front, 9328796F=Left Wrist, 6B24796F=Right Wrist, BEDDD37F=Top).
- Automatically disables Auto-Focus (sets Fixed Focus) for stable ACT dataset collection.
- Displays live preview matching ACT dataset resolution (640x480, 3-channel BGR).
- Provides 2x2 Grid Mode and Single-Camera Focus Mode for physical camera position & focus adjustment.
- Measures real-time FPS for each webcam stream.
- Saves camera configuration to webcam_config.json.
"""

import os
# Suppress Qt and OpenCV noise warnings in terminal
os.environ["QT_LOGGING_RULES"] = "*=false"
os.environ["OPENCV_LOG_LEVEL"] = "ERROR"

import sys
import glob
import time
import json
import subprocess
import threading
import numpy as np
import cv2

# Persistent Hardware Serial Mapping for the 4 Logitech C922 Webcams (Default Fallback)
DEFAULT_CAMERA_SERIALS = {
    'F1ADD37F': 'front',
    '9328796F': 'left_wrist',
    '6B24796F': 'right_wrist',
    'BEDDD37F': 'top'
}

CONFIG_FILE_PATHS = [
    os.path.expanduser('~/workspace/harvest_ws/src/harvest_teleop/config/webcam_config.json'),
    os.path.join(os.getcwd(), 'src', 'harvest_teleop', 'config', 'webcam_config.json'),
    os.path.join(os.getcwd(), 'config', 'webcam_config.json'),
    os.path.join(os.getcwd(), 'webcam_config.json')
]


def load_existing_config():
    """Load existing webcam_config.json if available from candidate paths."""
    for path in CONFIG_FILE_PATHS:
        if os.path.exists(path):
            try:
                with open(path, 'r') as f:
                    config = json.load(f)
                print(f"📖 Loaded existing configuration from: {path}")
                return config, path
            except Exception as e:
                print(f"⚠️ Warning loading config from {path}: {e}")
    return None, None


def get_focus_absolute_v4l2(device_path):
    """Query current V4L2 focus_absolute value from the hardware."""
    import re
    try:
        res = subprocess.run(
            ['v4l2-ctl', '-d', device_path, '--get-ctrl=focus_absolute'],
            capture_output=True, text=True, check=False
        )
        if res.returncode == 0:
            match = re.search(r"focus_absolute:\s*(\d+)", res.stdout)
            if match:
                return int(match.group(1))
    except Exception:
        pass
    return 100


def set_autofocus_v4l2(device_path, enable):
    """Enable or disable V4L2 continuous autofocus."""
    val = 1 if enable else 0
    try:
        subprocess.run(
            ['v4l2-ctl', '-d', device_path, f'--set-ctrl=focus_automatic_continuous={val}'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False
        )
    except Exception:
        pass


def disable_autofocus_v4l2(device_path):
    """Disable V4L2 continuous autofocus for robust fixed-focus operation."""
    set_autofocus_v4l2(device_path, False)


def extract_serial_from_path(path_str):
    """Extract 8-character hex serial number from /dev/v4l/by-id path."""
    import re
    match = re.search(r'Webcam_([A-Za-z0-9]{8})-video', path_str)
    if match:
        return match.group(1)
    for serial in DEFAULT_CAMERA_SERIALS.keys():
        if serial in path_str:
            return serial
    return None


def get_v4l_devices():
    """
    Find primary video capture devices registered in /dev/v4l/by-id/ and /dev/video*.
    Uses persistent /dev/v4l/by-id/ index0 symlinks for reliable hardware discovery.
    """
    devices = []
    by_id_dir = '/dev/v4l/by-id'
    
    if os.path.exists(by_id_dir):
        for filename in sorted(os.listdir(by_id_dir)):
            if 'index0' in filename:
                by_id_path = os.path.join(by_id_dir, filename)
                real_node = os.path.realpath(by_id_path)
                serial = extract_serial_from_path(filename)
                devices.append({
                    'device_path': real_node,
                    'by_id_path': by_id_path,
                    'name': filename,
                    'real_path': real_node,
                    'serial': serial
                })
    
    # Fallback to /dev/video* if by-id is empty
    if not devices:
        video_nodes = sorted(glob.glob('/dev/video*'))
        for node in video_nodes:
            if int(node.replace('/dev/video', '')) % 2 == 0:
                devices.append({
                    'device_path': node,
                    'by_id_path': node,
                    'name': os.path.basename(node),
                    'real_path': node,
                    'serial': None
                })

    return devices


class WebcamStreamThread:
    """Threaded webcam reader for high frame rate acquisition and FPS tracking."""
    def __init__(self, device_path, target_width=640, target_height=480, name="Webcam", initial_focus=None):
        self.device_path = device_path
        self.target_width = target_width
        self.target_height = target_height
        self.name = name
        
        # Disable autofocus via v4l2-ctl prior to opening
        self.autofocus = False
        disable_autofocus_v4l2(self.device_path)

        if initial_focus is not None:
            self.focus_value = initial_focus
        else:
            self.focus_value = get_focus_absolute_v4l2(self.device_path)

        self.cap = cv2.VideoCapture(self.device_path, cv2.CAP_V4L2)
        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.target_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.target_height)
            self.cap.set(cv2.CAP_PROP_FPS, 15)
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) # Disable OpenCV autofocus property
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.running = True
        self.lock = threading.Lock()
        self.latest_frame = np.zeros((target_height, target_width, 3), dtype=np.uint8)
        self.is_valid = False
        self.fps = 0.0
        self.fps_timer = time.time()
        self.actual_shape = (0, 0, 0)

        # Apply initial focus value after opening VideoCapture
        self.set_manual_focus(self.focus_value)
        
        self.thread = threading.Thread(target=self._update_loop, daemon=True)
        if self.cap.isOpened():
            self.thread.start()

    def _update_loop(self):
        fps_counter = 0

        while self.running and self.cap.isOpened():
            ret, frame = self.cap.read()
            now = time.time()
            
            if ret and frame is not None:
                shape = frame.shape
                if (shape[1], shape[0]) != (self.target_width, self.target_height):
                    frame_resized = cv2.resize(frame, (self.target_width, self.target_height))
                else:
                    frame_resized = frame

                fps_counter += 1
                if now - self.fps_timer >= 1.0:
                    self.fps = fps_counter / (now - self.fps_timer)
                    fps_counter = 0
                    self.fps_timer = now

                with self.lock:
                    self.latest_frame = frame_resized
                    self.actual_shape = shape
                    self.is_valid = True
            else:
                with self.lock:
                    self.is_valid = False
                time.sleep(0.01)

    def set_autofocus(self, enable):
        with self.lock:
            self.autofocus = enable
        val = 1 if enable else 0
        if self.cap and self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_AUTOFOCUS, val)
        set_autofocus_v4l2(self.device_path, enable)
        if not enable:
            self.set_manual_focus(self.focus_value)
        print(f"[{self.name}] Autofocus set to {'ON' if enable else 'OFF'}")

    def set_manual_focus(self, val):
        val = max(0, min(250, val))
        with self.lock:
            self.focus_value = val
            if self.autofocus:
                self.autofocus = False
                set_autofocus_v4l2(self.device_path, False)
                if self.cap and self.cap.isOpened():
                    self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        try:
            subprocess.run(
                ['v4l2-ctl', '-d', self.device_path, f'--set-ctrl=focus_absolute={val}'],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False
            )
        except Exception:
            pass
        if self.cap and self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FOCUS, val)
        print(f"[{self.name}] Manual focus set to {val}")

    def get_frame(self):
        with self.lock:
            return self.latest_frame.copy(), self.is_valid, self.fps, self.actual_shape

    def stop(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)
        if self.cap and self.cap.isOpened():
            self.cap.release()


class WebcamCheckerApp:
    POSITIONS = ['front', 'left_wrist', 'right_wrist', 'top']
    POSITION_LABELS = {
        'front': 'FRONT WEBCAM',
        'left_wrist': 'LEFT WRIST',
        'right_wrist': 'RIGHT WRIST',
        'top': 'TOP WEBCAM'
    }

    def __init__(self, devices):
        self.devices = devices
        self.streams = []
        self.assignments = {'front': None, 'left_wrist': None, 'right_wrist': None, 'top': None}
        self.camera_serials = dict(DEFAULT_CAMERA_SERIALS) # serial -> role

        # Load existing config if available
        self.config_data, self.config_path = load_existing_config()
        if self.config_data:
            # Load serial_to_role or role_to_serial
            if 'role_to_serial' in self.config_data and isinstance(self.config_data['role_to_serial'], dict):
                for role, serial in self.config_data['role_to_serial'].items():
                    if serial:
                        self.camera_serials[serial] = role
            elif 'camera_serials' in self.config_data and isinstance(self.config_data['camera_serials'], dict):
                cam_ser = self.config_data['camera_serials']
                for k, v in cam_ser.items():
                    if k in DEFAULT_CAMERA_SERIALS: # serial -> role
                        self.camera_serials[k] = v
                    elif v in DEFAULT_CAMERA_SERIALS: # role -> serial
                        self.camera_serials[v] = k

        print("\n" + "="*70)
        print("  HARVEST FRAMEWORK: 4-POINT WEBCAM DISCOVERY & ALIGNMENT TOOL")
        print("="*70)
        print(f"Found {len(self.devices)} video capture devices:\n")

        for idx, dev in enumerate(self.devices):
            serial = dev['serial']
            role = self.camera_serials.get(serial, "UNASSIGNED") if serial else "UNASSIGNED"
            print(f"  [{idx+1}] Node: {dev['device_path']} | Serial: {serial} -> {role.upper()}")
            print(f"      by-id: {dev['by_id_path']}")
            print("-" * 50)
            
            # Determine initial focus for this stream
            init_focus = None
            if self.config_data:
                focus_by_serial = self.config_data.get('focus_by_serial', {})
                focus_by_role = self.config_data.get('focus_by_role', {})
                focus_values = self.config_data.get('focus_values', {})

                if serial and serial in focus_by_serial:
                    init_focus = focus_by_serial[serial]
                elif role != "UNASSIGNED" and role in focus_by_role:
                    init_focus = focus_by_role[role]
                elif f"Cam {idx+1}" in focus_values:
                    init_focus = focus_values[f"Cam {idx+1}"]
                elif serial and serial in focus_values:
                    init_focus = focus_values[serial]

            stream = WebcamStreamThread(
                device_path=dev['by_id_path'],
                target_width=640,
                target_height=480,
                name=f"Cam {idx+1}",
                initial_focus=init_focus
            )
            self.streams.append(stream)

            if serial and role != "UNASSIGNED":
                self.assignments[role] = dev['by_id_path']

        self.selected_cam_idx = None
        self.running = True

    def assign_role_to_camera(self, cam_idx, new_role):
        if cam_idx is None or cam_idx < 0 or cam_idx >= len(self.devices):
            print("⚠️ Select a camera [1-4] first to assign its position!")
            return

        dev = self.devices[cam_idx]
        curr_dev_path = dev['by_id_path']
        serial = dev['serial']

        # Clear any previous assignment pointing to this dev_path or role
        for role, path in list(self.assignments.items()):
            if path == curr_dev_path or role == new_role:
                self.assignments[role] = None

        self.assignments[new_role] = curr_dev_path
        
        if serial:
            # Clear any other serial that mapped to new_role
            for s, r in list(self.camera_serials.items()):
                if r == new_role:
                    del self.camera_serials[s]
            self.camera_serials[serial] = new_role

        print(f"✅ Cam {cam_idx+1} (SN: {serial}) assigned to {new_role.upper().replace('_', ' ')}")

    def run(self):
        print("\n" + "="*70)
        print(" HARDWARE SERIAL & ROLE MAPPING:")
        for idx, dev in enumerate(self.devices):
            ser = dev['serial']
            role = self.camera_serials.get(ser, "UNASSIGNED") if ser else "UNASSIGNED"
            print(f"   Cam {idx+1} ({ser}) -> {role.upper().replace('_', ' ')}")
            
        print("\n KEYBOARD CONTROLS:")
        print("   [0]        : Grid View (All cameras)")
        print("   [1] - [4]  : Select Camera / Focus Mode (Cam 1..4)")
        print("   [F]        : Assign selected camera to FRONT")
        print("   [L]        : Assign selected camera to LEFT WRIST")
        print("   [R]        : Assign selected camera to RIGHT WRIST")
        print("   [T]        : Assign selected camera to TOP")
        print("   [-] / [+]  : Adjust Manual Focus (-5 / +5)")
        print("   [,] / [.]  : Fine Adjust Manual Focus (-1 / +1)")
        print("   [A]        : Toggle Autofocus (ON/OFF)")
        print("   [S]        : Save Configuration (webcam_config.json)")
        print("   [Q] / [ESC]: Exit Program")
        print("="*70 + "\n")

        window_name = "HARVEST 4-Point Webcam Checker (Autofocus OFF)"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 1280, 960)

        while self.running:
            if self.selected_cam_idx is None:
                canvas = self._render_grid_view()
            else:
                canvas = self._render_focus_view(self.selected_cam_idx)

            cv2.imshow(window_name, canvas)
            key = cv2.waitKey(30) & 0xFF

            if key in [ord('q'), ord('Q'), 27]:
                self.running = False
            elif key == ord('0'):
                self.selected_cam_idx = None
            elif ord('1') <= key <= ord('9'):
                idx = key - ord('1')
                if idx < len(self.streams):
                    self.selected_cam_idx = idx
            elif key in [ord('s'), ord('S')]:
                self._save_configuration()
            elif key in [ord('a'), ord('A')]:
                if self.selected_cam_idx is None:
                    # Toggle for all streams
                    any_off = any(not s.autofocus for s in self.streams)
                    for s in self.streams:
                        s.set_autofocus(any_off)
                    print(f"✅ All cameras autofocus set to {'ON' if any_off else 'OFF'}")
                elif 0 <= self.selected_cam_idx < len(self.streams):
                    stream = self.streams[self.selected_cam_idx]
                    new_state = not stream.autofocus
                    stream.set_autofocus(new_state)
            elif key in [ord('f'), ord('F')]:
                self.assign_role_to_camera(self.selected_cam_idx, 'front')
            elif key in [ord('l'), ord('L')]:
                self.assign_role_to_camera(self.selected_cam_idx, 'left_wrist')
            elif key in [ord('r'), ord('R')]:
                self.assign_role_to_camera(self.selected_cam_idx, 'right_wrist')
            elif key in [ord('t'), ord('T')]:
                self.assign_role_to_camera(self.selected_cam_idx, 'top')
            elif self.selected_cam_idx is not None and 0 <= self.selected_cam_idx < len(self.streams):
                stream = self.streams[self.selected_cam_idx]
                if key in [ord('['), ord('-')]:
                    new_val = stream.focus_value - 5
                    stream.set_manual_focus(new_val)
                elif key in [ord(']'), ord('+'), ord('=')]:
                    new_val = stream.focus_value + 5
                    stream.set_manual_focus(new_val)
                elif key in [ord(','), ord('<')]:
                    new_val = stream.focus_value - 1
                    stream.set_manual_focus(new_val)
                elif key in [ord('.'), ord('>')]:
                    new_val = stream.focus_value + 1
                    stream.set_manual_focus(new_val)

        self.cleanup()

    def _render_grid_view(self):
        canvas = np.zeros((960, 1280, 3), dtype=np.uint8)
        grid_offsets = [
            (0, 0),       # Top-Left (Cam 1)
            (640, 0),     # Top-Right (Cam 2)
            (0, 480),     # Bottom-Left (Cam 3)
            (640, 480)    # Bottom-Right (Cam 4)
        ]

        for idx in range(min(4, len(self.streams))):
            stream = self.streams[idx]
            frame, valid, fps, actual_shape = stream.get_frame()
            x, y = grid_offsets[idx]

            if not valid:
                cv2.rectangle(canvas, (x, y), (x + 640, y + 480), (30, 30, 30), -1)
                cv2.putText(canvas, f"CAM {idx+1} OFFLINE", (x + 200, y + 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            else:
                canvas[y:y+480, x:x+640] = frame

            cv2.rectangle(canvas, (x, y), (x + 640, y + 480), (80, 80, 80), 2)

            serial = self.devices[idx]['serial'] or 'UNKNOWN'
            dev_path = self.devices[idx]['by_id_path']
            
            assigned_role = "UNASSIGNED"
            for pos_key, path in self.assignments.items():
                if path == dev_path:
                    assigned_role = pos_key.upper().replace('_', ' ')
                    break

            banner_color = (0, 180, 0) if assigned_role != "UNASSIGNED" else (50, 50, 50)
            cv2.rectangle(canvas, (x + 10, y + 10), (x + 630, y + 65), (0, 0, 0), -1)
            cv2.rectangle(canvas, (x + 10, y + 10), (x + 630, y + 65), banner_color, 1)

            cv2.putText(canvas, f"CAM [{idx+1}] : {assigned_role} (SN: {serial})", (x + 20, y + 33),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            act_size_str = "640x480 (ACT OK)" if frame.shape == (480, 640, 3) else f"{actual_shape[1]}x{actual_shape[0]}"
            af_status = "ON" if stream.autofocus else "OFF"
            focus_str = f"Focus: {stream.focus_value}"
            cv2.putText(canvas, f"FPS: {fps:.1f} | Res: {act_size_str} | {focus_str} | AF: {af_status}", 
                        (x + 20, y + 55), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0) if stream.autofocus else (200, 200, 200), 1)

        cv2.rectangle(canvas, (0, 915), (1280, 960), (20, 20, 25), -1)
        cv2.putText(canvas, "Press [1-4] Select Cam | [A] Toggle Autofocus | [S] Save Config | [Q] Exit", 
                    (20, 942), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        return canvas

    def _render_focus_view(self, idx):
        canvas = np.zeros((960, 1280, 3), dtype=np.uint8)
        stream = self.streams[idx]
        frame, valid, fps, actual_shape = stream.get_frame()

        if valid:
            canvas = cv2.resize(frame, (1280, 960))
            cv2.line(canvas, (640, 0), (640, 960), (255, 255, 255), 1, cv2.LINE_AA)
            cv2.line(canvas, (0, 480), (1280, 480), (255, 255, 255), 1, cv2.LINE_AA)
            cv2.rectangle(canvas, (160, 120), (1120, 840), (0, 255, 0), 1)
        else:
            cv2.putText(canvas, f"CAMERA {idx+1} DISCONNECTED", (400, 480),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

        cv2.rectangle(canvas, (20, 20), (1260, 110), (0, 0, 0), -1)
        cv2.rectangle(canvas, (20, 20), (1260, 110), (0, 255, 255), 2)
        
        dev_path = self.devices[idx]['by_id_path']
        serial = self.devices[idx]['serial'] or 'UNKNOWN'
        assigned_role = "UNASSIGNED"
        for pos_key, path in self.assignments.items():
            if path == dev_path:
                assigned_role = pos_key.upper().replace('_', ' ')
                break

        cv2.putText(canvas, f"FOCUS MODE: CAM [{idx+1}] -> {assigned_role} (SN: {serial})", (40, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.85, (0, 255, 255), 2)
        cv2.putText(canvas, f"Persistent Path: {dev_path}", (40, 72),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (200, 200, 200), 1)
        
        af_status = "ON (Auto)" if stream.autofocus else "OFF (Fixed)"
        cv2.putText(canvas, f"FPS: {fps:.1f} | Res: 640x480 | Focus Value: {stream.focus_value} | Autofocus: {af_status}", 
                    (40, 96), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0) if stream.autofocus else (0, 255, 255), 1)

        cv2.rectangle(canvas, (20, 880), (1260, 940), (20, 20, 25), -1)
        cv2.rectangle(canvas, (20, 880), (1260, 940), (100, 100, 100), 1)
        cv2.putText(canvas, "ASSIGN ROLE: [F] Front | [L] Left Wrist | [R] Right Wrist | [T] Top",
                    (40, 905), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2)
        cv2.putText(canvas, "FOCUS ADJ: [-]/[+] (-5/+5) | [,/.] (-1/+1) | [0] Grid | [A] Toggle AF | [S] Save Config",
                    (40, 928), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 200), 1)

        return canvas

    def _save_configuration(self):
        role_to_serial = {}
        serial_to_role = {}
        for serial, role in self.camera_serials.items():
            if role and role != "UNASSIGNED":
                serial_to_role[serial] = role
                role_to_serial[role] = serial

        focus_by_serial = {}
        focus_by_role = {}
        focus_by_cam = {}

        for idx, stream in enumerate(self.streams):
            cam_name = f"Cam {idx+1}"
            serial = self.devices[idx]['serial']
            focus_val = stream.focus_value

            focus_by_cam[cam_name] = focus_val
            if serial:
                focus_by_serial[serial] = focus_val
                role = serial_to_role.get(serial)
                if role:
                    focus_by_role[role] = focus_val

        config_data = {
            'timestamp': time.strftime("%Y-%m-%d %H:%M:%S"),
            'target_resolution': [640, 480],
            'autofocus_disabled': True,
            'autofocus_states': {s.name: s.autofocus for s in self.streams},
            'focus_values': focus_by_cam,
            'focus_by_serial': focus_by_serial,
            'focus_by_role': focus_by_role,
            'role_to_serial': role_to_serial,
            'camera_serials': serial_to_role,
            'camera_mapping': self.assignments,
            'devices_detected': self.devices
        }

        saved_any = False
        for path in CONFIG_FILE_PATHS:
            try:
                os.makedirs(os.path.dirname(path), exist_ok=True)
                with open(path, 'w') as f:
                    json.dump(config_data, f, indent=4)
                print(f"✅ Camera Config saved to: {path}")
                saved_any = True
            except Exception as e:
                print(f"⚠️ Could not write config to {path}: {e}")

        if saved_any:
            print("\n" + "★"*60)
            print("  ✅ Camera Serial & Focus Configuration Successfully Saved!")
            print("★"*60 + "\n")

    def cleanup(self):
        print("\nStopping webcam threads...")
        for stream in self.streams:
            stream.stop()
        cv2.destroyAllWindows()
        print("Cleanup complete.")


def main():
    devices = get_v4l_devices()
    if not devices:
        print("❌ No valid video capture devices (/dev/video*) found!")
        sys.exit(1)

    app = WebcamCheckerApp(devices)
    app.run()


if __name__ == '__main__':
    main()
