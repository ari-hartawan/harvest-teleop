#!/usr/bin/env python3
"""
HARVEST Framework: Spacebar / Foot-Pedal Driven Safety & Data Collection Trigger Node
Author: Ari Hartawan (Modified for Velocity Controller)
Description: 5-step safety controller with manual controller switching.
             Designed to work with teleop_node_velocity.py as per README_LEROBOT.md specifications.
"""

import sys
import os
import time
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def get_single_key():
    """Reads a single keyboard key press immediately without pressing Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class HarvestSpaceTriggerNode(Node):
    def __init__(self):
        super().__init__('harvest_keyboard_node_velocity')
        self.publisher_ = self.create_publisher(String, '/harvest_safety_trigger', 10)
        self.get_logger().info("Harvest Spacebar Safety Trigger Node (Velocity Mode) initialized.")
        
        self.state = 0  # 0: Standby, 1: Master Homed, 2: Slave Homed, 3: Live Teleop (Velocity), 4: Recording
        self.episode_counter = 1

        self.control_loop()

    def speak(self, text):
        os.system(f"spd-say -p 10 -t female1 '{text}' &")

    def play_beep(self):
        sys.stdout.write('\a')
        sys.stdout.flush()

    def send_trigger(self, trigger_cmd, desc=""):
        msg = String()
        msg.data = trigger_cmd
        self.publisher_.publish(msg)
        if desc:
            self.get_logger().info(f"Trigger Sent: {desc}")

    def print_menu(self):
        if self.state == 0:
            print("\n" + "="*65)
            print(f" [STEP 1] STANDBY / STARTUP")
            print(" -> Press [SPACE] / Foot Pedal to HOME MASTER DEVICE")
            print("="*65)
        elif self.state == 1:
            print("\n" + "="*65)
            print(f" [STEP 2] MASTER HOMED")
            print(" -> Press [SPACE] / Foot Pedal to HOME SLAVE ROBOT (Trajectory Mode)")
            print("="*65)
        elif self.state == 2:
            print("\n" + "="*65)
            print(f" [STEP 3] SLAVE HOMED (STIFF)")
            print(" -> Hold the Master Handle firmly.")
            print(" -> Press [SPACE] to SWITCH CONTROLLER to VELOCITY & ENGAGE TELEOP 🕹️")
            print("="*65)
        elif self.state == 3:
            print("\n" + "*"*65)
            print(f" [STEP 4] 🕹️ LIVE TELEOP ACTIVE (VELOCITY MODE) - NOT RECORDING")
            print(" -> Teleop active! Robot is responding to Master movements.")
            print(" -> Position your hands / robot for the demonstration start stance.")
            print(f" -> Press [SPACE] to START RECORDING EPISODE #{self.episode_counter} 🔴")
            print("*"*65)
        elif self.state == 4:
            print("\n" + "!"*65)
            print(f" [STEP 5] 🔴 RECORDING ACTIVE (EPISODE #{self.episode_counter})")
            print(" -> Perform the manipulation task demonstration.")
            print(" -> Press [SPACE] ANYTIME to STOP RECORDING, SAVE, & LOCK ROBOT 💾")
            print(" -> Press [0] ANYTIME to CANCEL RECORDING (Discard episode & Reset) 🗑️")
            print("!"*65)

    def control_loop(self):
        while rclpy.ok():
            self.print_menu()
            
            key_pressed = None
            while True:
                try:
                    key = get_single_key()
                    if key in [' ', '\r', '\n']:
                        key_pressed = 'SPACE'
                        break
                    elif key == '0' and self.state == 4:
                        key_pressed = '0'
                        break
                    elif key == '\x03':  # Ctrl+C
                        sys.exit(0)
                except Exception:
                    break

            self.play_beep()

            if self.state == 0:
                print("\n[EXECUTION] STEP 1: Homing Master Device...")
                self.speak("Homing Master")
                self.send_trigger("STEP_1", "Master Homing")
                time.sleep(1.0)
                self.state = 1

            elif self.state == 1:
                print("\n[EXECUTION] STEP 2: Homing Robot Slave...")
                self.speak("Homing Slave")
                self.send_trigger("STEP_2", "Robot Slave Homing")
                time.sleep(2.5)
                self.speak("Homed. Hold master handle and press Space.")
                self.state = 2

            elif self.state == 2:
                print("\n[EXECUTION] STEP 3: Switching Controllers and Engaging Teleop...")
                self.speak("Switching to Velocity Controller")
                self.send_trigger("STEP_3", "Switch to Velocity Controller & Engage Teleop")
                
                # Menunggu stabilisasi 3 detik sesuai dokumentasi
                print("Waiting 3 seconds for stabilization...")
                time.sleep(3.0)
                
                self.speak("Teleop Active. Position hands and press Space to record.")
                self.state = 3

            elif self.state == 3:
                print(f"\n[EXECUTION] STEP 4: STARTING RECORDING FOR EPISODE #{self.episode_counter} 🔴")
                self.speak("Start Recording")
                self.send_trigger("START_RECORDING", "Start HDF5 Logging")
                self.state = 4

            elif self.state == 4:
                if key_pressed == '0':
                    print("\n[EXECUTION] CANCELLED RECORDING: Discarding episode & Locking Robot...")
                    self.speak("Recording Cancelled")
                    self.send_trigger("CANCEL_RECORDING", "Cancel Logging & Lock Robot")
                    time.sleep(1.5)
                    self.speak("Robot Locked. Returning to Standby.")
                    self.state = 0
                else:
                    print(f"\n[EXECUTION] STEP 5: STOPPING RECORDING & LOCKING ROBOT FOR EPISODE #{self.episode_counter}... 💾")
                    self.speak("Saving Episode and Locking Robot")
                    self.send_trigger("STOP_RECORDING", "Stop Logging & Lock Robot")
                    
                    time.sleep(1.5)
                    self.episode_counter += 1
                    self.speak("Episode Saved. Robot Locked. Returning to Standby.")
                    self.state = 0

def main(args=None):
    rclpy.init(args=args)
    node = HarvestSpaceTriggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
