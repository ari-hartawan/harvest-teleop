#!/usr/bin/env python3
"""
HARVEST Framework: Spacebar / Foot-Pedal Driven Safety & Data Collection Trigger Node
Author: Ari Hartawan
Description: Multi-step safety controller with automatic return-to-home before dataset episode save.
             Supports unrecorded live teleop stance positioning before starting dataset recording.
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
        super().__init__('harvest_keyboard_node')
        self.publisher_ = self.create_publisher(String, '/harvest_safety_trigger', 10)
        self.get_logger().info("Harvest Spacebar Safety Trigger Node initialized.")
        
        self.state = 0  # 0: Startup Homing Pending, 2: Homed & Stiff, 3: Live Teleop (Unrecorded), 4: Recording Active
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
            print(f" [HARVEST STARTUP] INITIAL HOMING REQUIRED")
            print(" -> Press [SPACE] / Foot Pedal for INITIAL HOMING (Master & Slave)")
            print("="*65)
        elif self.state == 2:
            print("\n" + "="*65)
            print(f" 🏠 [HOMED AT POSITION] EPISODE #{self.episode_counter}")
            print(" -> Hold the Master Handle firmly.")
            print(" -> Press [SPACE] / Foot Pedal to ENGAGE LIVE TELEOP (Unrecorded) 🕹️")
            print("="*65)
        elif self.state == 3:
            print("\n" + "*"*65)
            print(f" 🕹️  [LIVE TELEOP ACTIVE - NOT RECORDING YET] EPISODE #{self.episode_counter}")
            print(" -> Teleop active! Robot is responding to Master movements.")
            print(" -> Position your hands / robot for the demonstration start stance.")
            print(" -> Press [SPACE] / Foot Pedal when ready to START RECORDING 🔴")
            print("*"*65)
        elif self.state == 4:
            print("\n" + "!"*65)
            print(f" 🔴 [RECORDING ACTIVE] EPISODE #{self.episode_counter}")
            print(" -> Perform the manipulation task demonstration.")
            print(" -> Press [SPACE] / Foot Pedal ANYTIME to AUTO-HOME & SAVE DATASET 💾")
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
                # FIRST TIME INITIAL HOMING
                print("\n[EXECUTION] Running INITIAL HOMING for Master & Slave...")
                self.speak("Initial Homing")
                self.send_trigger("INITIAL_HOME", "Initial Master & Slave Homing")
                time.sleep(2.5)
                self.speak("Homed. Hold master handle and press Space.")
                self.state = 2  # Transition to Homed & Stiff

            elif self.state == 2:
                # ENGAGE LIVE TELEOP (UNRECORDED)
                print("\n[EXECUTION] ENGAGING LIVE TELEOP (Admittance On, Unrecorded Mode)... 🕹️")
                self.speak("Teleop Active")
                self.send_trigger("ENGAGE_TELEOP", "Engage Live Teleop (Unrecorded)")
                time.sleep(0.5)
                self.speak("Position hands and press Space to record.")
                self.state = 3  # Transition to Live Teleop Active (Unrecorded)

            elif self.state == 3:
                # START RECORDING
                print(f"\n[EXECUTION] STARTING RECORDING FOR EPISODE #{self.episode_counter} 🔴")
                self.speak("Start Recording")
                self.send_trigger("START_RECORDING", "Start HDF5 Logging")
                self.state = 4  # Transition to Recording Active

            elif self.state == 4:
                if key_pressed == '0':
                    # CANCEL RECORDING
                    print("\n[EXECUTION] CANCELLED RECORDING: Discarding episode & returning to Home...")
                    self.speak("Recording Cancelled")
                    self.send_trigger("CANCEL_RECORDING", "Cancel Logging & Return to Home")
                    time.sleep(2.5)
                    self.speak("Homed. Hold master handle and press Space.")
                    self.state = 2  # Return to Homed & Stiff
                else:
                    # AUTO HOME & SAVE EPISODE
                    print(f"\n[EXECUTION] AUTO-HOMING MASTER & SLAVE WHILE RECORDING FOR EPISODE #{self.episode_counter}... 🏠")
                    self.speak("Homing and Saving")
                    self.send_trigger("AUTO_HOME_AND_STOP", "Auto Home & Save Episode")
                    
                    # Wait for homing & saving sequence to finish (~2.5 sec)
                    time.sleep(2.5)
                    self.episode_counter += 1
                    self.speak("Episode Saved. Homed. Hold master handle and press Space.")
                    self.state = 2  # Return to Homed & Stiff for next episode!

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