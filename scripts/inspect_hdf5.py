#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
HARVEST Dataset Inspector: Episode 56 Deep-Dive
Analyzes joint angle discontinuities (-pi / +pi jumps) and velocity spikes in HDF5 files.
Author: Ari Hartawan & Gemini
"""

import os
import h5py
import numpy as np
import matplotlib.pyplot as plt

# Path to your episode_56 dataset
DATASET_PATH = os.path.expanduser('~/harvest_dataset/episode_56.hdf5')

# Continuous joint indices for Kinova Gen 3 (Left Arm: 0,2,4,6 | Right Arm: 8,10,12,14)
CONTINUOUS_JOINTS = {
    0: "Left Joint 1", 2: "Left Joint 3", 4: "Left Joint 5", 6: "Left Joint 7",
    8: "Right Joint 1", 10: "Right Joint 3 (Elbow/Wrist)", 12: "Right Joint 5", 14: "Right Joint 7"
}

def inspect_hdf5(file_path):
    if not os.path.exists(file_path):
        print(f"❌ File not found at: {file_path}")
        return

    print("=" * 70)
    print(f"🔍 DEEP INSPECTING DATASET: {file_path}")
    print("=" * 70)

    with h5py.File(file_path, 'r') as root:
        # 1. Search for qpos & action datasets
        qpos = None
        action = None

        if 'observations/qpos' in root:
            qpos = root['observations/qpos'][:]
        elif 'qpos' in root:
            qpos = root['qpos'][:]

        if 'action' in root:
            action = root['action'][:]

        if qpos is None:
            print("❌ Dataset 'qpos' not found in HDF5 file!")
            return

        total_frames, num_dof = qpos.shape
        print(f"📊 Total Frames : {total_frames} Timesteps")
        print(f"🦾 Total DOF    : {num_dof} Joint Degrees of Freedom\n")

        # 2. Analyze Each Continuous Joint
        print("-" * 70)
        print(f"{'JOINT INDEX & NAME':<30} | {'MIN (RAD)':<10} | {'MAX (RAD)':<10} | {'DISCONTINUITY JUMPS (>3.0 rad)'}")
        print("-" * 70)

        discontinuity_found = False

        for idx, j_name in CONTINUOUS_JOINTS.items():
            if idx >= num_dof:
                continue

            j_data = qpos[:, idx]
            j_min, j_max = np.min(j_data), np.max(j_data)
            
            # Calculate difference between consecutive frames (delta)
            deltas = np.abs(np.diff(j_data))
            
            # Detect modular wrap-around jumps typical of -pi / +pi cuts (usually ~6.28 rad / > 3.0 rad in 1 frame)
            jump_indices = np.where(deltas > 3.0)[0]
            num_jumps = len(jump_indices)

            jump_info = f"⚠️ {num_jumps} x Modular Jumps!" if num_jumps > 0 else "✅ Smooth (Continuous)"
            print(f"Joint [{idx:02d}] {j_name:<20} | {j_min:+.3f}      | {j_max:+.3f}      | {jump_info}")

            if num_jumps > 0:
                discontinuity_found = True
                for step_idx in jump_indices:
                    val_before = j_data[step_idx]
                    val_after = j_data[step_idx + 1]
                    print(f"   ↳ 🚨 Jump at Frame [{step_idx:03d} -> {step_idx+1:03d}]: {val_before:+.3f} rad -> {val_after:+.3f} rad (Delta: {deltas[step_idx]:.3f} rad)")

        print("-" * 70)

        # 3. Create visualization plots for Joint 10 (Right Joint 3) and other continuous joints
        fig, axes = plt.subplots(4, 2, figsize=(14, 10), sharex=True)
        fig.suptitle(f"Discontinuity Analysis of Continuous Joints - Episode 56", fontsize=14, fontweight='bold')

        plot_indices = list(CONTINUOUS_JOINTS.keys())
        for i, idx in enumerate(plot_indices):
            row = i // 2
            col = i % 2
            ax = axes[row, col]

            raw_data = qpos[:, idx]
            unwrapped_data = np.unwrap(raw_data)

            ax.plot(raw_data, label='Raw Dataset (HDF5)', color='red', alpha=0.8, linewidth=1.5)
            ax.plot(unwrapped_data, label='Unwrapped True Trajectory', color='blue', linestyle='--', alpha=0.7)

            ax.set_title(f"Joint [{idx}] {CONTINUOUS_JOINTS[idx]}", fontsize=10)
            ax.grid(True, linestyle=':', alpha=0.6)
            if row == 3:
                ax.set_xlabel("Timestep (Frame)")
            ax.set_ylabel("Rad")
            if i == 0:
                ax.legend(loc='upper right', fontsize=8)

        plt.tight_layout()
        output_plot = 'episode_56_analysis.png'
        plt.savefig(output_plot, dpi=150)
        print(f"\n📈 Plot graphic successfully saved to: '{output_plot}'")
        print("   Please open the image to see the difference between RED (Raw HDF5) and BLUE (Unwrapped) trajectories.")

if __name__ == '__main__':
    inspect_hdf5(DATASET_PATH)
