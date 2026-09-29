#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
HARVEST Dataset Fast Auto-Fixer (Direct Object Copy)
Fixes qpos & action wrap-around without re-compressing image data.
Completed for 101 episodes in < 10 SECONDS!
Author: Ari Hartawan & Gemini
"""

import os
import glob
import time
import h5py
import numpy as np

SOURCE_DIR = os.path.expanduser('~/harvest_dataset')
TARGET_DIR = os.path.expanduser('~/harvest_dataset_unwrapped')

CONTINUOUS_JOINTS = [0, 2, 4, 6, 8, 10, 12, 14]

def fast_process_single(src_file, dst_file):
    fixed_count = 0

    with h5py.File(src_file, 'r') as f_in, h5py.File(dst_file, 'w') as f_out:
        # 1. Copy Global Attributes
        for k, v in f_in.attrs.items():
            f_out.attrs[k] = v

        # 2. Copy Image Data Directly Without Decompress/Recompress (SUPER FAST!)
        if 'observations/images' in f_in:
            f_in.copy('observations/images', f_out, name='observations/images')
        elif 'images' in f_in:
            f_in.copy('images', f_out, name='observations/images')

        # 3. Read & Unwrap qpos/action Data
        obs_grp = f_out.require_group('observations')

        qpos_path = 'observations/qpos' if 'observations/qpos' in f_in else 'qpos'
        if qpos_path not in f_in:
            return False, 0

        qpos = f_in[qpos_path][:]
        qvel = f_in['observations/qvel'][:] if 'observations/qvel' in f_in else None
        action = f_in['action'][:] if 'action' in f_in else None

        for idx in CONTINUOUS_JOINTS:
            if idx < qpos.shape[1]:
                deltas = np.abs(np.diff(qpos[:, idx]))
                if np.any(deltas > 3.0):
                    fixed_count += 1

                qpos[:, idx] = np.unwrap(qpos[:, idx], axis=0)
                if action is not None and idx < action.shape[1]:
                    action[:, idx] = np.unwrap(action[:, idx], axis=0)

        # 4. Write Smooth Numeric Data
        obs_grp.create_dataset('qpos', data=qpos)
        if qvel is not None:
            obs_grp.create_dataset('qvel', data=qvel)
        if action is not None:
            f_out.create_dataset('action', data=action)

    return True, fixed_count

def run_fast_batch():
    os.makedirs(TARGET_DIR, exist_ok=True)
    pattern = os.path.join(SOURCE_DIR, "episode_*.hdf5")
    files = sorted(glob.glob(pattern))

    if not files:
        print(f"❌ No HDF5 files found at: {SOURCE_DIR}")
        return

    print("=" * 70)
    print(f"⚡ FAST UNWRAPPER ACTIVE (Direct Binary Copy)")
    print(f"📂 SOURCE: {SOURCE_DIR}")
    print(f"💾 TARGET: {TARGET_DIR}")
    print("=" * 70)

    t0 = time.time()
    for i, src_path in enumerate(files):
        filename = os.path.basename(src_path)
        dst_path = os.path.join(TARGET_DIR, filename)
        
        success, num_fixed = fast_process_single(src_path, dst_path)
        if success:
            status = f"✅ FIXED ({num_fixed} joint unwrapped)" if num_fixed > 0 else "ℹ️ COPIED"
            print(f"[{i+1:03d}/{len(files)}] {filename:<20} -> {status}")

    total_time = time.time() - t0
    print("=" * 70)
    print(f"🎉 FINISHED IN ONLY {total_time:.2f} SECONDS!")
    print("=" * 70)

if __name__ == '__main__':
    run_fast_batch()
