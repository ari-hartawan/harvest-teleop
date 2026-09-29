#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
HARVEST Dataset Auto-Fixer for LeRobot (v3.0 Parquet format)
Applies np.unwrap on continuous joints for all existing recorded episodes in place.
Recomputes stats automatically at the end.
"""

import os
import shutil
import pandas as pd
import numpy as np
import subprocess

DATASET_DIR = os.path.expanduser('~/harvest_lerobot_dataset')
BACKUP_DIR = os.path.expanduser('~/harvest_lerobot_dataset_pre_unwrap_backup')
PARQUET_FILE = os.path.join(DATASET_DIR, 'data/chunk-000/file-000.parquet')
CONTINUOUS_JOINTS = [0, 2, 4, 6, 8, 10, 12, 14]

def main():
    print("=" * 70)
    print("⚡ LEROBOT DATASET UNWRAPPER ACTIVE")
    print(f"📂 TARGET: {DATASET_DIR}")
    print("=" * 70)

    if not os.path.exists(DATASET_DIR):
        print(f"❌ Dataset directory not found at: {DATASET_DIR}")
        return

    # 1. Create a safety backup
    print("⏳ Creating safety backup...")
    if os.path.exists(BACKUP_DIR):
        shutil.rmtree(BACKUP_DIR)
    shutil.copytree(DATASET_DIR, BACKUP_DIR)
    print(f"✅ Backup created at: {BACKUP_DIR}")

    # 2. Read the Parquet file
    if not os.path.exists(PARQUET_FILE):
        print(f"❌ Parquet file not found at: {PARQUET_FILE}")
        return

    print("⏳ Reading Parquet data...")
    df = pd.read_parquet(PARQUET_FILE)
    print(f"✅ Loaded {len(df)} frames across {df['episode_index'].nunique()} episodes.")

    # 3. Extract states and actions columns to numpy arrays
    states = np.vstack(df["observation.state"].values)
    actions = np.vstack(df["action"].values)

    # 4. Perform unwrap episode by episode
    print("⏳ Unwrapping continuous joints...")
    fixed_episodes_count = 0
    
    unique_episodes = df["episode_index"].unique()
    for ep_idx in unique_episodes:
        mask = df["episode_index"] == ep_idx
        ep_states = states[mask]
        ep_actions = actions[mask]

        has_jumps = False
        for joint_idx in CONTINUOUS_JOINTS:
            if joint_idx < ep_states.shape[1]:
                # Check if there are jumps larger than pi
                deltas = np.abs(np.diff(ep_states[:, joint_idx]))
                if np.any(deltas > 3.0):
                    has_jumps = True
                
                # Unwrap
                ep_states[:, joint_idx] = np.unwrap(ep_states[:, joint_idx], axis=0)
                
            if joint_idx < ep_actions.shape[1]:
                ep_actions[:, joint_idx] = np.unwrap(ep_actions[:, joint_idx], axis=0)

        if has_jumps:
            fixed_episodes_count += 1
            print(f"   Episode #{ep_idx:03d} -> corrected joint jumps.")

        states[mask] = ep_states
        actions[mask] = ep_actions

    # Convert back to list format for pandas Parquet writing
    df["observation.state"] = list(states)
    df["action"] = list(actions)

    # 5. Save Parquet back to disk
    print("⏳ Writing unwrapped data back to Parquet...")
    df.to_parquet(PARQUET_FILE)
    print("✅ Parquet file updated.")

    # 6. Recompute LeRobot stats
    print("⏳ Recomputing LeRobot dataset statistics (info.json / stats.json)...")
    try:
        cmd = [
            "lerobot-edit-dataset",
            "--repo_id", "ari-hartawan/harvest_test_dataset",
            "--root", DATASET_DIR,
            "--operation.type", "recompute_stats"
        ]
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        print("✅ Statistics successfully recomputed!")
    except Exception as e:
        print(f"⚠️ Warning: Could not automatically recompute stats via CLI: {e}")
        print("Please run manually: lerobot-edit-dataset --repo_id ari-hartawan/harvest_test_dataset --root ~/harvest_lerobot_dataset --operation.type recompute_stats")

    print("=" * 70)
    print(f"🎉 SUCCESS! Fixed {fixed_episodes_count} episodes.")
    print("=" * 70)

if __name__ == '__main__':
    main()
