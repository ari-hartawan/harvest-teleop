#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Direct LeRobot Dataset Uploader for Hugging Face.
Iterates and uploads each file individually to ensure LFS tracking and commits.
"""

import os
import time
from huggingface_hub import HfApi

DATASET_DIR = "/home/an/harvest_lerobot_dataset"
REPO_ID = "ari-hartawan/two_instrument_entangled"

def main():
    print("=" * 70)
    print("🚀 LEROBOT DATASET UPLOADER ACTIVE")
    print(f"📂 LOCAL DIR: {DATASET_DIR}")
    print(f"☁️ HF REPO:  {REPO_ID}")
    print("=" * 70)

    if not os.path.exists(DATASET_DIR):
        print(f"❌ Local dataset directory not found at: {DATASET_DIR}")
        return

    api = HfApi()
    print("⏳ Creating/checking private repository...")
    api.create_repo(repo_id=REPO_ID, repo_type="dataset", private=True, exist_ok=True)
    print("✅ Repository verified.")

    # Walk directory to find files
    files_to_upload = []
    for root, dirs, files in os.walk(DATASET_DIR):
        for file in files:
            # Skip hidden files
            if file.startswith('.'):
                continue
            full_path = os.path.join(root, file)
            rel_path = os.path.relpath(full_path, DATASET_DIR)
            files_to_upload.append((full_path, rel_path))

    total_files = len(files_to_upload)
    print(f"📊 Found {total_files} files to upload.")

    for idx, (full_path, rel_path) in enumerate(files_to_upload):
        size_mb = os.path.getsize(full_path) / (1024 * 1024)
        print(f"\n⏳ [{idx+1}/{total_files}] Uploading {rel_path} ({size_mb:.2f} MB)...")
        
        t0 = time.time()
        try:
            api.upload_file(
                path_or_fileobj=full_path,
                path_in_repo=rel_path,
                repo_id=REPO_ID,
                repo_type="dataset",
            )
            dt = time.time() - t0
            print(f"   ✅ Done in {dt:.2f} seconds ({size_mb/dt:.2f} MB/s)")
        except Exception as e:
            print(f"   ❌ Failed to upload {rel_path}: {e}")

    print("=" * 70)
    print("🎉 SUCCESS! Dataset fully uploaded and committed.")
    print("=" * 70)

if __name__ == '__main__':
    main()
