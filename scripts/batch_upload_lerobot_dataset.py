#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Batch Uploader for LeRobot Dataset to Hugging Face.
Uses create_commit to batch all files in a single commit, utilizing cached LFS files.
"""

import os
import time
from huggingface_hub import HfApi, CommitOperationAdd

DATASET_DIR = "/home/an/harvest_lerobot_dataset"
REPO_ID = "ari-hartawan/two_instrument_entangled"

def main():
    print("=" * 70)
    print("🚀 BATCH LEROBOT DATASET UPLOADER ACTIVE")
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
    print(f"📊 Found {total_files} files to commit.")

    operations = []
    for full_path, rel_path in files_to_upload:
        operations.append(
            CommitOperationAdd(path_in_repo=rel_path, path_or_fileobj=full_path)
        )

    print("⏳ Starting batch commit (uploading only missing/new LFS files)...")
    t0 = time.time()
    try:
        api.create_commit(
            repo_id=REPO_ID,
            repo_type="dataset",
            operations=operations,
            commit_message="Upload 102 episodes two_instrument_entangled"
        )
        print(f"🎉 SUCCESS! Batch upload and commit complete in {time.time() - t0:.2f} seconds!")
    except Exception as e:
        print(f"❌ Failed to execute batch commit: {e}")

    print("=" * 70)

if __name__ == '__main__':
    main()
