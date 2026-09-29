#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
HDF5 Dataset to Video Converter (10 Episode Sampler)
Features:
  - Extracts the first 10 HDF5 episode files from the dataset folder.
  - Combines 4 camera feeds (Top, Front, Left Wrist, Right Wrist) into a 2x2 grid.
  - Saves each episode as an MP4 video file ready for presentation.
  - Overlays text (Timestep & Episode ID) on the video.
Author: Ari Hartawan
"""

import os
import glob
import h5py
import cv2
import numpy as np

# 📂 ADJUST YOUR DATASET DIRECTORY AND OUTPUT VIDEO DIRECTORY
DATASET_DIR = os.path.expanduser('~/harvest_dataset_unwrapped')
OUTPUT_DIR = os.path.expanduser('~/harvest_dataset_unwrapped/sample_videos_v2')

# Video Configuration
NUM_SAMPLES = 10      # Number of episodes to render
FPS = 10              # 10 Hz according to POLICY_CONTROL_PERIOD (100 ms)


def inspect_and_extract_images(root):
    """Automatically searches for camera image dataset paths inside the HDF5 file."""
    dataset_paths = []
    root.visititems(lambda name, node: dataset_paths.append((name, node.shape)) if isinstance(node, h5py.Dataset) else None)

    image_paths = {}
    for path, shape in dataset_paths:
        path_lower = path.lower()
        if len(shape) == 4:  # Image array shape: (N_frames, H, W, C)
            if any(x in path_lower for x in ['top', 'ensenso', 'high']):
                image_paths['top'] = path
            elif 'left' in path_lower:
                image_paths['left'] = path
            elif 'right' in path_lower:
                image_paths['right'] = path
            elif 'front' in path_lower:
                image_paths['front'] = path

    return image_paths.get('top'), image_paths.get('left'), image_paths.get('right'), image_paths.get('front')


def convert_hdf5_to_video(hdf5_file_path, output_mp4_path):
    print(f"🎬 Processing: {os.path.basename(hdf5_file_path)}...")

    with h5py.File(hdf5_file_path, 'r') as root:
        top_p, left_p, right_p, front_p = inspect_and_extract_images(root)

        if not all([top_p, left_p, right_p, front_p]):
            print(f"❌ Error: Incomplete camera data in {hdf5_file_path}")
            return False

        image_top = root[top_p][:]
        image_left = root[left_p][:]
        image_right = root[right_p][:]
        image_front = root[front_p][:]

    total_frames = image_top.shape[0]
    h, w, _ = image_top[0].shape

    # Combined video size: 4 cameras (2x2 grid)
    # Layout: 
    # [Top Camera] | [Front Camera]
    # [Left Wrist] | [Right Wrist]
    combined_width = w * 2
    combined_height = h * 2

    # Initialize MP4 VideoWriter (MP4V Codec)
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(output_mp4_path, fourcc, FPS, (combined_width, combined_height))

    for t_idx in range(total_frames):
        # Fetch frame
        img_top = cv2.cvtColor(image_top[t_idx], cv2.COLOR_RGB2BGR)
        img_left = cv2.cvtColor(image_left[t_idx], cv2.COLOR_RGB2BGR)
        img_right = cv2.cvtColor(image_right[t_idx], cv2.COLOR_RGB2BGR)
        # Front camera is already in BGR format, so no need to convert RGB to BGR
        img_front = image_front[t_idx].copy()

        # Add Camera Labels to each frame
        cv2.putText(img_left, "Left Wrist", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.putText(img_top, "Top Camera", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.putText(img_right, "Right Wrist", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.putText(img_front, "Front Camera", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        # Combine 4 cameras into 2x2 grid
        top_row = np.hstack([img_top, img_front])
        bottom_row = np.hstack([img_left, img_right])
        combined_frame = np.vstack([top_row, bottom_row])

        # Add Timestep Information / Presentation Overlay at the bottom
        info_text = f"Episode: {os.path.basename(hdf5_file_path)} | Frame: {t_idx + 1}/{total_frames} ({((t_idx+1)/FPS):.1f}s)"
        cv2.putText(combined_frame, info_text, (30, combined_height - 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

        out.write(combined_frame)

    out.release()
    print(f"✅ Finished! Video saved to: {output_mp4_path}\n")
    return True


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # Find all .hdf5 files in the dataset folder
    hdf5_files = sorted(glob.glob(os.path.join(DATASET_DIR, "*.hdf5")))

    if not hdf5_files:
        print(f"🚨 HDF5 files not found in folder {DATASET_DIR}")
        return

    # Take sample of the first 10 episodes
    sampled_files = hdf5_files[:NUM_SAMPLES]
    print(f"🚀 Found {len(hdf5_files)} episodes. Processing the first {len(sampled_files)} episodes...\n")

    for idx, filepath in enumerate(sampled_files):
        filename = os.path.basename(filepath).replace('.hdf5', '.mp4')
        output_path = os.path.join(OUTPUT_DIR, f"sample_{idx+1:02d}_{filename}")
        
        convert_hdf5_to_video(filepath, output_path)

    print(f"🎉 All {len(sampled_files)} videos successfully rendered in folder: {OUTPUT_DIR}")


if __name__ == '__main__':
    main()
