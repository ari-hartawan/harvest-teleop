import os
import re
import argparse
import shutil

def main():
    parser = argparse.ArgumentParser(description="Rename and move HDF5 episode files, starting from a specific number.")
    parser.add_argument("--src", type=str, required=True, help="Directory path of the source dataset to be renamed.")
    parser.add_argument("--dest", type=str, required=True, help="Directory path where the renamed dataset will be placed.")
    parser.add_argument("--start_idx", type=int, required=True, help="Starting episode index.")
    
    args = parser.parse_args()
    
    if not os.path.exists(args.src):
        print(f"Error: Source directory (src) '{args.src}' not found.")
        return
        
    if not os.path.exists(args.dest):
        print(f"Destination directory (dest) '{args.dest}' not found. Creating a new directory...")
        os.makedirs(args.dest)
        
    # Get all episode files in src
    pattern = re.compile(r"episode_(\d+)\.hdf5")
    src_files = []
    for f in os.listdir(args.src):
        match = pattern.match(f)
        if match:
            idx = int(match.group(1))
            src_files.append((idx, f))
            
    if not src_files:
        print(f"No files with format 'episode_*.hdf5' found in '{args.src}'.")
        return
        
    # Sort files by their original index to ensure correct ordering (0, 1, 2, ...)
    src_files.sort()
    
    print(f"Found {len(src_files)} files. Starting to rename and move to '{args.dest}'...")
    print(f"Numbering will start from episode_{args.start_idx}.hdf5\n")
    
    current_idx = args.start_idx
    
    for idx, filename in src_files:
        new_filename = f"episode_{current_idx}.hdf5"
        
        old_path = os.path.join(args.src, filename)
        new_path = os.path.join(args.dest, new_filename)
        
        # Prevent overwriting files in destination
        if os.path.exists(new_path) and old_path != new_path:
            print(f"Warning: {new_path} already exists! Skipping file {filename}.")
            current_idx += 1
            continue
            
        print(f"  {filename} -> {new_filename}")
        shutil.move(old_path, new_path)
        current_idx += 1
        
    print("\nFinished!")

if __name__ == "__main__":
    main()
