# HARVEST Teleoperation to LeRobot Dataset Pipeline

This documentation explains how to operate the HARVEST bimanual teleoperation recording system directly integrated with the **Hugging Face LeRobot (v3.0)** dataset format.

This workflow uses a **4-terminal space/foot-pedal driven safety system**, solves USB bandwidth issues using hardware MJPEG compression, and allows flexible task naming and saving paths from Terminal commands.
The system now uses a pure **Trajectory Controller** approach for safer and more stable continuous point-to-point position mirroring.

---

## 🛠️ Key Features of the LeRobot Pipeline

1. **Direct Recording (No Conversion)**: Joint angle numeric data (16D) and camera videos are directly written to LeRobot's Parquet and compressed MP4 formats in real-time.
2. **Trajectory Controller Stability**: Direct point-to-point position trajectory mirroring (continuous spline interpolation) ensures smooth and safe movements without sagging issues, as the controller natively connects trajectory points without forcing zero-velocity stops.
3. **Hardware MJPEG Compression**: 4 Logitech C922 cameras are forced to use MJPEG compression via the V4L2 backend to save USB bandwidth by 90%, allowing all cameras to record simultaneously at 30 FPS.
4. **Dynamic Parameters**: Save location, data frequency (FPS), and language instruction task descriptions can be configured directly from the command line.
5. **Continuous Odd-Joint Auto-Unwrap**: The logger detects wrap-around movements on odd joints (1, 3, 5, 7) for both arms and automatically applies continuous smoothing (`np.unwrap`) before saving. This prevents erratic robot movements during inference.

---

## 🕹️ 4-Terminal Operating Guide

Open 4 terminals on your computer, and run the following commands sequentially:

### 💻 Terminal 1: Launch Dual-Arm Driver (ROS 2)
Activates the physical controllers for the Kinova Gen3 robot arms.
```bash
source /opt/ros/jazzy/setup.bash
source /home/an/workspace/harvest_ws/install/setup.bash

# Physical Arms Launch
ros2 launch kortex_bringup gen3_dual.launch.py \
    robot_ip_1:=192.168.1.10 robot_ip_2:=192.168.1.11 \
    prefix_1:=left_ prefix_2:=right_ \
    use_fake_hardware:=false gripper_1:=robotiq_2f_85 gripper_2:=robotiq_2f_85 launch_rviz:=false
```

### 🕹️ Terminal 2: Launch Teleoperation Core (ROS 2)
Opens the serial connection to the master arms and starts teleoperation motion mapping.
* **Dual Arm (Bimanual)**:
  ```bash
  source /opt/ros/jazzy/setup.bash
  source /home/an/workspace/harvest_ws/install/setup.bash
  
  ros2 launch harvest_teleop harvest.launch.py arm_mode:=both
  ```
* **Single Arm (Right Only)**:
  ```bash
  ros2 launch harvest_teleop harvest.launch.py arm_mode:=right_only
  ```
* **Single Arm (Left Only)**:
  ```bash
  ros2 launch harvest_teleop harvest.launch.py arm_mode:=left_only
  ```

### 📊 Terminal 3: Launch LeRobot Live Logger (Conda + Python)
Opens the camera dashboard visualization, monitors joint telemetry, and logs data directly to LeRobot.
```bash
source /opt/ros/jazzy/setup.bash
source /home/an/workspace/harvest_ws/install/setup.bash
conda activate lerobot_env

python3 /home/an/workspace/harvest_ws/src/harvest_teleop/harvest_teleop/harvest_lerobot_logger.py \
  --ros-args \
  -p dataset_dir:=~/harvest_lerobot_dataset \
  -p record_fps:=30.0 \
  -p arm_mode:=both \
  -p task_description:="perform teleoperation demonstration"
```

#### 💡 Parameter Explanations for Terminal 3:
* **`dataset_dir`**: The destination directory path where your dataset will be saved locally.
* **`record_fps`**: The target frequency for data recording (e.g., `30.0` Hz).
* **`arm_mode`**: The robot arm configuration to record (`both` [16D], `right_only` [8D], or `left_only` [8D]). Single-arm modes will automatically adjust the active wrist cameras and state/action dimensions.
* **`task_description`**: The language instruction / task description that will be embedded into the dataset (e.g., `"pick up the red block"`, `"open the drawer"`).

### ⌨️ Terminal 4: Keyboard Interface (ROS 2)
The spacebar/foot-pedal interface to guide the robot's safety state machine.
```bash
source /opt/ros/jazzy/setup.bash
source /home/an/workspace/harvest_ws/install/setup.bash

ros2 run harvest_teleop keyboard_node
```

---

## 👟 Recording Procedure using the Foot Pedal / Spacebar

Focus on **Terminal 4 (Keyboard Interface)**, then control the state machine using the spacebar or your foot pedal:

1. **Press Space #1 (Initial Homing)**: Both the Master Device and Robot Slave will return to their home positions safely.
2. **Press Space #2 (Engage Live Teleop)**: Unrecorded mode. The robot will now respond to Master movements. Use this state to position your hands and the robot to the starting stance for your demonstration. Data is NOT being recorded yet.
3. **Press Space #3 (Start Recording)**: Terminal 3 will change status to `🔴 LEROBOT REC`. Perform your manipulation task.
4. **Press Space #4 (Stop, Auto-Home & Save)**: The recording stops, the episode is saved to disk, and both the master and slave automatically return to the home position seamlessly.
   * **💡 Cancel Failed Episode (Press `0`)**: If you make a mistake (e.g., drop an object) during recording (Step 3), **press `0` on the keyboard in Terminal 4**. The recording will be canceled immediately, the data will be discarded, and the system will return to the home position.
5. **Next Episode**: The system is back in the Homed state. Rearrange your objects, and press Space again to engage live teleop (Back to Step 2).
6. **Finalization**: After all episodes are recorded, press **`Ctrl+C` in Terminal 3 (Logger)** to allow LeRobot to process the final videos and write global dataset statistics.

---

## ☁️ How to Upload Dataset to Hugging Face Hub

After recording is finalized (Terminal 3 closed with Ctrl+C), you can upload the dataset:

### 1. Login to Hugging Face (Once)
Run this command using your **Write** access token:
```bash
conda activate lerobot_env
huggingface-cli login --token hf_YOUR_TOKEN_HERE
```

### 2. Upload Dataset Folder
```bash
# Disable Xet protocol to avoid transfer errors
export HF_HUB_DISABLE_XET=1

hf upload \
  ari-hartawan/two_instrument_entangled \
  ~/harvest_lerobot_dataset \
  --repo-type=dataset \
  --private
```

---

## 🎥 Visualizing the Dataset

### 1. Online Visualization (Browser)
Visit: **[LeRobot Dataset Visualizer Space](https://huggingface.co/spaces/lerobot/visualize_dataset)**.
Type your repository name (e.g., `ari-hartawan/harvest_dataset_drawer`) to replay it.

### 2. Local Offline Visualization (Streaming / Disk)
```bash
conda activate lerobot_env

# Streaming from Hugging Face
lerobot-dataset-viz --repo-id ari-hartawan/harvest_dataset_drawer

# Reading directly from local disk
lerobot-dataset-viz --repo-id ari-hartawan/harvest_test_dataset --root ~/harvest_lerobot_dataset --mode local --episode-index 0
```

---

## 🗑️ How to Delete a Dataset

### 1. Deleting from Hugging Face Hub (Online)
```bash
conda activate lerobot_env
hf repos delete ari-hartawan/harvest_test_dataset --repo-type=dataset -y
```

### 2. Deleting from Local Disk (Offline)
```bash
rm -rf ~/harvest_lerobot_dataset
```

---

## ✂️ Pruning / Deleting Failed Episodes (Local)

If you have failed episodes, use `lerobot-edit-dataset`.

> [!IMPORTANT]
> **Avoid in-place edits**. Always output to a temporary folder (`_temp`) and swap them to avoid file-locking crashes.

```bash
conda activate lerobot_env

# 1. Edit and write to a temporary folder (e.g. deleting index 35)
lerobot-edit-dataset \
  --repo_id harvest_dataset \
  --root ~/harvest_dataset \
  --new_repo_id harvest_dataset \
  --new_root ~/harvest_dataset_temp \
  --operation.type delete_episodes \
  --operation.episode_indices "[35]"

# 2. Swap folders
rm -rf ~/harvest_dataset
mv ~/harvest_dataset_temp ~/harvest_dataset
```

---

## 🦖 Training & Inference with DINOv2 Backbone (LeRobot)

We support the **DINOv2** (`dinov2_vits14`) visual backbone for the ACT policy in LeRobot, including **16-bit AMP**, **WandB**, and **Tmux** for background training.

### 1. Training in Background (Tmux + WandB + DINOv2)

#### A. Create a new Tmux session:
```bash
tmux new -s train_act
```

#### B. Run the training command:
```bash
conda activate lerobot_env

lerobot-train \
  --dataset.repo_id single_arm_dataset \
  --dataset.root ~/single_arm_dataset \
  --policy.type act \
  --policy.vision_backbone dinov2_vits14 \
  --policy.use_amp true \
  --policy.push_to_hub false \
  --batch_size 16 \
  --steps 100000 \
  --save_freq 20000 \
  --wandb.enable true \
  --wandb.project harvest_single_arm \
  --output_dir outputs/train/act_dinov2_single_arm_100k
```
*(Check outputs in `outputs/train/act_dinov2_single_arm_100k/checkpoints/`)*

#### C. Detach from Tmux:
Press **`Ctrl + B`**, then release and press **`D`**.

#### D. Re-attach to check progress:
```bash
tmux attach -t train_act
```

### 2. Offline Simulation Evaluation (RViz Dry-Run)

1. **Terminal 1**: Launch fake hardware
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/workspace/ros2_kortex_jazzy_ws/install/setup.bash
   ros2 launch kortex_bringup gen3_dual.launch.py use_fake_hardware:=true launch_rviz:=true
   ```
2. **Terminal 2**: Run dry-run script
   ```bash
   conda activate lerobot_env
   python3 ~/workspace/act_ws/eval_dryrun_lerobot.py
   ```

### 3. Real Robot Physical Inference

1. **Terminal 1**: Launch real hardware
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/workspace/ros2_kortex_jazzy_ws/install/setup.bash
   ros2 launch kortex_bringup gen3_dual.launch.py use_fake_hardware:=false launch_rviz:=false
   ```
2. **Terminal 2**: Run physical evaluation script
   ```bash
   conda activate lerobot_env
   python3 ~/workspace/act_ws/eval_real_robot_lerobot.py
   ```
   * **Emergency Stop**: Press **`ESC`** or **`q`** in the camera video window to instantly trigger the emergency brake.
