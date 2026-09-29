# pyrefly: ignore [missing-import]
import h5py
import cv2
import numpy as np
import os
import sys

# Specify the HDF5 file to check
if len(sys.argv) > 1:
    DATASET_PATH = os.path.expanduser(sys.argv[1])
else:
    DATASET_PATH = os.path.expanduser("~/harvest_dataset/episode_0.hdf5")

def main():
    if not os.path.exists(DATASET_PATH):
        print(f"❌ File not found: {DATASET_PATH}")
        sys.exit(1)

    print(f"Opening {DATASET_PATH} ...\n")
    with h5py.File(DATASET_PATH, 'r') as root:
        # Read qpos and image datasets according to the correct HDF5 structure
        qpos = root['/observations/qpos'][()]
        cam_top = root['/images/cam_top'][()]
        cam_left = root['/images/cam_left_wrist'][()]
        cam_right = root['/images/cam_right_wrist'][()]
        has_front = '/images/cam_front' in root
        if has_front:
            cam_front = root['/images/cam_front'][()]
        else:
            cam_front = None

    num_frames = qpos.shape[0]
    print(f"✅ Successfully loaded total {num_frames} frames.")

    def on_trackbar(val):
        pass

    cv2.namedWindow("HDF5 Sync Inspector", cv2.WINDOW_NORMAL)
    if has_front:
        cv2.resizeWindow("HDF5 Sync Inspector", 1280, 960)
    else:
        cv2.resizeWindow("HDF5 Sync Inspector", 1920, 480)
    cv2.createTrackbar("Frame", "HDF5 Sync Inspector", 0, num_frames - 1, on_trackbar)

    print("\n--- CONTROLS ---")
    print("Drag the trackbar with the mouse to inspect frame-by-frame.")
    print("Press 'D' for the next frame (Next).")
    print("Press 'A' for the previous frame (Prev).")
    print("Press 'ESC' or 'Q' to exit.\n")

    while True:
        idx = cv2.getTrackbarPos("Frame", "HDF5 Sync Inspector")

        img_top = cam_top[idx]
        img_left = cam_left[idx]
        img_right = cam_right[idx]

        # Based on qpos index layout: 0-6: Left Joints, 7: Left Gripper, 8-14: Right Joints, 15: Right Gripper
        left_gripper_val = qpos[idx, 7]
        right_gripper_val = qpos[idx, 15]

        if has_front and cam_front is not None:
            img_front = cam_front[idx]
            # Combine 4 images into a 2x2 grid
            row1 = np.hstack([img_left, img_right])
            row2 = np.hstack([img_top, img_front])
            dashboard = np.vstack([row1, row2])

            # Overlay telemetry text on the image
            cv2.putText(dashboard, f"FRAME: {idx} / {num_frames-1}", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            cv2.putText(dashboard, f"L_GRIPPER: {left_gripper_val:.4f}", (20, 80), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            
            cv2.putText(dashboard, f"R_GRIPPER: {right_gripper_val:.4f}", (640 + 20, 80), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        else:
            # Combine 3 images horizontally
            dashboard = np.hstack([img_left, img_top, img_right])

            # Overlay telemetry text on the image
            cv2.putText(dashboard, f"FRAME: {idx} / {num_frames-1}", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            cv2.putText(dashboard, f"L_GRIPPER: {left_gripper_val:.4f}", (20, 80), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            
            cv2.putText(dashboard, f"R_GRIPPER: {right_gripper_val:.4f}", (1280 + 20, 80), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        cv2.imshow("HDF5 Sync Inspector", dashboard)

        key = cv2.waitKey(20) & 0xFF
        if key == 27 or key == ord('q'):
            break
        elif key == ord('d'):
            cv2.setTrackbarPos("Frame", "HDF5 Sync Inspector", min(idx + 1, num_frames - 1))
        elif key == ord('a'):
            cv2.setTrackbarPos("Frame", "HDF5 Sync Inspector", max(idx - 1, 0))

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
