# Stereo Vision 3D Measurement & Point Cloud Project

This project provides a complete pipeline for stereo vision 3D measurement and point cloud generation using C++, OpenCV, PCL, and the ZED SDK.  
It is designed for researchers, engineers, and students working with stereo cameras (like ZED2) and 3D scene analysis.

---

## Features

- **Stereo Video Processing:**  
  Handles ZED2 side-by-side stereo video (2560x720 → 2×1280x720).
- **Calibration Support:**  
  Uses calibration/configuration from `stereo.yaml` for accurate rectification and depth.
- **Disparity & Depth Map Computation:**  
  Computes disparity maps and reprojects to 3D using OpenCV.
- **Interactive 3D Measurement:**  
  GUI tool for selecting points and measuring real-world distances in the scene.
- **Point Cloud Generation:**  
  Converts depth/disparity to colored 3D point clouds, with downsampling (VoxelGrid).
- **PCD File Saving & Visualization:**  
  Saves point clouds as `.pcd` files and displays them with PCL’s viewer.
- **Batch Processing:**  
  Easily process multiple frames or videos.

---

## Project Structure
wayland_thirdProject/ ├── app/ │ └── stereo_ruler.cpp # Main 3D measurement tool ├── point_cloud/ │ └── src/pcd_write.cpp # Point cloud generation ├── stereo_vision/ │ ├── include/ # Stereo vision headers │ └── src/ # Stereo vision source files ├── assets/ │ ├── output.mp4 # Example stereo video (side-by-side) │ ├── cam.mp4 # Example camera video │ └── stereo.yaml # Calibration/config file ├── results/ # Output folder for measurements and PCD files ├── Dockerfile # (Optional) Docker build instructions ├── .dockerignore ├── .gitignore └── README.md

## How It Works

1. **Calibration:**  
   Load stereo calibration from `assets/stereo.yaml` for rectification and 3D mapping.
2. **Frame Extraction:**  
   Read stereo video, split each frame into left/right images.
3. **Rectification:**  
   Apply calibration to align images for accurate stereo matching.
4. **Disparity Computation:**  
   Use OpenCV’s SGBM to compute disparity between left/right images.
5. **3D Reprojection:**  
   Convert disparity to 3D coordinates using the Q matrix.
6. **Interactive Measurement:**  
   - Freeze a frame, select two points, and measure real-world distance.
   - Save measurements to CSV for later analysis.
7. **Point Cloud Generation:**  
   - Colorize 3D points with the left image.
   - Downsample with a 5mm voxel grid.
   - Save as `.pcd` and visualize in 3D.

---

## Usage

### **Stereo Ruler (3D Measurement Tool)**

- **Start the program:**  
./build/app/stereo_ruler


- **Controls:**
- **ESC:** Exit
- **F:** Freeze current frame for measurement
- **A:** Return to video playback
- **S:** Save measurements to CSV
- **R:** Reset current measurements
- **N:** Start new measurement session
- **Shift+Click:** Select two points to measure distance

- **Output:**  
- Measurements saved to `results/measurements.csv`
- Overlays shown on the video window

---

### **Point Cloud Generation**

- **Start the program:** 
./build/point_cloud/point_cloud


- **What it does:**
- Processes a stereo frame
- Computes disparity and reprojects to 3D
- Downsamples and saves the point cloud as `results/frame_xxxxx.pcd`
- Opens a 3D viewer window for visualization

---

## Configuration

- **Input videos and calibration:**  
Place your stereo video(s) and `stereo.yaml` in the `assets/` folder.
- **Paths in code:**  
All code expects files at `assets/output.mp4`, `assets/cam.mp4`, and `assets/stereo.yaml`.

---

## Troubleshooting

- **Video file not found:**  
Make sure your video is in `assets/` and the path in code matches.
- **No GUI windows:**  
Ensure you are running on a system with a display (not headless).
- **Permission errors:**  
Make sure you have read/write access to the `results/` folder.

---

## Credits

- [OpenCV](https://opencv.org/)
- [Point Cloud Library (PCL)](https://pointclouds.org/)
- [Stereolabs ZED SDK](https://www.stereolabs.com/)
