# Situational Awareness

This project demonstrates a **sensor fusion** that integrates:
- A **128-beam LiDAR**
- An **Intel RealSense RGB-D camera**
- A **pretrained semantic segmentation model (DeepLabV3 ResNet50)**

The output is a 2D **occupancy grid map** at a resolution of **0.1 m/cell**, published at **10Hz** or more on a **Jetson Orin NX** running ROS2 (`rclpy`).

---

## 🚀 Features
- Fuses LiDAR and RGB-D data for real-time environment understanding.
- Uses semantic segmentation to classify obstacles from RGB image.
- Marks **free space** and **obstacles** on a 100x100 occupancy grid.
- Lightweight implementation: under 60 lines of Python code (excluding comments/imports).
- Compatible with **ROS2** and hardware-accelerated using **CUDA (GPU)**.

---

## 📁 Topic Subscriptions
| Topic | Type | Purpose |
|-------|------|---------|
| `/camera_color_image_raw` | `sensor_msgs/Image` | RGB image for semantic segmentation |
| `/camera_depth_image_raw` | `sensor_msgs/Image` | Depth image used for range filtering |
| `/lidar_points` | `sensor_msgs/PointCloud2` | LiDAR data to enhance obstacle mapping |

---

## 📤 Published Topic
| Topic | Type |
|-------|------|
| `/map` | `nav_msgs/OccupancyGrid` |

---

## 🧠 Semantic Segmentation
The code uses a pretrained `DeepLabV3_ResNet50` model from TorchHub for segmentation.
- Output classes are used to distinguish **obstacles (class > 0)** vs. **free space (class 0)**.
- Only depth values in the range **1.0m–5.0m** are used.

---

## 🧪 Mock Data Support

This project includes a **mock environment generator** to simulate camera and LiDAR input for testing.

### 🧪 Mock Publishers:
| Node | Description |
|------|-------------|
| `MockCameraPublisher` | Publishes synthetic RGB (with red box obstacles) and constant-depth images |
| `MockLidarPublisher` | Publishes random 2D point clouds simulating obstacles in front of robot |

### 🧩 RGB Image:
- Size: 640x480
- Red box in center: interpreted as an obstacle by the segmentation model
- Timestamped and published on `/camera_color_image_raw` (format: `rgb8`)

### 🧩 Depth Image:
- Default value: `2.0 m` for entire frame
- Published on `/camera_depth_image_raw` (format: `32FC1`)

### 🧩 LiDAR:
- Simulates a semi-circular arc of 3D points (x, y, z=0)
- Points range from `1.0 m` to `4.0 m` in front of robot
- Published on `/lidar_points` as `sensor_msgs/PointCloud2`

---

## 🛠 What’s Missing for a Real Project
> The following should be added for this to be deployment-ready:

1. **Installation instructions**:
   - Python environment setup
   - ROS2 (e.g. Humble or Foxy) installation
   - `cv_bridge`, `torch`, `rclpy`, `sensor_msgs` dependencies, `setup.py`, `package.xml`, `launch files` and etc.  
2. **Camera and LiDAR calibration**:
   - Currently assumes perfect alignment; real-world deployment requires extrinsics calibration.
3. **Time synchronization**:
   - RGB, depth, and LiDAR messages should be synced.
4. **Error handling & logging**:
   - Logging should replace `print()` or `pass`, and GPU errors should be caught.
5. **Performance monitoring**:
   - FPS tracking and ROS diagnostics could help in validation.

---

## 🧩 Notes
- Run this node with a ROS2 launch file or directly via `python3`.

---
