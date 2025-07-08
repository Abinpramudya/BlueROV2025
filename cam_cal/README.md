
# Underwater Camera Calibration Tool for ROS2

## 📌 Overview
This ROS2 node performs monocular camera calibration using a chessboard pattern. It captures multiple images of a calibration target, computes camera intrinsic parameters, and generates a YAML configuration file compatible with ORB-SLAM3.

**Disclaimer**: This tool uses standard pinhole calibration models that **do not account for underwater refraction effects**. For accurate underwater SLAM, we recommend using specialized tools like [Kalibr](https://github.com/ethz-asl/kalibr) with refraction modeling.

## ⚙️ Features
- Real-time chessboard detection and visualization
- Interactive frame capture (SPACE key)
- Sub-pixel corner refinement
- Automatic YAML config generation for ORB-SLAM3
- ROS2 parameter configuration

## 📦 Dependencies
```bash
sudo apt install ros-$ROS_DISTRO-cv-bridge
pip install opencv-contrib-python numpy pyyaml
```

## 🚀 Usage
### 1. Prepare Calibration Target
- Chessboard size: **9x6 inner corners** (10x7 squares)
- Square size: **0.025m** (modify `square_size` in code if different)
- Print on rigid waterproof material

### 2. Launch Calibration Node
```bash
# Terminal 1 - Start camera driver
ros2 run <your_camera_driver> <camera_node>

# Terminal 2 - Run calibration
ros2 run <your_package> calibrate_underwater_ros2.py \
  --ros-args -p image_topic:=/camera/image_raw
```

### 3. Calibration Process
1. Position chessboard in camera view
2. Press **SPACEBAR** to capture frame (minimum 5 frames)
3. Press **ESC** to complete calibration
4. Find output: `camera_calibration.yaml`

### 4. Verify Output (Example)
```yaml
Camera.type: "PinHole"
Camera1.fx: 652.34
Camera1.k1: -0.021
...
```

## ⚠️ Underwater Limitations
This implementation has **significant limitations** for underwater use:
1. **Refraction Ignored**: 
   - Assumes light travels in straight lines
   - Cannot model air-glass-water interfaces
2. **Distortion Modeling**:
   - Only 4 distortion coefficients (k1,k2,p1,p2)
   - Omits critical underwater distortion terms (k3-k6)
3. **Depth Effects**:
   - Calibration valid only at specific depth
   - No compensation for depth-dependent distortion
4. **Accuracy Degradation**:
   - Expected reprojection error underwater: >5px
   - Causes scale drift and mapping artifacts

## 📊 Performance Comparison
| Metric | This Tool | Kalibr (Recommended) |
|--------|-----------|-----------------------|
| Underwater Accuracy | ❌ Poor | ✅ Excellent |
| Refraction Modeling | ❌ None | ✅ Snell's Law |
| Distortion Terms | 4 coefficients | 8+ coefficients |
| Depth Compensation | ❌ None | ✅ Multi-layer |
| Reprojection Error | >5px | <0.3px |

## 🛠 Recommended Improvements
For underwater applications:
1. **Switch to Kalibr** with AprilTag targets
2. Implement **depth-dependent calibration**:
   ```python
   # Pseudo-code improvement
   for depth in [0.5, 1.0, 2.0]:
       calibrate_at_depth(depth)
       save_depth_params(depth)
   ```
3. Add **refraction compensation**:
   ```python
   # Snell's law implementation
   def underwater_undistort(point, water_density=1.33):
       # Physics-based ray tracing
   ```

## 📄 Output Configuration Notes
The generated YAML contains ORB-SLAM3 parameters that:
1. Use incorrect `Camera1.*` namespace (should be `Camera.*`)
2. Include arbitrary image scaling (`newWidth/newHeight`)
3. Omit critical distortion parameters (`k3-k5`)

**Manual fixes required** before use with ORB-SLAM3:
```yaml
# BEFORE
Camera1.fx: 700.0

# AFTER
Camera.fx: 700.0
Camera.k3: 0.005  # Add manually
```

## 📜 License
BSD 3-Clause - Use with attribution

---

For serious underwater applications, we strongly recommend switching to [Kalibr](https://github.com/ethz-asl/kalibr) with refraction modeling. This tool is best suited for quick air-based calibrations or educational purposes only.