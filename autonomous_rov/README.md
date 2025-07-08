# Autonomous ROV Package 
*Standardized BlueROV control system for MIR students*

![BlueROV Heavy](https://www.bluerobotics.com/store/rov/bluerov2/heavy-config/bluerov2-heavy-image-01.jpg)

## 📦 Package Overview  
A ROS 2 Humble (Ubuntu 22.04) package providing core functionality for BlueROV Heavy configurations. Features include:

- MAVROS integration
- Dual gamepad control
- Camera streaming (mono/stereo)
- PID control node by [Mahmoud Aboelrayat](https://github.com/MahmoudAboelrayat)
- Stereo camera support (experimental)

## 🛠 Hardware Requirements  
| Component | Specification |
|-----------|---------------|
| Vehicle | BlueROV Heavy Configuration |
| Companion Computer | BlueROV Jetson/NUC |
| Cameras | Primary: Built-in • Secondary: USB (see config) |
| Controller | Logitech F710 (or compatible gamepad) |

## 🚀 Quick Start  

### 1. Core Systems  
```bash
# Terminal 1 - MAVROS
ros2 launch autonomous_rov run_mavros.launch

# Terminal 2 - Gamepad Control 
ros2 launch autonomous_rov run_gamepad.launch
```
*⚠️ Note: Set `SYS_MYCGS=1` in ArduPilot when using MAVROS*

### 2. Camera Systems  
#### Single Camera:
```bash
ros2 launch autonomous_rov run_video.launch
```

#### Stereo Setup (Requires pre-config):
```bash
# On BlueOS (first terminal):
gst-launch-1.0 -v v4l2src device=/dev/video6 ! video/x-h264,width=1280,height=720,framerate=30/1 ! rtph264pay ! udpsink host=192.168.2.1 port=5602

# Then on companion computer:
ros2 launch autonomous_rov run_video_stereo.launch
```

### 3. Advanced Control  
```bash
# PID Controller (Mahmoud's implementation)
ros2 launch autonomous_rov run_listener_MIR_pid.launch
```

## ⚙️ Configuration Guide  

### Gamepad Tuning  
Adjust scaling factors in:  
`~/ros2_ws/src/autonomous_rov/launch/run_gamepad.launch`  
*(Restart node after changes)*

### Camera Settings  
| Parameter | File Location |
|-----------|---------------|
| Primary Camera | `run_video.launch` |
| Secondary Camera | `run_video_stereo.launch` |
| Device Paths | Update `joy_dev` in respective launch files |

## � Troubleshooting  

### Common Issues  
1. **Gamepad Not Detected**  
   - Verify device path: `ls /dev/input/js*`  
   - Update `joy_dev` in launch file  

2. **MAVROS Connection Failures**  
   - Confirm `SYS_MYCGS=1` in ArduPilot  
   - Check `~/.ros/setup.bash` for IP conflicts  

3. **Stereo Camera Sync**  
   - Ensure secondary camera is powered before launch  
   - Verify UDP stream on port 5602:  
     ```bash
     tcpdump -i any port 5602
     ```

## 📝 To-Do List  
- [ ] Standardize topic naming convention  
- [ ] Document PID tuning procedure  
- [ ] Add refraction-aware camera calibration  
- [ ] Create system diagram  

## 📜 License & Attribution  
BSD 3-Clause License  
PID Controller: [Mahmoud Aboelrayat](https://github.com/MahmoudAboelrayat)  
Base package: MIR Program  

---

### 🎯 Key Features Table  
| Feature | Launch File | Notes |  
|---------|-------------|-------|  
| MAVROS Link | `run_mavros.launch` | Heartbeat @ 1Hz |  
| Gamepad Control | `run_gamepad.launch` | Dual-stick config |  
| Video Stream | `run_video.launch` | H.264 @ 720p30 |  
| Stereo Vision | `run_video_stereo.launch` | Experimental |  
| PID Control | `run_listener_MIR_pid.launch` | Tune gains in code |  

For advanced users: All launch files support ROS2 parameters - override at runtime with:  
```bash
ros2 launch autonomous_rov run_video.launch camera_id:=4
```