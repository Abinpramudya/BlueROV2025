
---

# Ping360 Package Collection

This directory contains a set of ROS 2 packages for working with the **Ping360 imaging sonar**. This project has been the main focus of my internship, where I explored how to operate and integrate this sensor in real-world scenarios.

### Sensor Information

* **Device**: [Blue Robotics Ping360 Sonar](https://bluerobotics.com/store/sonars/imaging-sonars/ping360-sonar-r1-rp/)
* **Documentation**: [Official Product Page](https://bluerobotics.com/store/sonars/imaging-sonars/ping360-sonar-r1-rp/)

### Acknowledgements

Based on the work by [Centrale Nantes Robotics](https://github.com/CentraleNantesRobotics/ping360_sonar/), which provided the core codebase for the sonar interface.

### Included Packages

* `ping360_sonar`: Main ROS 2 node for Ping360
* `ping360_sonar_msgs`: Custom message definitions used by the node
* `ping_sonar_ros-master`: Prerequisite utility package from the MIR program

### Project Overview

These packages are tested with a **BlueROV2 Heavy Configuration**, particularly in a controlled water tank environment. The goal is to evaluate how the Ping360 performs under different conditions and explore various configuration strategies.

### Recommended Tools

* `rqt`: For easy GUI-based ROS visualization
* `Foxglove Studio`: For a cleaner and more modern view of sonar data

---

## Getting Started

### 1. Setup

Place all the listed packages into your ROS 2 workspace (`src` folder), then build and source:

```bash
colcon build
source install/setup.bash
```

### 2. First-Time Run

Before running the node, check that the sensor is properly connected:

1. Open your BlueROV’s web interface: `http://192.168.2.2/` (or your specific IP)
2. Go to the **Ping360** tab and ensure the sensor is detected and responsive

![Ping360 output in Foxglove](images/blueOS_ping.png)

if everything is in order, you can ran the code with following command, make sure everything has been built and sourced properly.


You can run the code with the following code
```bash
ros2 run ping360_sonar ping360.py
```

---

### ROS 2 Topics

Once the node is running, you’ll see the following topics:

1. **`/scan`** – Can be ignored; publishes empty or unused data
   • Message type: `sensor_msgs/msg/LaserScan`

2. **`/scan_echo`** – Main data stream containing raw sonar echo information
   • Message type: `ping360_sonar_msgs/msg/SonarEcho`

3. **`/scan_image`** – Visualization of the sonar scan as an image
   • Message type: `sensor_msgs/msg/Image`

> You can use tools like `rqt`, `ros2 topic echo`, or **Foxglove Studio** to visualize these topics in real-time.


You can add that section like this, using a placeholder for the image path so it’s easy to update later:

---

### Visual Output in Foxglove

Once everything is running, you can visualize the sonar data in **Foxglove Studio**. This provides a clean, real-time interface for inspecting the Ping360 output.

Here’s an example of what the sonar data looks like:

![Ping360 output in Foxglove](images/foxglove_pinger1.png)



# A Deeper Dive
You can use the **RQT user interface** to modify various parameters of the Ping360 sonar during runtime. However, if you want to set **default parameters**, you’ll need to modify the source code directly.

Open the file `ping360.py` located inside the `ping360_sonar` package under the `src` folder. Focus on the section where the default parameters are defined:

```python
parameters = {
    'gain': [0, 0, 2],
    'frequency': 740,
    'angle_sector': [360, 60, 360],
    'scan_threshold': [200, 0, 255],
    'angle_step': [1, 1, 20],
    'image_size': [200, 200, 1000],
    'image_rate': [50, 50, 2000],
    'speed_of_sound': [1500, 1000, 2000],
    'range_max': [6, 1, 50],
    'publish_image': True,
    'publish_scan': True,
    'publish_echo': True,
    'image_min_threshold': [10, 0, 255],
}
```
## ⚙️ Parameter Descriptions

### 1. `gain`

* Controls the **intensity (sensitivity)** of the sonar.
* A higher value makes the sonar more sensitive but may also increase noise.

### 2. `frequency`

* The **base frequency** of the sonar in kHz.
* Usually left at the default value unless fine-tuning for a specific environment.

### 3. `angle_sector`

* Defines the **scanning angle range**.
* The sonar rotates using a stepper motor, so you can limit the scan to a forward-facing arc (e.g., 60° to 360°).

### 4. `scan_threshold`

* Sets the minimum signal strength (echo intensity) required for a point to be considered valid.
* Possibly sets a threshold for valid return signals; needs confirmation.

### 5. `angle_step`

* Defines the **angular resolution** of the scan.
* Smaller steps → higher resolution but slower scan.
* Larger steps → faster scan but lower detail.

### 6. `image_size`

* defines the pixel resolution or dimensions of the sonar image.

### 7. `image_rate`

* controls how frequently images are generated or updated.
* Higher rate → more frequent updates, but increased CPU/network usage

### 8. `speed_of_sound`

* Critical for accurate range measurements.
* Refer to environmental documentation: **salinity, temperature, and pressure** significantly affect this value.

### 9. `range_max`

* Sets the **maximum detection range** of the sonar in meters.
* Adjust based on your operational needs (e.g., `6`, `10`, or `20` meters).

### 10. `publish_*` flags (`image`, `scan`, `echo`)

* Enable or disable publishing of sonar data on ROS topics.
* Useful for reducing network load or debugging specific outputs.

### 11. `image_min_threshold`

* A custom filtering parameter to **remove low-intensity noise** from images.
* Values below this threshold are blacked out, helping to visually isolate relevant objects.

## 📌 Notes

* Values can be adjusted through **RQT** during runtime.
* If you're unsure about a parameter (e.g., `scan_threshold`, `image_size`, `image_rate`), it's best to consult the **Ping360 BlueRobotics Documentation** or experiment incrementally.
* Visualization samples and performance comparisons (e.g., angle step effects) can be added below.

Sure! Here's the content rewritten as a polished `README.md` section for including the rosbag replay instructions:

---

# 🎞️ Offline Data Replay with ROS Bag

If you want to **replay the sonar data locally** without connecting to the actual robot, you can use the pre-recorded **ROS bag** file provided below.

You can download the bag file from this Google Drive folder:

👉 [ROS Bag Folder](https://drive.google.com/drive/folders/1jmrwAuXjsGBp1a2yrn4YiLzvOXilutw_?usp=drive_link)

