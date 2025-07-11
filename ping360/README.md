
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

Then run the main node with:

```bash
ros2 run ping360_sonar ping360.py
```

---


once ran, you can see three topics

1. /scan (you can ignore this one) | message used :sensor_msgs/msg/LaserScan
2. /scan_echo : this is where all the data lives : ping360_sonar_msgs/msg/SonarEcho
3. /scan_image : the image of the scan sonar : sensor_msgs/msg/Image


You can add that section like this, using a placeholder for the image path so it’s easy to update later:

---

### Visual Output in Foxglove

Once everything is running, you can visualize the sonar data in **Foxglove Studio**. This provides a clean, real-time interface for inspecting the Ping360 output.

Here’s an example of what the sonar data looks like:

![Ping360 output in Foxglove](images/foxglove_pinger1.png)
