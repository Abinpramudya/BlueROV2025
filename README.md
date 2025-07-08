# BlueROV2025

Welcome to **BlueROV2025** — a repository for all BlueROV experiments conducted during my **Summer Internship 2025**.

This project is currently a work in progress and will be updated as development continues. It contains a collection of ROS 2 Humble packages designed to work with the **BlueROV Heavy configuration**.

> ⚠️ A full and proper README is coming soon... *assuming I don’t get too lazy!*

---

## ✅ Project Status

| Package Name         | Status        | Comment                                                          |
| -------------------- | ------------- | ---------------------------------------------------------------- |
| `autonomous_rov`     | ✅ Done        | Main package for communication and control of BlueROV            |
| `camera_calibration` | ✅ Done (kinda)       | Underwater camera calibration using OpenCV + chessboard patterns(i still want to add kalibr into this)|
| `pinger360`          | ❌ In Progress | Code available, documentation is still pending and i will add result too                   |
| `orbslam3`           | ❌ In Progress | Actively working on integration and testing (this work has been the bane to my existence, may god help me)|

---

## 🛠️ How to Use

To get started, clone this repo into your ROS 2 workspace:

```bash
cd ~/your_ws/src/
git clone <this-repo-url>
```

Then build the workspace:

```bash
cd ~/your_ws/
colcon build
```

> Make sure you're using:
>
> * ROS 2 Humble
> * Ubuntu 22.04
> * BlueROV Heavy configuration

---

## 📌 TODO

* [ ] Complete and clean up `pinger360` documentation + result of experiments !
* [ ] add `kalibr` as another option for calibration
* [ ] Finalize and test `orbslam3` integration into ros2 with proper documentation
* [ ] Add launch files and example bags
* [ ] Write full documentation for each package

---

Stay tuned for updates — and if you’re working on similar projects, feel free to fork or open issues! 😄
