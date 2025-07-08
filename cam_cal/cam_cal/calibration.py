# File: calibrate_underwater_ros2.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml

class CameraCalibrator(Node):
    def __init__(self):
        super().__init__('underwater_camera_calibration')
        self.declare_parameter("image_topic", "/camera")
        self.image_topic = self.get_parameter("image_topic").value

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10)
        
        # Calibration parameters
        self.chessboard_size = (9, 6)
        self.square_size = 0.025
        self.objpoints = []
        self.imgpoints = []
        self.calibration_done = False
        self.latest_frame = None
        self.latest_gray = None
        self.latest_corners = None
        self.latest_found = False

        objp = np.zeros((self.chessboard_size[0]*self.chessboard_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.chessboard_size[0], 0:self.chessboard_size[1]].T.reshape(-1, 2)
        objp *= self.square_size
        self.objp_template = objp

        self.get_logger().info("📷 Press SPACE to capture calibration frame. Press ESC to finish and calibrate.")

    def image_callback(self, msg):
        if self.calibration_done:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)

        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, self.chessboard_size, corners, found)

        # Store the latest valid frame
        self.latest_frame = display
        self.latest_gray = gray
        self.latest_corners = corners
        self.latest_found = found

        cv2.imshow("Calibration", display)
        key = cv2.waitKey(1)

        if key == 27:  # ESC
            if len(self.objpoints) >= 5:
                self.calibrate_camera(self.latest_gray.shape[::-1])
            else:
                self.get_logger().warn("Not enough captures to perform calibration.")
            self.calibration_done = True
            rclpy.shutdown()

        elif key == 32 and found:  # SPACE key
            corners2 = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            self.objpoints.append(self.objp_template)
            self.imgpoints.append(corners2)
            self.get_logger().info(f"Frame {len(self.objpoints)} saved.")

    def calibrate_camera(self, image_size):
        self.get_logger().info("Running calibration...")
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            self.objpoints, self.imgpoints, image_size, None, None)

        fx, fy = mtx[0, 0], mtx[1, 1]
        cx, cy = mtx[0, 2], mtx[1, 2]
        k1, k2, p1, p2 = dist[0][0:4]

        yaml_dict = {
            "File.version": "1.0",
            "Camera.type": "PinHole",
            "Camera1.fx": float(fx),
            "Camera1.fy": float(fy),
            "Camera1.cx": float(cx),
            "Camera1.cy": float(cy),
            "Camera1.k1": float(k1),
            "Camera1.k2": float(k2),
            "Camera1.p1": float(p1),
            "Camera1.p2": float(p2),
            "Camera.width": int(image_size[0]),
            "Camera.height": int(image_size[1]),
            "Camera.newWidth": int(image_size[0] * 0.8),
            "Camera.newHeight": int(image_size[1] * 0.8),
            "Camera.fps": 20,
            "Camera.RGB": 1,
            "ORBextractor.nFeatures": 1000,
            "ORBextractor.scaleFactor": 1.2,
            "ORBextractor.nLevels": 8,
            "ORBextractor.iniThFAST": 20,
            "ORBextractor.minThFAST": 7,
            "Viewer.KeyFrameSize": 0.05,
            "Viewer.KeyFrameLineWidth": 1.0,
            "Viewer.GraphLineWidth": 0.9,
            "Viewer.PointSize": 2.0,
            "Viewer.CameraSize": 0.08,
            "Viewer.CameraLineWidth": 3.0,
            "Viewer.ViewpointX": 0.0,
            "Viewer.ViewpointY": -0.7,
            "Viewer.ViewpointZ": -1.8,
            "Viewer.ViewpointF": 500.0
        }

        with open("camera_calibration.yaml", "w") as f:
            f.write("%YAML:1.0\n\n")
            yaml.dump(yaml_dict, f, default_flow_style=False)

        self.get_logger().info("Calibration complete! Saved to camera_calibration.yaml")
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
