import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import gi
import cv2

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class UDPCameraStreamer(Node):
    def __init__(self):
        super().__init__('udp_camera_streamer')
        self.publisher_ = self.create_publisher(Image, '/camera/right', 10)
        self.bridge = CvBridge()
        self.frame = None

        # Initialize GStreamer
        Gst.init(None)
        self.mainloop = GLib.MainLoop()

        pipeline_str = (
            "udpsrc port=5602 caps=application/x-rtp,media=video,encoding-name=H264,payload=96 ! "
            "rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! appsink name=sink emit-signals=true sync=false"
        )

        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name("sink")
        self.appsink.connect("new-sample", self.on_new_sample)

        self.pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info("GStreamer pipeline started")

        # Timer to handle OpenCV window updates
        self.create_timer(0.03, self.display_frame)  # roughly 30 fps

    def on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        buf = sample.get_buffer()
        caps = sample.get_caps()

        structure = caps.get_structure(0)
        width = structure.get_value("width")
        height = structure.get_value("height")

        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.ERROR

        frame = np.frombuffer(map_info.data, dtype=np.uint8)
        frame = frame.reshape((height, width, 3))  # BGR format

        # Store frame for OpenCV display
        self.frame = frame.copy()

        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(image_msg)

        buf.unmap(map_info)
        return Gst.FlowReturn.OK

    def display_frame(self):
        if self.frame is not None:
            cv2.imshow("UDP Camera Stream", self.frame)
            key = cv2.waitKey(1)
            if key == ord('q'):
                self.get_logger().info("Quitting display...")
                rclpy.shutdown()

    def destroy_node(self):
        self.pipeline.set_state(Gst.State.NULL)
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UDPCameraStreamer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
