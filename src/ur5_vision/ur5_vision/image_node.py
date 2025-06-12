#!/venv/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.subscription = self.create_subscription(
            Image,
            '/depth_camera/depth_image',
            self.camera_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        self.bridge = CvBridge()

        # Load YOLO model
        self.detection_model = YOLO("yolov8m.pt")  # or your custom model path
        self.segmentation_model = YOLO("yolo11m-seg.pt")

    def camera_callback(self, msg):
        try:
            # Convert depth image
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Normalize and convert to BGR for YOLO
            if depth_image.dtype != 'uint8':
                normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
                depth_rgb = cv2.convertScaleAbs(normalized)
            else:
                depth_rgb = depth_image
                
                depth_rgb = cv2.cvtColor(depth_rgb, cv2.COLOR_GRAY2BGR)
                
                # Run YOLO
                results = self.detection_model(depth_rgb)
                
                annotated_frame = results[0].plot()

                for result in results:
                    for box in result.boxes:
                        # Get bounding box center
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cx = (x1 + x2) // 2
                        cy = (y1 + y2) // 2

                        # Get distance at center
                        if 0 <= cy < depth_image.shape[0] and 0 <= cx < depth_image.shape[1]:
                            distance = float(depth_image[cy, cx])
                            self.get_logger().info(f"Object at ({cx}, {cy}) distance: {distance:.2f} meters")
                            
                            # Optional: Draw distance on image
                            cv2.putText(
                                annotated_frame,
                                f"{distance:.2f}m",
                                (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (0, 255, 0),
                                1
                            )

            # Show annotated image
            cv2.imshow("YOLO + Depth", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
