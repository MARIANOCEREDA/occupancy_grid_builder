import os
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO


class SegmentationNode(Node):
    def __init__(self):
        super().__init__('segmentation_node')

        # Parameters
        default_model = os.path.join(
            get_package_share_directory('segmentation_inference'),
            'models', 'yolo11n-seg.pt',
        )
        self.declare_parameter('model_path', default_model)
        self.declare_parameter('confidence', 0.25)
        self.declare_parameter('device', 'cpu')

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        confidence = self.get_parameter('confidence').get_parameter_value().double_value
        device = self.get_parameter('device').get_parameter_value().string_value

        self.confidence = confidence
        self.bridge = CvBridge()

        self.get_logger().info(f'Loading model from: {model_path}')
        self.model = YOLO(model_path)
        self.model.to(device)
        self.get_logger().info('Model loaded successfully.')

        self.sub = self.create_subscription(
            Image, '/image_rect', self.image_callback, 10
        )
        self.pub = self.create_publisher(Image, '/segmentation/mask', 10)

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)
        
        results = self.model(frame, conf=self.confidence, verbose=False)

        mask_img = self._build_mask(frame.shape[:2], results)

        out_msg = Image()
        out_msg.header = msg.header
        out_msg.height, out_msg.width = mask_img.shape[:2]
        out_msg.encoding = 'rgb8'
        out_msg.is_bigendian = 0
        out_msg.step = out_msg.width * 3
        out_msg.data = mask_img.tobytes()
        self.pub.publish(out_msg)

    # ------------------------------------------------------------------
    def _build_mask(self, hw, results):
        """Return an RGB coloured segmentation mask (H x W x 3, uint8)."""
        h, w = hw
        canvas = np.zeros((h, w, 3), dtype=np.uint8)

        result = results[0]
        if result.masks is None:
            return canvas

        masks = result.masks.data.cpu().numpy()   # (N, H', W')
        class_ids = result.boxes.cls.cpu().numpy().astype(int)

        for mask, cls_id in zip(masks, class_ids):
            # Resize mask to original image size
            binary = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
            binary = (binary > 0.5).astype(np.uint8)

            colour = _class_colour(cls_id)
            canvas[binary == 1] = colour

        return canvas


def _class_colour(class_id: int):
    """Deterministic per-class colour from the class index."""
    np.random.seed(class_id)
    return tuple(int(c) for c in np.random.randint(64, 255, 3))


def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
