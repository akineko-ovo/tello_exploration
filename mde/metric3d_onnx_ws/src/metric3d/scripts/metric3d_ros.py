#!/usr/bin/env python3
import onnxruntime as ort
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
import time
from cv_bridge import CvBridge
from rospy import Subscriber, Publisher


class MDE():
    def __init__(self):
        rospy.init_node("metric3d")
        rospy.loginfo("Starting rosnode.")
        self._read_params()
        self._init_model(self.model_path, self.image_size, self.P)
        self._init_topics()
        self._warmup()

        rospy.loginfo("Initialization Done")
        rospy.spin()

    def _read_params(self):
        rospy.loginfo("Reading parameters...")

        self.model_path = rospy.get_param("~MODEL_FILE", "./models/metric3d.onnx")
        self.warmup_path = rospy.get_param("~WARMUP_FILE", "./warmup/image.png")
        self.frame_id = "world"

        self.image_size = (320, 480)
        self.P = np.zeros([3, 4])
        self.P[0:3, 0:3] = np.array(
            [532.5586983875527, 0, 255.5,
             0, 532.5586983875527, 161.0,
             0, 0, 1.0000]
        ).reshape(3, 3)

    def _init_model(self, onnx_path, image_size, P):
        providers = [("CUDAExecutionProvider", {"cudnn_conv_use_max_workspace": '0', 'device_id': '0'})]
        sess_options = ort.SessionOptions()
        self.ort_session = ort.InferenceSession(onnx_path, providers=providers, sess_options=sess_options)

        # image parameters
        self.h, self.w = image_size

        # network parameters
        input_size = (616, 1064)
        scale = min(input_size[0] / self.h, input_size[1] / self.w)

        self.rgb_h = int(self.h * scale)
        self.rgb_w = int(self.w * scale)

        self.padding = [123.675, 116.28, 103.53]

        self.pad_h = input_size[0] - self.rgb_h
        self.pad_w = input_size[1] - self.rgb_w
        self.pad_h_half = self.pad_h // 2
        self.pad_w_half = self.pad_w // 2

        self.pad_info = [self.pad_h_half, self.pad_h - self.pad_h_half, self.pad_w_half, self.pad_w - self.pad_w_half]

        self.P_scale = P
        self.P_scale[0:2, :] = P[0:2, :] * scale

        # Create P_inv
        P_expanded = np.eye(4)
        P_expanded[0:3, 0:4] = self.P_scale
        self.P_inv = np.linalg.inv(P_expanded)  # 4x4

        # Create T
        self.T = np.eye(4)

        # Create mask
        H, W = input_size
        self.mask = np.zeros([H, W], dtype=np.uint8)
        self.mask[self.pad_info[0]: H - self.pad_info[1], self.pad_info[2]: W - self.pad_info[3]] = 1

    def _init_topics(self):
        self.cv_bridge = CvBridge()
        self.depth_pub = rospy.Publisher("depth_image", Image, queue_size=1)
        self.image_sub = Subscriber("image_raw", Image, self.image_callback, queue_size=1)

    def _warmup(self):
        image = cv2.imread(self.warmup_path)
        self.infer(image)
        rospy.loginfo("Finishing warmup...")

    def image_callback(self, msg: Image):
        start_time = time.time()

        height = msg.height
        width = msg.width
        stamp = msg.header.stamp

        image = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, 3))[:, :, ::-1]

        depth_image = self.infer(image)

        # publish depth
        depth_msg = self.cv_bridge.cv2_to_imgmsg(depth_image, encoding="passthrough")
        depth_msg.header.frame_id = self.frame_id
        depth_msg.header.stamp = stamp
        self.depth_pub.publish(depth_msg)

        rospy.loginfo(f"Total runtime: {time.time() - start_time}")

    def infer(self, image):
        # prepare input
        rgb = cv2.resize(image, (self.rgb_w, self.rgb_h), interpolation=cv2.INTER_LINEAR)
        rgb: np.ndarray = cv2.copyMakeBorder(rgb,
                                             self.pad_h_half, self.pad_h - self.pad_h_half,
                                             self.pad_w_half, self.pad_w - self.pad_w_half,
                                             cv2.BORDER_CONSTANT, value=self.padding)

        onnx_input = {
            'image': np.ascontiguousarray(np.transpose(rgb, (2, 0, 1))[None], dtype=np.float32),  # 1, 3, H, W
            'P': self.P_scale.astype(np.float32)[None],  # 1, 3, 4
            'P_inv': self.P_inv.astype(np.float32)[None],  # 1, 4, 4
            'T': self.T.astype(np.float32)[None],  # 1, 4, 4
            'mask': self.mask.astype(np.bool)[None]  # 1, H, W
        }

        # infer
        outputs = self.ort_session.run(None, onnx_input)

        # postprocess
        depth_image = outputs[0][0, 0]  # [1, 1, H, W] -> [H, W]
        mask = outputs[2]  # [HW]
        depth_image = depth_image[self.pad_info[0]: depth_image.shape[0] - self.pad_info[1],
                      self.pad_info[2]: depth_image.shape[1] - self.pad_info[3]]
        depth_image = cv2.resize(depth_image, (self.w, self.h), interpolation=cv2.INTER_LINEAR)  # [H, W] -> [h, w]

        return depth_image


def main(args=None):
    MDE()


if __name__ == "__main__":
    main()
