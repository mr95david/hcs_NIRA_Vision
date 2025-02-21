#!/usr/bin/env python3
import cv2
import numpy as np
from typing import List, Tuple

import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState

import message_filters
from cv_bridge import CvBridge
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener

from sensor_msgs.msg import CameraInfo, Image, CompressedImage
from geometry_msgs.msg import TransformStamped
from yolov8_msgs.msg import Detection
from yolov8_msgs.msg import DetectionArray
from yolov8_msgs.msg import KeyPoint3D
from yolov8_msgs.msg import KeyPoint3DArray
from yolov8_msgs.msg import BoundingBox3D


class Detect3DNode(LifecycleNode):

    def __init__(self) -> None:
        super().__init__("bbox3d_node")

        # parameters
        self.declare_parameter("target_frame", "odom")
        self.declare_parameter("maximum_detection_threshold", 0.3)
        self.declare_parameter("depth_image_units_divisor", 1000)
        self.declare_parameter("depth_image_reliability",
                               QoSReliabilityPolicy.BEST_EFFORT)
        self.declare_parameter("depth_info_reliability",
                               QoSReliabilityPolicy.RELIABLE)

        # aux
        self.tf_buffer = Buffer()
        self.cv_bridge = CvBridge()

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Configuring {self.get_name()}")

        self.target_frame = self.get_parameter(
            "target_frame").get_parameter_value().string_value
        self.maximum_detection_threshold = self.get_parameter(
            "maximum_detection_threshold").get_parameter_value().double_value
        self.depth_image_units_divisor = self.get_parameter(
            "depth_image_units_divisor").get_parameter_value().integer_value
        dimg_reliability = self.get_parameter(
            "depth_image_reliability").get_parameter_value().integer_value

        self.depth_image_qos_profile = QoSProfile(
            reliability=dimg_reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        dinfo_reliability = self.get_parameter(
            "depth_info_reliability").get_parameter_value().integer_value

        self.depth_info_qos_profile = QoSProfile(
            reliability=dinfo_reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # pubs
        self._pub = self.create_publisher(DetectionArray, "detections_3d", 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Activating {self.get_name()}")

        # subs
        self.depth_sub = message_filters.Subscriber(
            # self, Image, "/depth/image_rect_raw",
            self, CompressedImage, "/Nira/aligned_depth_to_color/compressed_image",
            qos_profile=self.depth_image_qos_profile)
        self.depth_sub
        self.depth_info_sub = message_filters.Subscriber(
            self, CameraInfo, "/aligned_depth_to_color/camera_info",
            qos_profile=self.depth_info_qos_profile)
        self.depth_info_sub
        self.detections_sub = message_filters.Subscriber(
            self, DetectionArray, "/detections")
        self.detections_sub

        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            (self.depth_sub, self.depth_info_sub, self.detections_sub), 10, 0.5)
        self._synchronizer.registerCallback(self.on_detections)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Deactivating {self.get_name()}")

        self.destroy_subscription(self.depth_sub.sub)
        self.destroy_subscription(self.depth_info_sub.sub)
        self.destroy_subscription(self.detections_sub.sub)

        del self._synchronizer

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Cleaning up {self.get_name()}")

        del self.tf_listener

        self.destroy_publisher(self._pub)

        return TransitionCallbackReturn.SUCCESS

    def on_detections(
        self,
        #depth_msg: Image,
        depth_msg: CompressedImage,
        depth_info_msg: CameraInfo,
        detections_msg: DetectionArray,
    ) -> None:

        new_detections_msg = DetectionArray()
        new_detections_msg.header = detections_msg.header
        new_detections_msg.detections = self.process_detections(
            depth_msg, depth_info_msg, detections_msg)
        self._pub.publish(new_detections_msg)

    def process_detections(
        self,
        depth_msg: Image,
        depth_info_msg: CameraInfo,
        detections_msg: DetectionArray
    ) -> List[Detection]:

        # check if there are detections
        if not detections_msg.detections:
            return []
        # self.get_logger().info(f"Frame id: {depth_info_msg.header.frame_id}")
        transform = self.get_transform('camera_link')
        # transform = self.get_transform(depth_info_msg.header.frame_id)
        # self.get_logger().info(f"Transformacion desde: {transform}")
        if transform is None:
            return []
        
        new_detections = []
        np_arr = np.frombuffer(depth_msg.data, np.uint8)
        depth_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        try:
            self.get_logger().warn(f"{type(depth_image)}")
        except Exception as e:
            self.get_logger().error(f"Error in depth_img: {e}")

        for detection in detections_msg.detections:
            
            bbox3d = self.convert_bb_to_3d(
                depth_image, depth_info_msg, detection)
            #print(bbox3d)
            if bbox3d is not None:
                #print("entra")
                new_detections.append(detection)
                
                #print(len(new_detections))
                bbox3d = self.transform_3d_box(
                    bbox3d, transform[0], transform[1])
                # print(bbox3d.center.position.x)
                bbox3d.frame_id = self.target_frame
                new_detections[-1].bbox3d = bbox3d
                #print(bbox3d)
                if detection.keypoints.data:
                    
                    keypoints3d = self.convert_keypoints_to_3d(
                        depth_image, depth_info_msg, detection)
                    keypoints3d = Detect3DNode.transform_3d_keypoints(
                        keypoints3d, transform[0], transform[1])
                    keypoints3d.frame_id = self.target_frame
                    new_detections[-1].keypoints3d = keypoints3d

        return new_detections

    def convert_bb_to_3d(
        self,
        depth_image: np.ndarray,
        depth_info: CameraInfo,
        detection: Detection
    ) -> BoundingBox3D:

        center_x = int(detection.bbox.center.position.x)
        center_y = int(detection.bbox.center.position.y)
        size_x = int(detection.bbox.size.x)
        size_y = int(detection.bbox.size.y)

        if detection.mask.data:
            # crop depth image by mask
            mask_array = np.array([[int(ele.x), int(ele.y)]
                                   for ele in detection.mask.data])
            mask = np.zeros(depth_image.shape[:2], dtype=np.uint8)
            cv2.fillPoly(mask, [np.array(mask_array, dtype=np.int32)], 255)
            roi = cv2.bitwise_and(depth_image, depth_image, mask=mask)

        else:
            # crop depth image by the 2d BB
            u_min = max(center_x - size_x // 2, 0)
            u_max = min(center_x + size_x // 2, depth_image.shape[1] - 1)
            v_min = max(center_y - size_y // 2, 0)
            v_max = min(center_y + size_y // 2, depth_image.shape[0] - 1)

            roi = depth_image[v_min:v_max, u_min:u_max]

        roi = roi / self.depth_image_units_divisor  # convert to meters
        if not np.any(roi):
            return None

        # find the z coordinate on the 3D BB
        if detection.mask.data:
            roi = roi[roi > 0]
            bb_center_z_coord = np.median(roi)

        else:
            bb_center_z_coord = depth_image[int(center_y)][int(
                center_x)] / self.depth_image_units_divisor

        z_diff = np.abs(roi - bb_center_z_coord)
        mask_z = z_diff <= self.maximum_detection_threshold
        if not np.any(mask_z):
            return None

        roi = roi[mask_z]
        z_min, z_max = np.min(roi), np.max(roi)
        z = (z_max + z_min) / 2

        if z == 0:
            return None

        # project from image to world space
        k = depth_info.k
        px, py, fx, fy = k[2], k[5], k[0], k[4]
        x = z * (center_x - px) / fx
        y = z * (center_y - py) / fy
        w = z * (size_x / fx)
        h = z * (size_y / fy)

        # create 3D BB
        msg = BoundingBox3D()
        msg.center.position.x = x
        msg.center.position.y = y
        msg.center.position.z = z
        msg.size.x = w
        msg.size.y = h
        msg.size.z = float(z_max - z_min)

        return msg

    def convert_keypoints_to_3d(
        self,
        depth_image: np.ndarray,
        depth_info: CameraInfo,
        detection: Detection
    ) -> KeyPoint3DArray:

        # build an array of 2d keypoints
        keypoints_2d = np.array([[p.point.x, p.point.y]
                                for p in detection.keypoints.data], dtype=np.int16)
        u = np.array(keypoints_2d[:, 1]).clip(0, depth_info.height - 1)
        v = np.array(keypoints_2d[:, 0]).clip(0, depth_info.width - 1)

        # sample depth image and project to 3D
        z = depth_image[u, v]
        k = depth_info.k
        px, py, fx, fy = k[2], k[5], k[0], k[4]
        x = z * (v - px) / fx
        y = z * (u - py) / fy
        points_3d = np.dstack([x, y, z]).reshape(-1, 3) / \
            self.depth_image_units_divisor  # convert to meters

        # generate message
        msg_array = KeyPoint3DArray()
        for p, d in zip(points_3d, detection.keypoints.data):
            if not np.isnan(p).any():
                msg = KeyPoint3D()
                msg.point.x = p[0]
                msg.point.y = p[1]
                msg.point.z = p[2]
                msg.id = d.id
                msg.score = d.score
                msg_array.data.append(msg)

        return msg_array

    def get_transform(self, frame_id: str) -> Tuple[np.ndarray]:
        # transform position from image frame to target_frame
        rotation = None
        translation = None

        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                frame_id,
                rclpy.time.Time())

            translation = np.array([transform.transform.translation.x,
                                    transform.transform.translation.y,
                                    transform.transform.translation.z])

            rotation = np.array([transform.transform.rotation.w,
                                 transform.transform.rotation.x,
                                 transform.transform.rotation.y,
                                 transform.transform.rotation.z])

            return translation, rotation

        except TransformException as ex:
            self.get_logger().error(f"Could not transform: {ex}")
            return None

    @staticmethod
    def transform_3d_box(
        bbox: BoundingBox3D,
        translation: np.ndarray,
        rotation: np.ndarray
    ) -> BoundingBox3D:

        # position
        position = Detect3DNode.qv_mult(
            rotation,
            np.array([bbox.center.position.x,
                      bbox.center.position.y,
                      bbox.center.position.z])
        ) + translation

        bbox.center.position.x = position[0]
        bbox.center.position.y = position[1]
        bbox.center.position.z = position[2]

        # size
        size = Detect3DNode.qv_mult(
            rotation,
            np.array([bbox.size.x,
                      bbox.size.y,
                      bbox.size.z])
        )

        bbox.size.x = abs(size[0])
        bbox.size.y = abs(size[1])
        bbox.size.z = abs(size[2])

        return bbox

    @staticmethod
    def transform_3d_keypoints(
        keypoints: KeyPoint3DArray,
        translation: np.ndarray,
        rotation: np.ndarray,
    ) -> KeyPoint3DArray:

        for point in keypoints.data:
            position = Detect3DNode.qv_mult(
                rotation,
                np.array([
                    point.point.x,
                    point.point.y,
                    point.point.z
                ])
            ) + translation

            point.point.x = position[0]
            point.point.y = position[1]
            point.point.z = position[2]

        return keypoints

    @staticmethod
    def qv_mult(q: np.ndarray, v: np.ndarray) -> np.ndarray:
        q = np.array(q, dtype=np.float64)
        v = np.array(v, dtype=np.float64)
        qvec = q[1:]
        uv = np.cross(qvec, v)
        uuv = np.cross(qvec, uv)
        return v + 2 * (uv * q[0] + uuv)


def main():
    rclpy.init()
    node = Detect3DNode()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()