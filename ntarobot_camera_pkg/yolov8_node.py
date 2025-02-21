#!/usr/bin/env python3
# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


from typing import List, Dict

import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState

from cv_bridge import CvBridge

import torch
from ultralytics import YOLO, NAS
from ultralytics.engine.results import Results
from ultralytics.engine.results import Boxes
from ultralytics.engine.results import Masks
from ultralytics.engine.results import Keypoints

from std_srvs.srv import SetBool
from ntarobot_services.srv import OwmStr
# from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CompressedImage
from yolov8_msgs.msg import Point2D
from yolov8_msgs.msg import BoundingBox2D
from yolov8_msgs.msg import Mask
from yolov8_msgs.msg import KeyPoint2D
from yolov8_msgs.msg import KeyPoint2DArray
from yolov8_msgs.msg import Detection
from yolov8_msgs.msg import DetectionArray

# Utilitarias
from ntarobot_camera_pkg import ID_OBJECTS
import numpy as np
import cv2

class Yolov8Node(LifecycleNode):

    def __init__(self) -> None:
        super().__init__("yolov8_node")

        # params
        self.declare_parameter("model_type", "YOLO")
        self.declare_parameter("model", "./src/ntarobot_camera_pkg/models/yolov8n-seg.pt")
        self.declare_parameter("device", "cuda:0")
        self.declare_parameter("threshold", 0.5)
        self.declare_parameter("enable", True)
        self.declare_parameter("object_id", "all")
        self.declare_parameter("image_reliability", QoSReliabilityPolicy.RELIABLE)
        #self.declare_parameter("topic_img", "/color/image_raw")
        self.declare_parameter("topic_img", "/color/image_raw/compressed")
        # self.declare_parameter("topic_img", "/camera/image_raw/compressed")

        self.type_to_model = {
            "YOLO": YOLO,
            "NAS": NAS
        }

        self.get_logger().info("Yolov8 Node created")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Configuring {self.get_name()}")

        self.model_type = self.get_parameter(
            "model_type").get_parameter_value().string_value

        self.model = self.get_parameter(
            "model").get_parameter_value().string_value

        self.device = self.get_parameter(
            "device").get_parameter_value().string_value

        self.threshold = self.get_parameter(
            "threshold").get_parameter_value().double_value

        self.enable = self.get_parameter(
            "enable").get_parameter_value().bool_value

        self.reliability = self.get_parameter(
            "image_reliability").get_parameter_value().integer_value
        
        self.topic_img_ = self.get_parameter(
            "topic_img").get_parameter_value().string_value
        
        self.object_id_selected = self.get_parameter(
            "object_id").get_parameter_value().string_value

        self.object_num = ID_OBJECTS[self.object_id_selected]

        self.image_qos_profile = QoSProfile(
            reliability=self.reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )

        self._pub = self.create_lifecycle_publisher(
            DetectionArray, "detections", 10)
        # Servicio para activacion de tracking de camara
        self._srv = self.create_service(
            SetBool, "enable_object_detection", self.enable_cb
        )
        # Creacion de servicio para definicion de objeto para deteccion 
        self._srv_obj = self.create_service(
            OwmStr,
            "object_to_detect",
            self.get_object
        )
        self.cv_bridge = CvBridge()

        return TransitionCallbackReturn.SUCCESS

    def enable_cb(self, request, response):
        self.enable = request.data
        response.success = True
        return response
    
    # Funcion para definicion de deteccion de objeto
    def get_object(self, request, response):
        self.object_id_selected = request.data
        self.object_num = ID_OBJECTS[self.object_id_selected]
        response.success = True
        return response 

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Activating {self.get_name()}")
        
        self.yolo = self.type_to_model[self.model_type](self.model)

        if "v10" not in self.model:
            self.yolo.fuse()

        # subs
        self._sub = self.create_subscription(
            CompressedImage,
            # Image,
            self.topic_img_,
            self.image_cb,
            self.image_qos_profile
        )
        self._sub

        super().on_activate(state)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Deactivating {self.get_name()}")

        del self.yolo
        if "cuda" in self.device:
            self.get_logger().info("Clearing CUDA cache")
            torch.cuda.empty_cache()

        self.destroy_subscription(self._sub)
        self._sub = None

        super().on_deactivate(state)

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Cleaning up {self.get_name()}")

        self.destroy_publisher(self._pub)

        del self.image_qos_profile

        return TransitionCallbackReturn.SUCCESS

    def parse_hypothesis(self, results: Results) -> List[Dict]:
        
        hypothesis_list = []

        if results.boxes:
            box_data: Boxes
            for box_data in results.boxes:
                if self.object_num == 100 or int(box_data.cls) == self.object_num:
                    hypothesis = {
                        "class_id": int(box_data.cls),
                        "class_name": self.yolo.names[int(box_data.cls)],
                        "score": float(box_data.conf)
                    }
                    hypothesis_list.append(hypothesis)
                # else:
                #     if int(box_data.cls) == self.object_num:
                #         hypothesis = {
                #             "class_id": int(box_data.cls),
                #             "class_name": self.yolo.names[int(box_data.cls)],
                #             "score": float(box_data.conf)
                #         }
                #         hypothesis_list.append(hypothesis)

        elif results.obb:
            for i in range(results.obb.cls.shape[0]):
                if self.object_num == 100 or int(results.obb.cls[i]) == self.object_num:
                    hypothesis = {
                        "class_id": int(results.obb.cls[i]),
                        "class_name": self.yolo.names[int(results.obb.cls[i])],
                        "score": float(results.obb.conf[i])
                    }
                    hypothesis_list.append(hypothesis)
                # else:
                #     if int(results.obb.cls[i]) == self.object_num:
                #         hypothesis = {
                #             "class_id": int(results.obb.cls[i]),
                #             "class_name": self.yolo.names[int(results.obb.cls[i])],
                #             "score": float(results.obb.conf[i])
                #         }
                #         hypothesis_list.append(hypothesis)

        return hypothesis_list

    def parse_boxes(self, results: Results) -> List[BoundingBox2D]:

        boxes_list = []

        if results.boxes:
            box_data: Boxes
            for box_data in results.boxes:
                msg = BoundingBox2D()
                if self.object_num == 100 or int(box_data.cls) == self.object_num:
                    # get boxes values
                    box = box_data.xywh[0]
                    msg.center.position.x = float(box[0])
                    msg.center.position.y = float(box[1])
                    msg.size.x = float(box[2])
                    msg.size.y = float(box[3])

                    # append msg
                    boxes_list.append(msg)
                # else:
                #     if int(box_data.cls) == self.object_num:
                #         # get boxes values
                #         box = box_data.xywh[0]
                #         msg.center.position.x = float(box[0])
                #         msg.center.position.y = float(box[1])
                #         msg.size.x = float(box[2])
                #         msg.size.y = float(box[3])

                #         # append msg
                #         boxes_list.append(msg)

        elif results.obb:
            for i in range(results.obb.cls.shape[0]):
                msg = BoundingBox2D()
                if self.object_num == 100 or int(results.obb.cls[i]) == self.object_num:
                    # get boxes values
                    box = results.obb.xywhr[i]
                    msg.center.position.x = float(box[0])
                    msg.center.position.y = float(box[1])
                    msg.center.theta = float(box[4])
                    msg.size.x = float(box[2])
                    msg.size.y = float(box[3])

                    # append msg
                    boxes_list.append(msg)
                # else:
                #     if int(results.obb.cls[i]) == self.object_num:

                #         # Obtener valores de las cajas
                #         box = results.obb.xywhr[i]
                #         msg.center.position.x = float(box[0])
                #         msg.center.position.y = float(box[1])
                #         msg.center.theta = float(box[4])
                #         msg.size.x = float(box[2])
                #         msg.size.y = float(box[3])

                #         # Agregar al listado
                #         boxes_list.append(msg)

        return boxes_list

    def parse_masks(self, results: Results) -> List[Mask]:

        masks_list = []

        def create_point2d(x: float, y: float) -> Point2D:
            p = Point2D()
            p.x = x
            p.y = y
            return p

        # Prueba de nuevo modelo
        #mask: Masks
        #for mask in results.masks:
        for mask, cls in zip(results.masks, results.boxes.cls if results.boxes else []):

            msg = Mask()
            if self.object_num == 100 or int(cls) == self.object_num:
                msg.data = [create_point2d(float(ele[0]), float(ele[1]))
                            for ele in mask.xy[0].tolist()]
                msg.height = results.orig_img.shape[0]
                msg.width = results.orig_img.shape[1]

                masks_list.append(msg)

        return masks_list

    def parse_keypoints(self, results: Results) -> List[KeyPoint2DArray]:

        keypoints_list = []

        #points: Keypoints
        #for points in results.keypoints:
        for points, cls in zip(results.keypoints, results.boxes.cls if results.boxes else []):

            msg_array = KeyPoint2DArray()
            if self.object_num == 100 or int(cls) == self.object_num:
                if points.conf is None:
                    continue

                for kp_id, (p, conf) in enumerate(zip(points.xy[0], points.conf[0])):

                    if conf >= self.threshold:
                        msg = KeyPoint2D()

                        msg.id = kp_id + 1
                        msg.point.x = float(p[0])
                        msg.point.y = float(p[1])
                        msg.score = float(conf)

                        msg_array.data.append(msg)

                keypoints_list.append(msg_array)

        return keypoints_list

    def image_cb(self, msg: CompressedImage) -> None:
        #self.get_logger().warn(f"Valor de objeto de identificacion: {self.object_num}")
        if self.enable:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            #cv_image = self.cv_bridge.imgmsg_to_cv2(msg)
            results = self.yolo.predict(
                source=cv_image,
                verbose=False,
                stream=False,
                conf=self.threshold,
                device=self.device
            )
            print(results)
            results: Results = results[0].cpu()
            hypothesis = []
            if results.boxes or results.obb:
                hypothesis = self.parse_hypothesis(results)
                boxes = self.parse_boxes(results)

            if results.masks:
                masks = self.parse_masks(results)

            if results.keypoints:
                keypoints = self.parse_keypoints(results)

            
            detections_msg = DetectionArray()
            
            #for i in range(len(results)):
            for i in range(len(hypothesis)):
                
                aux_msg = Detection()

                if results.boxes or results.obb:
                    aux_msg.class_id = hypothesis[i]["class_id"]
                    aux_msg.class_name = hypothesis[i]["class_name"]
                    aux_msg.score = hypothesis[i]["score"]

                    aux_msg.bbox = boxes[i]

                if results.masks:
                    aux_msg.mask = masks[i]

                if results.keypoints:
                    aux_msg.keypoints = keypoints[i]

                detections_msg.detections.append(aux_msg)

            # publish detections
            detections_msg.header = msg.header
            self._pub.publish(detections_msg)

            del results
            del cv_image
        


def main():
    rclpy.init()
    node = Yolov8Node()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
