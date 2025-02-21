#!/usr/bin/env python3
# Seccion de importe de librerias 
# Librerias de ros2, intereaccion de nodos y ejecucion de herramientas
import rclpy
# Importe de herramientas de manejo de nodos lifecycle
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.lifecycle import LifecycleState
# Imṕorte de herramientas de cualidad de servicio
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
# Import de librerias utilitarias
from cv_bridge import CvBridge
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
# Importe de librerias de interfaces usadas
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
from yolov8_msgs.msg import DetectionArray, Detection
from yolov8_msgs.msg import BoundingBox2D
from yolov8_msgs.msg import BoundingBox3D
from yolov8_msgs.msg import Point2D
from yolov8_msgs.msg import Mask
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
# Importe de librerias de validacion de datos
from typing import Tuple
# Importe de librerias externas para procesamiento de datos y demas
import message_filters
import numpy as np
import cv2
from time import time

# Seccion de inicializacion de clase
class PoseDetection(LifecycleNode):

    def __init__(self) -> None:
        super().__init__("object_pose_detection")
        # Declracion de parametros de uso de la clase
        self.declare_parameter("target_frame", "camera_link"),
        self.declare_parameter("qos_robot_comm", QoSReliabilityPolicy.RELIABLE),
        self.declare_parameter("topic_depth_image", "/Nira/aligned_depth_to_color/compressed_image"),
        self.declare_parameter("topic_camera_info", "/aligned_depth_to_color/camera_info"),
        self.declare_parameter("topic_detections_object", "/detections")
        self.declare_parameter("depth_image_units_divisor", 1000)
        self.declare_parameter("maximum_detection_threshold", 0.3)

        # Inicializaicon de variables de instancia
        self.tf_buffer = Buffer()
        self.cv_bridge = CvBridge()
        

    
    # Seccion de configuracion de nodo
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Configuring {self.get_name()}")
        # En esta seccion se inicializan las variable provenientes de paramtros y variables temporales
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.qos_robot_comm = self.get_parameter('qos_robot_comm').get_parameter_value().integer_value
        self.topic_depth_image = self.get_parameter('topic_depth_image').get_parameter_value().string_value
        self.topic_camera_info = self.get_parameter('topic_camera_info').get_parameter_value().string_value
        self.topic_detections_object = self.get_parameter('topic_detections_object').get_parameter_value().string_value
        self.depth_image_units_divisor = self.get_parameter("depth_image_units_divisor").get_parameter_value().integer_value
        self.maximum_detection_threshold = self.get_parameter("maximum_detection_threshold").get_parameter_value().double_value

        # Creacion de configuracion de perfil de calidad de servicio
        self.depth_image_qos_profile = QoSProfile(
            reliability = self.qos_robot_comm,
            history = QoSHistoryPolicy.KEEP_LAST,
            durability = QoSDurabilityPolicy.VOLATILE,
            depth = 1
        )

        # Creacion de perfil de seleccion de cualidad de servicio de informacion de camara
        self.depth_info_qos_profile = QoSProfile(
            reliability = self.qos_robot_comm,
            history = QoSHistoryPolicy.KEEP_LAST,
            durability = QoSDurabilityPolicy.VOLATILE,
            depth = 1
        )

        # Transformacion de posicion de tf publicados de secciones de robot
        self.tf_listener = TransformListener(self.tf_buffer, self)

        #Creacion de publicadores de resultados obtenidos
        self.pub_ = self.create_publisher(
            BoundingBox3D,
            "test_",
            10
        )

        return TransitionCallbackReturn.SUCCESS
    
    # Seccion de activacion de nodo
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Activating {self.get_name()}")
        try:
        # Se crea un sincronizador que permite establecer el mismo tiempo de interaccion para cada nodo recibido
            self.sub_depth = message_filters.Subscriber(
                self, 
                CompressedImage,
                self.topic_depth_image,
                qos_profile = self.depth_image_qos_profile
            )
            self.sub_depth

            # Subscripcion a valores de informacion de camara
            self.sub_info_sub = message_filters.Subscriber(
                self,
                CameraInfo,
                self.topic_camera_info,
                qos_profile = self.depth_info_qos_profile
            )
            self.sub_info_sub 

            # Subscripcion a array de detecciones obtenidas
            self.sub_detections = message_filters.Subscriber(
                self,
                DetectionArray,
                self.topic_detections_object
            )
            self.sub_detections 
            
            # Sincronizador de valores de subscripcion
            self._synchronizer = message_filters.ApproximateTimeSynchronizer(
                (self.sub_depth, self.sub_info_sub, self.sub_detections),
                10,
                0.5
            )

            self.get_logger().warning("Subscripcion finalizada")
        except Exception as e:
            self.get_logger().error(e)
        # self.get_logger().info(f"Activacion general")
        # Registro de callback de subscriptores varios
        self._synchronizer.registerCallback(cb = self.on_detections)

        return TransitionCallbackReturn.SUCCESS
    
    # Seccion de desactivacion de nodo
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Deactivating {self.get_name()}")
        # Eliminacion de topicos creados en la seccion de activacion
        self.destroy_subscription(self.sub_depth.sub)
        self.destroy_subscription(self.sub_info_sub.sub)
        self.destroy_subscription(self.sub_detections.sub)

        del self._synchronizer

        return TransitionCallbackReturn.SUCCESS
    
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Cleaning up {self.get_name()}")
        # Eliminacion de variables creadas en la seccion de configuracion
        del self.tf_listener
        self.destroy_publisher(self._pub)

        return TransitionCallbackReturn.SUCCESS

    def on_detections(
            self,
            depth_msg: CompressedImage,
            depth_info_msg: CameraInfo,
            detections_msg: DetectionArray
        ) -> None:
        
        self.get_logger().info("Recibiendo informacion correctamente")

        # Variabels de scope local
        lista_coor = []

        new_detect = detections_msg
        if len(new_detect.detections) < 1:
            self.get_logger().info("No se detectan objetos")
            return
        height_det, width_det  = new_detect.detections[0].mask.height, new_detect.detections[0].mask.width

        # Primero se convierte el valor de entrada de la imgen de profundidad recibida
        try:
            cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(
                depth_msg, desired_encoding='bgr8'
            )
            # Dimensiones provenientes de la imagen
            height_depth, width_depth, _ = cv_image.shape

        except Exception as e:
            self.get_logger().error(f"Failed to convert CompressedImage: {e}")
            return 

        # Deteccion de valores obtenidos de array de detecciones
        val_shape = self.shape_validation(
            [height_det, width_det],
            [height_depth, width_depth]
        )
        #print(val_shape)
        # Se obtiene la transformacion desde el valor de la camara del robot
        transform = self.get_transform("base_link")

        if transform is None:
            self.get_logger().error("No se detecta transformacion")
            return None

        #print(f"Valores de transformacion: {transform}")
        #print(type(cv_image))
        start_time = time()
        
        if not val_shape:
            for detection in new_detect.detections:
                # Conversion de valores de caja 2d de acuerdo de la diferencia de tamanhs de imagen
                actual_box = Detection()
                list_values_t = self.scale_size_box(value_transform = detection.bbox)
                # Asignacion de valores a nuevo valor de deteccion
                actual_box.bbox.center.position.x = list_values_t[0]
                actual_box.bbox.center.position.y = list_values_t[1]
                actual_box.bbox.size.x = list_values_t[2]
                actual_box.bbox.size.y = list_values_t[3]
                
                # Conversion de valores obtenidos de la mascara
                # lista_coor.append(self.scale_vars(detection.mask.data))
                actual_box.mask = self.scale_vars(detection.mask.data)
                
                bbox3d = self.convert_bb_to_3d(cv_image, depth_info_msg, actual_box)
                if bbox3d is not None:
                    bbox3d = self.transform_3d_box(bbox3d, transform[0], transform[1])
                    self.pub_.publish(bbox3d)

        end_time = time()-start_time
        self.get_logger().info(f"El tiempo de ejecucion de la funcion fue: {end_time}")


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
        

    def shape_validation(self, shape_rgb, shape_depth):
        # Considerando de que [0] corresponde a height y [1] corresponde a width
        return shape_rgb[0] == shape_depth[0] and shape_rgb[1] == shape_depth[1]

    # Funcion para escalar las coordenadas encontradas de las detecciones y almacenaralas en un nuevo array
    def scale_vars(self, coordenate_list, t_x = 0.5, t_y = 2/3):
        internal_mask = Mask()

        # Adicion de nuevo valor de dimensiones ingresadas
        internal_mask.height = 240
        internal_mask.width = 320
        # Conversion de valores obtenidos
        for point in coordenate_list:
            single_value = Point2D()
            single_value.x = float(round(point.x*t_x, 0))
            single_value.y = float(round(point.y*t_y, 0))
            internal_mask.data.append(single_value)
        return internal_mask
    
    def scale_size_box(self, value_transform: BoundingBox2D, t_x = 0.5, t_y = 2/3) -> list:
        new_x = value_transform.center.position.x*t_x
        new_y = value_transform.center.position.y*t_y
        new_size_x = value_transform.size.x*t_x
        new_size_y = value_transform.size.y*t_y 
        return [new_x,
                new_y,
                new_size_x,
                new_size_y]
        
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
        # print("test3")
        if not np.any(roi):
            return None
        # print("test4")
        # find the z coordinate on the 3D BB
        if detection.mask.data:
            roi = roi[roi > 0]
            bb_center_z_coord = np.median(roi)
        
        else:
            bb_center_z_coord = depth_image[int(center_y)][int(
                center_x)] / self.depth_image_units_divisor
        # print("test5")
        z_diff = np.abs(roi - bb_center_z_coord)
        mask_z = z_diff <= self.maximum_detection_threshold
        if not np.any(mask_z):
            return None
        # print("test6")
        roi = roi[mask_z]
        z_min, z_max = np.min(roi), np.max(roi)
        z = (z_max + z_min) / 2
        # print("test7")
        if z == 0:
            return None
        # print("test8")
        # project from image to world space
        k = depth_info.k
        px, py, fx, fy = k[2], k[5], k[0], k[4]
        x = z * (center_x - px) / fx
        y = z * (center_y - py) / fy
        w = z * (size_x / fx)
        h = z * (size_y / fy)
        # print("test9")
        # create 3D BB
        msg = BoundingBox3D()
        msg.center.position.x = x
        msg.center.position.y = y
        msg.center.position.z = z
        msg.size.x = w
        msg.size.y = h
        msg.size.z = float(z_max - z_min)
        # print("test10")
        return msg

    @staticmethod
    def transform_3d_box(
        bbox: BoundingBox3D,
        translation: np.ndarray,
        rotation: np.ndarray
    ) -> BoundingBox3D:

        # position
        position = PoseDetection.qv_mult(
            rotation,
            np.array([bbox.center.position.x,
                      bbox.center.position.y,
                      bbox.center.position.z])
        ) + translation

        bbox.center.position.x = position[0]
        bbox.center.position.y = position[1]
        bbox.center.position.z = position[2]

        # size
        size = PoseDetection.qv_mult(
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
    def qv_mult(q: np.ndarray, v: np.ndarray) -> np.ndarray:
        q = np.array(q, dtype=np.float64)
        v = np.array(v, dtype=np.float64)
        qvec = q[1:]
        uv = np.cross(qvec, v)
        uuv = np.cross(qvec, uv)
        return v + 2 * (uv * q[0] + uuv)
# Seccion de ejecucion de clase main
def main(args = None):
    # Inicializacion de nodo
    rclpy.init()

    # Ejecucion de clase de nodo
    node = PoseDetection()

    # Automatizacion de activacion automatica de nodo
    node.trigger_configure()
    node.trigger_activate()

    # Ejecucion continua de nodo
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# Ejecucion de modo main
if __name__=="__main__":
    main()