#!/usr/bin/env python3
# Seccion de importe de librerias
# Importe de librerias de ros2
import rclpy
from rclpy.lifecycle import LifecycleNode
# Ros para transformacion
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
# Importe de interfaces de mensaje
from yolov8_msgs.msg import DetectionArray, Detection
# Librerias de procesamiento de imagenes
from cv_bridge import CvBridge
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn

# Seccion de cracion de clase denodo
class PosDetectNode(LifecycleNode):
    # Constructor de clase
    def __init__(self) -> None:
        super().__init__("position_detection_node")

        # Seccion de declaracion de parametros
        self.declare_parameter("detections_topic", "/detections")

        # Creacion de variables de instancia 
        self.cv_bridge = CvBridge()
        #self.tf_listener = Buffer()

    # Funcion para declaracion de configuracion de lifecycle node
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        # Validacion de cambio de estado de nodo a configuracion
        self.get_logger().info(f"Configuring {self.get_name()}")

        # Asignacion de parametros
        self.topic_dect_ = self.get_parameter("detections_topic").get_parameter_value().string_value

        # Buffer de comunicacion
        #self.tf_listener = TransformListener(self.tf_buffer, self)
        
        return TransitionCallbackReturn.SUCCESS
    
    # Seccion de funcion de activacion de nodo 
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Activating {self.get_name()}")

        # Seccion de creacion de subscriptores.
        self.sub_dect_ = self.create_subscription(
            DetectionArray,
            self.topic_dect_,
            self.detect_callback,
            1
        )

        return TransitionCallbackReturn.SUCCESS
    
    # funcion de desactivacion
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Deactivating {self.get_name()}")
        # Eliminacion de subscriptores
        self.destroy_subscription(self.sub_dect_)
        
        return TransitionCallbackReturn.SUCCESS
    
    # Funcion de limpieza de nodo
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.get_logger().info(f"Cleaning up {self.get_name()}")
        #del self.tf_listener
        return TransitionCallbackReturn.SUCCESS
    
    # Funciones de callback de deteccion de datos
    def detect_callback(self, msg: DetectionArray) -> None:

        # Decta de array de deteccion
        detections_msg = msg
        # Descripcion de valores detectados
        detection: Detection
        # ciclo de visualizacion de detecciones conseguidas
        for detection in detections_msg.detections:
            #Validacion de deteccion de persona
            #detection: Detection
            if int(detection.class_id) == 0:
                
                # Funcion de calculo definida por matheus
                angulo_ = (
                    (70/2)*((640/2)-(detection.bbox.center.position.x))
                    )/(640/2)
                print(f"El angulo resultante es: {angulo_}")

def main():
    rclpy.init()
    node = PosDetectNode()
    node.trigger_configure()
    node.trigger_activate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()