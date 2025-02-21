# Importe de librerias de ejecucion
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Funcion para generacion de launch descriptiom
def generate_launch_description():
    # Declaracion de configuraciones
    model_type = LaunchConfiguration("model_type")
    model = LaunchConfiguration("model")
    tracker = LaunchConfiguration("tracker")
    device = LaunchConfiguration("device")
    enable = LaunchConfiguration("enable")
    threshold = LaunchConfiguration("threshold")
    input_image_topic = LaunchConfiguration("input_image_topic")
    image_reliability = LaunchConfiguration("image_reliability")
    namespace = LaunchConfiguration("namespace")

    # Asignacion y declaracion de parametros
    model_type_cmd = DeclareLaunchArgument(
        "model_type",
        default_value="YOLO", # Valor por defecto de modelo usado para la deteccion
        choices=["YOLO", "NAS"],
        description="Model type form Ultralytics (YOLO, NAS")
    model_cmd = DeclareLaunchArgument(
        "model",
        #default_value="./src/ntarobot_camera_pkg/models/yolov8n.pt",
        default_value="./src/ntarobot_camera_pkg/models/yolov8n-seg.pt",
        description="Model name or path")
    tracker_cmd = DeclareLaunchArgument(
        "tracker",
        default_value="bytetrack.yaml",
        description="Tracker name or path")
    device_cmd = DeclareLaunchArgument(
            "device",
            default_value="cuda:0",
            description="Device to use (GPU/CPU)")
    enable_cmd = DeclareLaunchArgument(
            "enable",
            default_value="True",
            description="Whether to start YOLOv8 enabled")
    threshold_cmd = DeclareLaunchArgument(
            "threshold",
            default_value="0.5",
            description="Minimum probability of a detection to be published")
    input_image_topic_cmd = DeclareLaunchArgument(
            "input_image_topic",
            default_value="/ntarobot/img",
            description="Name of the input image topic")
    image_reliability_cmd = DeclareLaunchArgument(
            "image_reliability",
            default_value="1",
            choices=["0", "1", "2"],
            description="Specific reliability QoS of the input image topic (0=system default, 1=Reliable, 2=Best Effort)")
    namespace_cmd = DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace for the nodes")  
    # Descripcion de nodos de ejecucion
    # Configuracion de nodo de recepcion de imagen comprimida
    republish_node_cmd = Node(
        package="ntarobot_camera_pkg",
        executable="republish_camera",
        name="republish_camera",
        namespace=namespace,
        parameters=[{
            "image_reliability": image_reliability,
        }]
        #remappings=[("/ntarobot/img", input_image_topic)]
    )
    # Configuracion para ejecucion primaria de nodo
    detector_node_cmd = Node(
        package="ntarobot_camera_pkg",
        executable="yolov8_node.py",
        name="yolov8_node",
        namespace=namespace,
        parameters=[{
            "model_type": model_type,
            "model": model,
            "device": device,
            "enable": enable,
            "threshold": threshold,
            "image_reliability": image_reliability,
        }],
        remappings=[("/ntarobot/img", input_image_topic)]
    )
    # Ejecucion para lectura de deteccion de objetos en el nodo
    tracking_node_cmd = Node(
        package="ntarobot_camera_pkg",
        executable="tracking_node.py",
        name="tracking_node",
        namespace=namespace,
        parameters=[{
            "tracker": tracker,
            "image_reliability": image_reliability
        }],
        remappings=[("/ntarobot/img", input_image_topic)]
    )
    # Visualizacion de deteccion
    debug_node_cmd = Node(
        package="ntarobot_camera_pkg",
        executable="debug_node.py",
        name="debug_node",
        namespace=namespace,
        parameters=[{"image_reliability": image_reliability}],
        remappings=[
            ("/ntarobot/img", input_image_topic),
            ("detections", "tracking")
        ]
    )

    ld = LaunchDescription()
    # Carga de configuraciones
    ld.add_action(model_type_cmd)
    ld.add_action(model_cmd)
    ld.add_action(tracker_cmd)
    ld.add_action(device_cmd)
    ld.add_action(enable_cmd)
    ld.add_action(threshold_cmd)
    ld.add_action(input_image_topic_cmd)
    ld.add_action(image_reliability_cmd)
    ld.add_action(namespace_cmd)
    # Carga de nodos de ejecucion
    ld.add_action(republish_node_cmd)
    ld.add_action(detector_node_cmd)
    ld.add_action(tracking_node_cmd)
    ld.add_action(debug_node_cmd)

    return ld