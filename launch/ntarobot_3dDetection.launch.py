# Seccion de importe de librerias
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Creacion de ejecucion de launch
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
    # Configuraciones de camara de profundidad
    input_depth_topic = LaunchConfiguration("input_depth_topic")
    depth_image_reliability = LaunchConfiguration("depth_image_reliability")
    input_depth_info_topic = LaunchConfiguration("input_depth_info_topic")
    depth_info_reliability = LaunchConfiguration("depth_info_reliability")
    depth_image_units_divisor = LaunchConfiguration("depth_image_units_divisor")
    target_frame = LaunchConfiguration("target_frame")
    maximum_detection_threshold = LaunchConfiguration("maximum_detection_threshold")
    # configuracion de nombre de proyecto
    namespace = LaunchConfiguration("namespace")

    # Declaracion y asignracion de parametros
    # Parametros de configuracion generales
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
    
    # Configuracion de parametros para camara de profundidad
    input_depth_topic_cmd = DeclareLaunchArgument(
        "input_depth_topic",
        default_value="/ntarobot/depth_img",
        description="Name of the input depth topic")
    depth_image_reliability_cmd = DeclareLaunchArgument(
        "depth_image_reliability",
        default_value="1",
        choices=["0", "1", "2"],
        description="Specific reliability QoS of the input depth image topic (0=system default, 1=Reliable, 2=Best Effort)")
    input_depth_info_topic_cmd = DeclareLaunchArgument(
        "input_depth_info_topic",
        default_value="/depth/camera_info",
        description="Name of the input depth info topic")
    depth_info_reliability_cmd = DeclareLaunchArgument(
        "depth_info_reliability",
        default_value="1",
        choices=["0", "1", "2"],
        description="Specific reliability QoS of the input depth info topic (0=system default, 1=Reliable, 2=Best Effort)")
    depth_image_units_divisor_cmd = DeclareLaunchArgument(
        "depth_image_units_divisor",
        default_value="1000",
        description="Divisor used to convert the raw depth image values into metres")
    target_frame_cmd = DeclareLaunchArgument(
        "target_frame",
        default_value="camera_link",
        description="Target frame to transform the 3D boxes")
    maximum_detection_threshold_cmd = DeclareLaunchArgument(
        "maximum_detection_threshold",
        default_value="0.3",
        description="Maximum detection threshold in the z axis")
    
    # Configuracion de nombre general de lanzamiento de proyecto
    namespace_cmd = DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace for the nodes")
    
    # Nodos de republidacion de imagenes comprimidas
    # Descripcion de nodos de ejecucion
    # Configuracion de nodo de recepcion de imagen comprimida
    republish_node_cmd = Node(
        package="ntarobot_camera_pkg",
        executable="republish_camera",
        name="republish_cam",
        namespace=namespace,
        parameters=[{
            "image_reliability": image_reliability,
        }]
        #remappings=[("/ntarobot/img", input_image_topic)]
    )

    # Configuracion de nodo de recepcion de imagen comprimida
    republish_node_2_cmd = Node(
        package="ntarobot_camera_pkg",
        executable="republish_depth",
        name="republish_depth",
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

    # Nodo de deteccion 3d
    detect_3d_node_cmd = Node(
        package="ntarobot_camera_pkg",
        executable="detect_3d_node.py",
        name="detect_3d_node",
        namespace=namespace,
        parameters=[{
            "target_frame": target_frame,
            "maximum_detection_threshold": maximum_detection_threshold,
            "depth_image_units_divisor": depth_image_units_divisor,
            "depth_image_reliability": depth_image_reliability,
            "depth_info_reliability": depth_info_reliability
        }],
        remappings=[
            ("/ntarobot/depth_img", input_depth_topic),
            ("/depth/camera_info", input_depth_info_topic),
            ("detections", "tracking")
        ]
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
            ("detections", "detections_3d")
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
    ld.add_action(input_depth_topic_cmd)
    ld.add_action(depth_image_reliability_cmd)
    ld.add_action(input_depth_info_topic_cmd)
    ld.add_action(depth_info_reliability_cmd)
    ld.add_action(depth_image_units_divisor_cmd)
    ld.add_action(target_frame_cmd)
    ld.add_action(maximum_detection_threshold_cmd)
    ld.add_action(namespace_cmd)
    # Carga de nodos de ejecucion
    ld.add_action(republish_node_cmd)
    ld.add_action(republish_node_2_cmd)
    ld.add_action(detector_node_cmd)
    ld.add_action(tracking_node_cmd)
    # ld.add_action(detect_3d_node_cmd)
    # ld.add_action(debug_node_cmd)

    return ld