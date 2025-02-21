cmake_minimum_required(VERSION 3.8)
project(ntarobot_camera_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Importe de dependencias necesarias
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Seccion de inclusion de archivo inclue
include_directories(include)

# Creacion de nodos y ejecutables
add_executable(republish_camera src/republish_camera.cpp)
# Importe de dependencias de nodo
ament_target_dependencies(republish_camera rclcpp sensor_msgs cv_bridge OpenCV)
# Republicacion de camara de profundidad
add_executable(republish_depth src/republish_depth.cpp)
# Republicacion de camara de profundidad
ament_target_dependencies(republish_depth rclcpp sensor_msgs cv_bridge OpenCV)

# Instalacion de source para ejecucion c++ y python
ament_python_install_package(${PROJECT_NAME})

# Declaracion e ingreso de includes para paquetes de cpp
install(
  DIRECTORY include
  DESTINATION include
)

# Instalacion de paquetes 
install(
  DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME}
)

# Ejecutores de nodos c++
install(TARGETS 
  republish_camera
  republish_depth
  DESTINATION lib/${PROJECT_NAME})

# Ejecutores de nodos python
install(PROGRAMS
  ${PROJECT_NAME}/yolov8_node.py
  ${PROJECT_NAME}/tracking_node.py
  ${PROJECT_NAME}/debug_node.py
  ${PROJECT_NAME}/pos_detect.py
  ${PROJECT_NAME}/detect_3d_node.py
  ${PROJECT_NAME}/detect_pose_objetcs.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
