cmake_minimum_required(VERSION 3.8)
project(bridge_monitor_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Force Python 3.12 install path to match ROS Jazzy rclpy (avoid conda 3.13 mismatch)
set(PYTHON_INSTALL_DIR "lib/python3.12/site-packages" CACHE STRING "" FORCE)

set(msg_files
  "msg/RiskAttribution.msg"
  "msg/DistributionMetrics.msg"
  "msg/RiskStatus.msg"
  "msg/DomainRandomizationConfig.msg"
  "msg/ExperimentMetadata.msg"
)

set(srv_files
  "srv/InjectShift.srv"
  "srv/SetRandomization.srv"
  "srv/AcknowledgeRisk.srv"
  "srv/ExportExperiment.srv"
)

set(action_files
  "action/ExecuteScenario.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
