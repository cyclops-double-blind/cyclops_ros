target_compile_features(cyclops_ros_backend PRIVATE cxx_std_17)

target_include_directories(
  cyclops_ros_backend
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/.. ${catkin_INCLUDE_DIRS})

add_dependencies(cyclops_ros_backend ${catkin_EXPORTED_TARGETS})

target_sources(
  cyclops_ros_backend
  PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/config.cpp
  ${CMAKE_CURRENT_LIST_DIR}/config.hpp
  ${CMAKE_CURRENT_LIST_DIR}/main.cpp
  ${CMAKE_CURRENT_LIST_DIR}/publisher.cpp
  ${CMAKE_CURRENT_LIST_DIR}/publisher.hpp
  ${CMAKE_CURRENT_LIST_DIR}/telemetry.initializer.cpp
  ${CMAKE_CURRENT_LIST_DIR}/telemetry.initializer.hpp
  ${CMAKE_CURRENT_LIST_DIR}/telemetry.keyframe.cpp
  ${CMAKE_CURRENT_LIST_DIR}/telemetry.keyframe.hpp
  ${CMAKE_CURRENT_LIST_DIR}/telemetry.sanity.cpp
  ${CMAKE_CURRENT_LIST_DIR}/telemetry.sanity.hpp
  ${CMAKE_CURRENT_LIST_DIR}/thread.data.cpp
  ${CMAKE_CURRENT_LIST_DIR}/thread.data.hpp
  ${CMAKE_CURRENT_LIST_DIR}/thread.optimizer.cpp
  ${CMAKE_CURRENT_LIST_DIR}/thread.optimizer.hpp)
