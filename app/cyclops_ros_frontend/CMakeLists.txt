target_compile_features(cyclops_ros_frontend PRIVATE cxx_std_17)
target_include_directories(
  cyclops_ros_frontend
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/..
  ${catkin_INCLUDE_DIRS}
)
target_compile_options(cyclops_ros_frontend PUBLIC -march=native)

target_sources(
  cyclops_ros_frontend
  PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/config.cpp
  ${CMAKE_CURRENT_LIST_DIR}/config.hpp
  ${CMAKE_CURRENT_LIST_DIR}/distortion.cpp
  ${CMAKE_CURRENT_LIST_DIR}/distortion.hpp
  ${CMAKE_CURRENT_LIST_DIR}/main.cpp
  ${CMAKE_CURRENT_LIST_DIR}/throttle.cpp
  ${CMAKE_CURRENT_LIST_DIR}/throttle.hpp
  ${CMAKE_CURRENT_LIST_DIR}/tracker.cpp
  ${CMAKE_CURRENT_LIST_DIR}/tracker.hpp)

add_dependencies(cyclops_ros_frontend ${catkin_EXPORTED_TARGETS})
