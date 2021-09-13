target_sources(
  cyclops_ros_frontend_visual PRIVATE ${CMAKE_CURRENT_LIST_DIR}/main.cpp)

target_compile_features(cyclops_ros_frontend_visual PRIVATE cxx_std_17)
target_include_directories(
  cyclops_ros_frontend_visual
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${catkin_INCLUDE_DIRS})
