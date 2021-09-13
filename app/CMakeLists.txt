find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(cyclops_ros_backend "")
add_executable(cyclops_ros_frontend "")
add_executable(cyclops_ros_frontend_visual "")

set(cyclops_ros_common_libs Eigen3::Eigen ${catkin_LIBRARIES})
set(cyclops_ros_frontend_opencv_libs opencv_core opencv_calib3d opencv_optflow)

set(cyclops_ros_backend_libs cyclops ${cyclops_ros_common_libs})
set(
  cyclops_ros_frontend_libs
  ${cyclops_ros_frontend_opencv_libs} ${cyclops_ros_common_libs} Boost::boost)
set(cyclops_ros_visual_libs opencv_imgproc ${cyclops_ros_common_libs})

target_link_libraries(
  cyclops_ros_backend PRIVATE ${cyclops_ros_backend_libs})
target_link_libraries(
  cyclops_ros_frontend PRIVATE ${cyclops_ros_frontend_libs})
target_link_libraries(
  cyclops_ros_frontend_visual PRIVATE ${cyclops_ros_visual_libs})

add_subdirectory(cyclops_ros_backend)
add_subdirectory(cyclops_ros_frontend)
add_subdirectory(cyclops_ros_frontend_visual)

add_dependencies(cyclops_ros_backend cyclops_ros_generate_messages_cpp)
add_dependencies(cyclops_ros_frontend cyclops_ros_generate_messages_cpp)
add_dependencies(cyclops_ros_frontend_visual cyclops_ros_generate_messages_cpp)

install(
  TARGETS cyclops_ros_backend
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(
  TARGETS cyclops_ros_frontend
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(
  TARGETS cyclops_ros_frontend_visual
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
