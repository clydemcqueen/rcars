cmake_minimum_required(VERSION 2.8.3)
project(rcars_visualization)

add_definitions(-Wfatal-errors -std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  std_msgs
  geometry_msgs
  image_transport
  cmake_modules
  message_filters
  rcars_detector
  tf
)
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)
find_package(kindr REQUIRED)


catkin_package(
   INCLUDE_DIRS
   CATKIN_DEPENDS cv_bridge roscpp geometry_msgs image_transport rcars_detector tf
)


include_directories(
   ${catkin_INCLUDE_DIRS} 
   ${OpenCV_INCLUDE_DIRS} 
   ${Eigen_INCLUDE_DIRS}
   ${kindr_INCLUDE_DIRS}
)

add_executable(rcars_image_visualization_node src/rcars_image_visualization_node.cpp)
target_link_libraries(rcars_image_visualization_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(rcars_tf_publisher_node src/rcars_tf_publisher_node.cpp)
target_link_libraries(rcars_tf_publisher_node ${catkin_LIBRARIES})

add_executable(rcars_path_visualization_node src/rcars_path_visualization_node.cpp)
target_link_libraries(rcars_path_visualization_node ${catkin_LIBRARIES})

add_executable(rcars_loop_closure_evaluation_node src/rcars_loop_closure_evaluation_node.cpp)
target_link_libraries(rcars_loop_closure_evaluation_node ${catkin_LIBRARIES})