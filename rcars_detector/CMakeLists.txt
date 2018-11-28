cmake_minimum_required(VERSION 2.8.3)
project(rcars_detector)

#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O3 -march=native")

set(${CMAKE_BUILD_TYPE} RELEASE)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  message_generation
  std_msgs
  geometry_msgs
  image_transport
  genmsg
  cmake_modules
)
find_package(OpenCV REQUIRED)
find_package(Eigen REQUIRED)
find_package(kindr REQUIRED)

add_message_files(FILES
  Tag.msg
  TagArray.msg
  TagPoses.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
   INCLUDE_DIRS
   CATKIN_DEPENDS cv_bridge roscpp geometry_msgs image_transport
   CATKIN_DEPENDS message_runtime
)


include_directories(
   cv2cg/
   ${catkin_INCLUDE_DIRS} 
   ${OpenCV_INCLUDE_DIRS} 
   ${Eigen_INCLUDE_DIRS}
   ${kindr_INCLUDE_DIRS}
)

add_executable(rcars_detector_node src/rcars_detector_node.cpp)
add_dependencies(rcars_detector_node rcars_detector_generate_messages_cpp)
target_link_libraries(rcars_detector_node ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
