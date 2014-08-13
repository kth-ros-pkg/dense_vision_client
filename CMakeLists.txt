cmake_minimum_required(VERSION 2.8.3)
project(dense_vision_client)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge sensor_msgs stereo_msgs
)
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
  ${OpenCV_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS roscpp cv_bridge sensor_msgs stereo_msgs
  DEPENDS boost opencv
)

add_executable(dense_vision_client_node src/dense_vision_client_node.cpp)
target_link_libraries(dense_vision_client_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}  ${Boost_LIBRARIES})

install(TARGETS dense_vision_client_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)