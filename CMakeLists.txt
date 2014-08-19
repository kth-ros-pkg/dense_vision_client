cmake_minimum_required(VERSION 2.8.3)
project(dense_vision_client)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge sensor_msgs stereo_msgs tf
)
find_package(Boost REQUIRED COMPONENTS system thread)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dense_vision_client_nodelet
  CATKIN_DEPENDS roscpp cv_bridge sensor_msgs stereo_msgs tf
  DEPENDS boost opencv
)

add_executable(dense_vision_client_node src/dense_vision_client_node.cpp src/DenseVisionClient.cpp)
target_link_libraries(dense_vision_client_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}  ${Boost_LIBRARIES})

add_library(dense_vision_client_nodelet src/dense_vision_client_nodelet.cpp src/DenseVisionClient.cpp)
target_link_libraries(dense_vision_client_nodelet ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}  ${Boost_LIBRARIES})

install(TARGETS dense_vision_client_node dense_vision_client_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES dense_vision_client_nodelet_plugin.xml 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)