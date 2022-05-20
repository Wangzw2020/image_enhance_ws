cmake_minimum_required(VERSION 2.8.3)
project(image_enhance)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge image_transport sensor_msgs roscpp rospy std_msgs)
find_package(OpenCV REQUIRED)

find_package(dlib REQUIRED)
find_package(Armadillo REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_image_transport
#  CATKIN_DEPENDS cv_bridge image_transport sensor_msgs
#  DEPENDS system_lib
)

include_directories(
 ${catkin_INCLUDE_DIRS} 
 ${OpenCV_INCLUDE_DIRS} 
 ${ARMADILLO_INCLUDE_DIRS}
 include
)

add_executable(
	${PROJECT_NAME}_node src/${PROJECT_NAME}.cpp
	src/util.cpp
    src/WTHE.cpp
    src/LDR.cpp
    src/AGCWD.cpp
    src/AGCIE.cpp
    src/IAGCWD.cpp
)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
	${PROJECT_NAME}_node 
	${catkin_LIBRARIES} 
	${OpenCV_LIBRARIES}
	${ARMADILLO_LIBRARIES}
	dlib::dlib
)


