cmake_minimum_required(VERSION 2.8.3)

project(csi_cameras)
find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport message_generation sensor_msgs)
catkin_package(CATKIN_DEPENDS cv_bridge image_transport message_runtime sensor_msgs)
find_package(OpenCV)
find_package(PkgConfig)

pkg_check_modules(GSTREAMER QUIET gstreamer-0.10)
if(NOT GSTREAMER_FOUND)
    set(GSTREAMER_VERSION_1_x TRUE)
endif()
if(GSTREAMER_VERSION_1_x)
    message(STATUS "gst 1.0")
    pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
    pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
else()
    message(STATUS "gst 0.1")
    pkg_check_modules(GSTREAMER REQUIRED gstreamer-0.10)
    pkg_check_modules(GST_APP REQUIRED gstreamer-app-0.10)
endif()
include_directories(${GLIB_INCLUDE_DIRS} ${GST_APP_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)
add_library(${PROJECT_NAME} SHARED 
	    src/csi_camera.cpp
	    src/configurationloader.cpp)
target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      ${GSTREAMER_LIBRARIES}
                      ${GST_APP_LIBRARIES})

add_executable(csi_cameras_publisher csi_cameras.cpp)
target_link_libraries(csi_cameras_publisher
                      ${catkin_LIBRARIES}
                      ${GSTREAMER_LIBRARIES}
                      ${GST_APP_LIBRARIES}
                      ${PROJECT_NAME})
