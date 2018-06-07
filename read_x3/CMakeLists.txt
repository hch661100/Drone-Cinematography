cmake_minimum_required(VERSION 2.8.3)
project(read_x3)



## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)

## is used, also find other catkin packages
catkin_package()
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  #opencv2
  roscpp
  sensor_msgs
  std_msgs
)

find_package(PkgConfig)

macro(find_gstreamer_component prefix pkg lib)
  pkg_check_modules(PC_${prefix} REQUIRED ${pkg})
  find_library(${prefix}_LIBRARIES NAMES ${lib} HINTS ${PC_${prefix}_LIBRARY_DIRS} ${PC_${prefix}_LIBDIR})
  list(APPEND gstreamer_INCLUDE_DIRS ${PC_${prefix}_INCLUDE_DIRS})
  list(APPEND gstreamer_LIBRARIES ${${prefix}_LIBRARIES})
  mark_as_advanced(${prefix}_LIBRARIES gstreamer_LIBRARIES gstreamer_INCLUDE_DIRS)
endmacro()

find_gstreamer_component(gst      gstreamer-1.0       gstreamer-1.0)
find_gstreamer_component(gstbase  gstreamer-base-1.0  gstbase-1.0)
find_gstreamer_component(gstapp   gstreamer-app-1.0   gstapp-1.0)
find_gstreamer_component(gstvideo gstreamer-video-1.0 gstvideo-1.0)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(read_x3 src/nv_cam.cpp)
link_directories(/usr/local/lib /usr/lib/arm-linux-gnueabihf /usr/lib)
target_link_libraries(
  read_x3
  dcam
  ${catkin_LIBRARIES})

add_executable(publish_img src/publish_img.cpp)
link_directories(/usr/local/lib /usr/lib/arm-linux-gnueabihf /usr/lib)
target_link_libraries(
  publish_img
  ${catkin_LIBRARIES})

add_executable(control_cam src/control_cam.cpp)
link_directories(/usr/local/lib /usr/lib/arm-linux-gnueabihf /usr/lib)
target_link_libraries(
  control_cam
  ${catkin_LIBRARIES})

