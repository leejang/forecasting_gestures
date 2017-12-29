cmake_minimum_required(VERSION 2.8.3)
project(forecasting_gestures)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  sensor_msgs
  std_msgs
  roscpp
  image_transport
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Image Converter
set(IMG_CONVERTER_SOURCE_FILES
  src/image_converter.cpp
  src/image_converter_node.cpp
)

set(IMG_CONVERTER_HEADER_FILES
  src/image_converter.h
)

# build! (Image Converter)
add_executable(image_converter ${IMG_CONVERTER_SOURCE_FILES} ${IMG_CONVERTER_HEADER_FILES})
target_link_libraries(image_converter ${catkin_LIBRARIES})
add_dependencies(image_converter ${catkin_EXPORTED_TARGETS})
