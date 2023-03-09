cmake_minimum_required(VERSION 3.0.2)
project(rqt_plugins)

#message(STATUS, "RQT_PLUGIN --------> ${CATKIN_DEVEL_PREFIX}")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/modules")
#list(APPEND CMAKE_MODULE_PATH /usr/lib/x86_64-linux-gnu/cmake/Qt5)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qwt REQUIRED)

set(rqt_plugins_SRCS
  src/rqt_platform_panda_joint_position.cpp
  src/qtplotchecked.cpp
  src/qtpositionsliders.cpp
)

set(rqt_plugins_HDRS
  include/rqt_plugins/rqt_platform_panda_joint_position.h
  include/rqt_plugins/qtplotchecked.h
  include/rqt_plugins/qtpositionsliders.h
)

set(rqt_plugins_QRC
  resources/resources.qrc
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rqt_plugins
  CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

qt5_wrap_cpp(rqt_plugins_MOCS 
  ${rqt_plugins_HDRS}
)

qt5_add_resources(rqt_plugins_QRC_RCC
  ${rqt_plugins_QRC}
)

set(rqt_plugins_INCLUDE_DIRECTORIES
  include
  ${QWT_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	${CATKIN_DEVEL_PREFIX}/include
	${rqt_plugins_INCLUDE_DIRECTORIES}
	${catkin_INCLUDE_DIRS}
	${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
	${Qt5Gui_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
  ${rqt_plugins_SRCS}
  ${rqt_plugins_MOCS}
  ${rqt_plugins_QRC_RCC}
)

## Add cmake target dependencies of the executable
## same as for the library above

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
  ${QWT_LIBRARY}
)

message(STATUS "***************** ")
message(STATUS "***************** ")
message(STATUS "***************** ====> Qt5Widgets_INCLUDE_DIRS = ${Qt5Widgets_INCLUDE_DIRS},
    Qt5Core_INCLUDE_DIRS = ${Qt5Core_INCLUDE_DIRS}, Qt5Gui_INCLUDE_DIRS = ${Qt5Gui_INCLUDE_DIRS},  QWT_INCLUDE_DIRS = ${QWT_INCLUDE_DIRS}, QWT_LIBRARY = ${QWT_LIBRARY}, Qt5Widgets_FOUND = ${Qt5Widgets_FOUND}, Qt5Widgets_LIBRARIES = ${Qt5Widgets_LIBRARIES} ")
message(STATUS "***************** ")
message(STATUS "***************** ")

