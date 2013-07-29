cmake_minimum_required(VERSION 2.8.3)
project(rviz_oculus)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS -fPIC)

find_package(catkin REQUIRED COMPONENTS rviz roscpp)

SET(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
FIND_PACKAGE( OculusSDK REQUIRED )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rviz_oculus
  CATKIN_DEPENDS rviz
  DEPENDS OculusSDK
)

find_package(PkgConfig)
pkg_check_modules(OGRE OGRE)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OCULUS_SDK_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)

# run Qt moc generator
qt4_wrap_cpp(MOC_FILES
  include/rviz_oculus/oculus_display.h
)

add_library(rviz_oculus
  src/ogre_oculus.cpp
  src/oculus_display.cpp
  ${MOC_FILES}
)

target_link_libraries(rviz_oculus
  ${OCULUS_SDK_LIBRARIES}
)

