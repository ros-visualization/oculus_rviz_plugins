cmake_minimum_required(VERSION 2.8.3)
project(oculus_rviz_plugins)

# Qt
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS -fPIC)

# Catkin
find_package(catkin REQUIRED COMPONENTS 
  rviz
  roscpp
  tf
)

# Oculus SDK
FIND_PACKAGE( OculusSDK REQUIRED )
link_directories(${OculusSDK_LIBRARY_DIRS})

# Boost
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES oculus_rviz_plugins
  CATKIN_DEPENDS rviz
  DEPENDS OculusSDK
)

find_package(PkgConfig)
pkg_check_modules(OGRE OGRE)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OculusSDK_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
)

# run Qt moc generator
qt4_wrap_cpp(MOC_FILES
  include/oculus_rviz_plugins/oculus_display.h
  include/oculus_rviz_plugins/fixed_view_controller.h
)

add_library(oculus_rviz_plugins
  src/ogre_oculus.cpp
  src/oculus_display.cpp
  src/fixed_view_controller.cpp
  ${MOC_FILES}
)

target_link_libraries(oculus_rviz_plugins
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  ${OculusSDK_LIBRARIES}
  ${BOOST_LIBRARIES}
)


#############
## Install ##
#############

install(TARGETS oculus_rviz_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY ogre_media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY
  icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
