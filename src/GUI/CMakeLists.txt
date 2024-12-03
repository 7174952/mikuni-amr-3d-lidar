cmake_minimum_required(VERSION 3.0.2)
project(gui_guide)
set(ROS_DEPENDS
  roscpp
  std_msgs
  )
set(EXTERNAL_DEPENDS
  Qt5Widgets
  Qt5Multimedia
  )
set(EXTERNAL_DEPENDS_LIB
  Qt5::Widgets
  Qt5::Multimedia
  ) 
set(CMAKE_INCLUDE_CURRENT_DIR ON)
foreach(DEPEND ${EXTERNAL_DEPENDS})
  find_package(${DEPEND} REQUIRED)
endforeach(DEPEND)
find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDS})
catkin_package(
   CATKIN_DEPENDS
    ${ROS_DEPENDS}
  DEPENDS
    ${EXTERNAL_DEPENDS}
)
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
file(GLOB NODES
  "src/main.cpp"
)
set(CMAKE_AUTOMOC ON)  
set(CMAKE_AUTORCC ON)  

qt5_wrap_ui (UI_HDRS  src/mainwindow.ui)
foreach(NODE_FILE ${NODES})
  get_filename_component(NODE_NAME ${NODE_FILE} NAME_WE)
  add_executable(${NODE_NAME} ${NODE_FILE} src/main.cpp src/mainwindow.cpp src/mainwindow.h src/mainwindow.ui)
  add_dependencies(${NODE_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
  target_link_libraries (${NODE_NAME} ${catkin_LIBRARIES}  ${EXTERNAL_DEPENDS_LIB})
  list(APPEND NODE_LIST ${NODE_NAME})
endforeach(NODE_FILE)
install(TARGETS ${NODE_LIST}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

