cmake_minimum_required(VERSION 2.8.3)
project(mecanum_controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    dynamic_reconfigure
    nav_msgs
    realtime_tools
    tf
    urdf
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

# generate_messages(DEPENDENCIES std_msgs)

# generate_dynamic_reconfigure_options(
#   cfg/MecanumController.cfg
# )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS}
)

include_directories(
  include
)

include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/mecanum_controller.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES mecanum_controller_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
