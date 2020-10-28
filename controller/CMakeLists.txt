cmake_minimum_required(VERSION 2.8.3)
project(controller)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  tf
  dynamic_reconfigure
  move_base_msgs
  actionlib_msgs
  controller_msgs
  roslib

)


generate_dynamic_reconfigure_options(cfg/lyapunov.cfg)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS	
    geometry_msgs
    nav_msgs
    roscpp
    rospy
    std_msgs
    tf
    move_base_msgs 
    std_msgs 
    actionlib_msgs
)
###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)





add_library(initial_testings
	src/talker.cpp
)

target_link_libraries(initial_testings ${catkin_LIBRARIES})


  


add_executable(tracking_controller
  src/RosClass.cpp
  src/LyapunovController.cpp 
  src/Transformations.cpp
  src/ControllerInterface.cpp
)
add_dependencies(tracking_controller ${PROJECT_NAME}_gencfg)

#For matplotlib 
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})
target_link_libraries(tracking_controller
  ${catkin_LIBRARIES} ${PYTHON_LIBRARIES}
)




if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(initial_testings-test launch/gtest.test tests/test_example.cpp)
  target_link_libraries(initial_testings-test ${catkin_LIBRARIES} initial_testings)
endif()


