cmake_minimum_required(VERSION 2.8.3)
project(sbf_ros_bridge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rtklib_msgs
)

catkin_package(
   CATKIN_DEPENDS
   roscpp
   std_msgs
   rtklib_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sbf_tcp src/sbf_tcp.cpp)
target_link_libraries(sbf_tcp ${catkin_LIBRARIES})
add_dependencies(sbf_tcp ${catkin_EXPORTED_TARGETS})

add_executable(sbf_udp src/sbf_udp.cpp)
target_link_libraries(sbf_udp ${catkin_LIBRARIES})
add_dependencies(sbf_udp ${catkin_EXPORTED_TARGETS})

add_executable(sbf_serial src/sbf_serial.cpp)
target_link_libraries(sbf_serial ${catkin_LIBRARIES})
add_dependencies(sbf_serial ${catkin_EXPORTED_TARGETS})
