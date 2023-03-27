cmake_minimum_required(VERSION 3.5.1)
project(f446_drone_shield_ros)
set(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -pg -O2")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}) 
find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)

SET(Boost_USE_STATIC_LIBS OFF)
SET(Boost_USE_MULTITHREAD ON)
FIND_PACKAGE(Boost 1.42.0 REQUIRED COMPONENTS system thread regex)
IF(Boost_FOUND)
  INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
  LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})
ENDIF(Boost_FOUND)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

set(SRC ${SRC} 
    src/timer.cpp

    src/serial_communicator.cpp
    src/serial_comm_ros.cpp
    src/crc16.cpp
)

set(INCLUDE_DIRS ${INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})
set(INCLUDE_DIRS ${INCLUDE_DIRS} include)

catkin_package(
  DEPENDS Eigen
  CATKIN_DEPENDS roscpp
  INCLUDE_DIRS include
)

include_directories( ${INCLUDE_DIRS} )
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${Eigen_INCLUDE_DIRS} )

# executables
add_executable(serial_comm_node src/node_serial_comm.cpp ${SRC})
target_link_libraries (serial_comm_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${Boost_REGEX_LIBRARY})

add_executable(sensor_publisher_node src/node_sensor_publisher.cpp ${SRC})
target_link_libraries (sensor_publisher_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(test_publisher_node src/test_publisher.cpp ${SRC})
target_link_libraries (test_publisher_node ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

