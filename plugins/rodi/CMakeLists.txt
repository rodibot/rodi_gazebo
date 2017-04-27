cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
set(GAZEBO_INCLUDE_DIRS "${GAZEBO_INCLUDE_DIRS} /usr/include/")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

find_library(MICROHTTP_LIB microhttpd)
if(NOT MICROHTTP_LIB)
  message(FATAL_ERROR "microhttpd library not found")
endif()

add_library(model_rodi SHARED model_rodi.cc)
target_link_libraries(model_rodi ${GAZEBO_LIBRARIES} microhttpd)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

