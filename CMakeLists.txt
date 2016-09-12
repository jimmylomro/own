cmake_minimum_required(VERSION 2.6)

project(own)

find_package(OpenCV REQUIRED)
MESSAGE(STATUS "LINK LIBRARIES ${OpenCV_LIBS}")

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)


# bfm
include_directories(include)
link_directories(${OpenCV_LIB_DIR})
add_library(bfm_static STATIC  src/bfm.cpp)
target_link_libraries(bfm_static ${OpenV_LIBS})


# own
include_directories(include)
link_directories(${OpenCV_LIB_DIR})
add_library(own_static STATIC src/own.cpp src/nms.cpp)
target_link_libraries(own_static bfm_static ${OpenV_LIBS})


# Training
link_directories(${OpenCV_LIB_DIR})
add_executable(own_train src/own_train.cpp)
target_link_libraries(own_train bfm_static ${OpenCV_LIBS})


# bfm usage example
link_directories(${OpenCV_LIB_DIR})
add_executable(bfm_example src/bfm_example.cpp)
target_link_libraries(bfm_example bfm_static ${OpenCV_LIBS})


# own usage example
link_directories(${OpenCV_LIB_DIR})
add_executable(own_example src/own_example.cpp)
target_link_libraries(own_example own_static ${OpenCV_LIBS})


