PROJECT(vo2)
cmake_minimum_required(VERSION 2.6)
#set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_VERBOSE_MAKEFILE OFF)
FIND_PACKAGE(OpenCV REQUIRED )

ADD_EXECUTABLE(recipe recipe.cpp feature_tracker.h)
TARGET_LINK_LIBRARIES(recipe ${OpenCV_LIBS})

add_subdirectory(5point)
ADD_EXECUTABLE(vo_5points vo_5points.cpp feature_tracker.h)
TARGET_LINK_LIBRARIES(vo_5points 5point ${OpenCV_LIBS})
