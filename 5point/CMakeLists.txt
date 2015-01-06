add_library(5point 5point.cpp Rpoly.cpp)
ADD_EXECUTABLE(5point_main         main.cpp)
TARGET_LINK_LIBRARIES(5point_main         ${OpenCV_LIBS} 5point)
