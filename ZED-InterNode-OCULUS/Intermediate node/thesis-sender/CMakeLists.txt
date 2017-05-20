cmake_minimum_required(VERSION 3.6)
project(thesis_sender)

set(CMAKE_CXX_STANDARD 11)
find_package( OpenCV REQUIRED )

set(SOURCE_FILES sender-jpeg.cpp)
add_executable(thesis_sender ${SOURCE_FILES})

target_link_libraries( thesis_sender ${OpenCV_LIBS} )