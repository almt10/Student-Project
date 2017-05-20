cmake_minimum_required(VERSION 3.6)
project(thesis)

set(CMAKE_CXX_STANDARD 11)
find_package( OpenCV REQUIRED )
#add_executable(sender recv2.cpp)
set(SOURCE_FILES recv-jpeg.cpp)
add_executable(thesis recv-jpeg.cpp)

target_link_libraries( thesis ${OpenCV_LIBS} )