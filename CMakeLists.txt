cmake_minimum_required( VERSION 2.8 )
project( stereoCalib )

set( CMAKE_CXX_FLAGS "-std=c++11" )
set( OpenCV_DIR /home/allride/software/opencv-3.4.1/build)
find_package( OpenCV 3 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( stereoCalib main.cpp )
add_executable( stereoCalib1 example.cpp)
target_link_libraries( stereoCalib ${OpenCV_LIBS} )
target_link_libraries( stereoCalib1 ${OpenCV_LIBS} )