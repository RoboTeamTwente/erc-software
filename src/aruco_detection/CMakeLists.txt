cmake_minimum_required(VERSION 3.28.3) # Change this to your CMake version if older
project(ComputerVision LANGUAGES CXX)

find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(CalibrateCamera CalibrateCamera.cpp)
add_executable(DetectAruco DetectAruco.cpp)

target_link_libraries(CalibrateCamera ${OpenCV_LIBS})
target_link_libraries(DetectAruco ${OpenCV_LIBS})

