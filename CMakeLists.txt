# minimum requirement
cmake_minimum_required(VERSION 3.6)

# solution
project(rotationExperiment)
set(TARGET rotationGUI)

# find OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(${TARGET} main.cpp rotationExperiment.cpp)

target_link_libraries(${TARGET} ${OpenCV_LIBS})
