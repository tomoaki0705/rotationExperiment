# minimum requirement
cmake_minimum_required(VERSION 3.6)

# solution
project(rotationExperiment)
set(TARGET rotationGUI)


# Define the available layout options
set(LAYOUT_JP "LAYOUT_JP")
set(LAYOUT_EN "LAYOUT_EN")
set(LAYOUT_OPTIONS "LAYOUT_JP" "LAYOUT_EN")

# Prompt the user to choose the layout option using a drop-down list
set(LAYOUT_CHOICE ${LAYOUT_JP} CACHE STRING "Choose layout option" FORCE)
set_property(CACHE LAYOUT_CHOICE PROPERTY STRINGS ${LAYOUT_OPTIONS})

# Set the SHIFT_2 variable based on the chosen layout option
set(SHIFT_1 "!")
set(SHIFT_3 "#")
set(SHIFT_4 "$")
set(SHIFT_5 "%")
if(${LAYOUT_CHOICE} STREQUAL ${LAYOUT_JP})
    set(SHIFT_2 "\"")
    set(SHIFT_6 "&")
elseif(${LAYOUT_CHOICE} STREQUAL ${LAYOUT_EN})
    set(SHIFT_2 "@")
    set(SHIFT_6 "^")
else()
    message(FATAL_ERROR "Invalid layout choice. Choose either ${LAYOUT_JP} or ${LAYOUT_EN}.")
endif()

# Configure the constant.in file with the SHIFT_2 variable
configure_file(cmake/constant.in constant.h @ONLY)

# find OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR})

add_executable(${TARGET} main.cpp rotationExperiment.cpp)

target_link_libraries(${TARGET} ${OpenCV_LIBS})
