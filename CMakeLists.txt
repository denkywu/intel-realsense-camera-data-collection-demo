# cmake needs this line
cmake_minimum_required(VERSION 3.1.0)

# Define project name
project(DemoRealSense)

# Enable C++11
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Find librealsense2 installed package
find_package(realsense2 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Add includes
include_directories( ${CMAKE_CURRENT_LIST_DIR} )

# Declare the executable target built from your sources
add_executable(
	${PROJECT_NAME} 
	mainDemo.cpp 
	realSenseData.cpp
)

# Link your application with some libraries
target_link_libraries(
	${PROJECT_NAME} 
	${realsense2_LIBRARY} 
	${OpenCV_LIBS} 
	${PCL_LIBRARIES}
)
