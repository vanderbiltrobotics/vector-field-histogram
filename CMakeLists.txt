
cmake_minimum_required(VERSION 3.1) # for the compiler standard setting
project(vector-field-histogram)

#Set the C++ compiler standard to c++14
set (CMAKE_CXX_STANDARD 14)

#Bring the headers, such as Student.h into the project
include_directories(include)

#Can manually add the sources using the set command as follows:
set(SOURCES main.cpp src/HistogramGrid.cpp)

#However, the file(GLOB...) allows for wildcard additions:
#file(GLOB SOURCES "src/*.cpp")

add_executable(vfh ${SOURCES})