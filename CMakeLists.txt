
cmake_minimum_required(VERSION 3.1) # for the compiler standard setting
project(vector-field-histogram)
include(ExternalProject)

# Set GoogleTest directory
set(GTEST_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/lib/googletest)
# Set the C++ compiler standard to c++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include GoogleTest into the build
find_package(Threads)
ExternalProject_Add(googletest
    DOWNLOAD_NO_PROGRESS 1
    UPDATE_DISCONNECTED 1
    GIT_REPOSITORY https://github.com/google/googletest
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${GTEST_DIRECTORY}
)


# Set the build type
set(CMAKE_BUILD_TYPE Debug)

# Bring the headers into the project
include_directories(include)
# Include the GoogleTest headers and libraries
include_directories(${GTEST_DIRECTORY}/include)
link_directories(${GTEST_DIRECTORY}/lib)

# Can manually add the sources using the set command as follows:
set(SOURCES main.cpp src/HistogramGrid.cpp)

# However, the file(GLOB...) allows for wildcard additions:
#file(GLOB SOURCES "src/*.cpp")

add_executable(testing ${SOURCES})

add_dependencies(testing googletest)

target_link_libraries(testing gtest ${CMAKE_THREAD_LIBS_INIT})