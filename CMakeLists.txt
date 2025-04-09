cmake_minimum_required(VERSION 3.10)

# set the project name
project(FABRIK_Full_Body_IK)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)

# add the executable
add_subdirectory(src)
add_subdirectory(test)