add_executable(test main.cpp)
target_link_libraries(test PRIVATE fabrik)
target_include_directories(test PRIVATE ../src)
