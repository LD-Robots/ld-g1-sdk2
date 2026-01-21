add_executable(activate activate.cpp)
target_link_libraries(activate unitree_sdk2)

add_executable(set_mode set_mode.cpp)
target_link_libraries(set_mode unitree_sdk2)
