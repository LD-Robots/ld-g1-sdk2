add_executable(activate activate.cpp)
target_link_libraries(activate unitree_sdk2)
target_compile_features(activate PUBLIC cxx_std_17)

add_executable(set_mode set_mode.cpp)
target_link_libraries(set_mode unitree_sdk2)
target_compile_features(set_mode PUBLIC cxx_std_17)
