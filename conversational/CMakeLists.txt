find_package(CURL REQUIRED)

add_executable(conv_main conv_main.cpp)
target_compile_features(conv_main PRIVATE cxx_std_17)
target_compile_definitions(conv_main
  PRIVATE WHISPER_MODEL_PATH="${CMAKE_SOURCE_DIR}/thirdparty/whisper.cpp/models/ggml-tiny.en.bin"
)
target_include_directories(conv_main
  PRIVATE ${CMAKE_SOURCE_DIR}/thirdparty/rnnoise/include
)
target_link_libraries(conv_main unitree_sdk2 whisper rnnoise CURL::libcurl)
