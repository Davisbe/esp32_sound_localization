# CMakeList.txt : CMake project for esp32_sound_localization, include source and define
# project specific logic here.
#

# Add source to this project's executable.
add_executable (esp32_sound_localization "wasm_module.cpp" "wasm_module.h")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET esp32_sound_localization PROPERTY CXX_STANDARD 20)
endif()

# TODO: Add tests and install targets if needed.
