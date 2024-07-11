# CMakeList.txt : Top-level CMake project file, do global configuration
# and include sub-projects here.
#
cmake_minimum_required (VERSION 3.8)

set(CMAKE_TOOLCHAIN_FILE "C:/src/vcpkg/scripts/buildsystems/vcpkg.cmake")
set(VCPKG_CHAINLOAD_TOOLCHAIN_FILE "C:/src/emsdk/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake")
set(VCPKG_TARGET_TRIPLET "wasm32-emscripten")
set(CMAKE_BUILD_TYPE Release)


# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("esp32_sound_localization")

# Use ceres-solver version 2.0.0#6, because otherwise it will not compile with emscripten
# I didn't try with version 2.2.0, because vcpkg only had up to 2.1.0#5
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(wasm_module wasm-module/wasm_module.cpp)
set_target_properties(wasm_module PROPERTIES
    LINK_FLAGS "-sEXPORTED_FUNCTIONS=['_malloc','_free'] -sEXPORTED_RUNTIME_METHODS=['ccall','cwrap'] -sWASM_BIGINT -O2 -sASSERTIONS" #-sMODULARIZE 
)
target_link_libraries(wasm_module Ceres::ceres)
target_link_libraries(wasm_module Eigen3::Eigen)

# emscripten commands:
# emcmake cmake -S. -Bbuild
# cd build
# emmake make