# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# "Trim" the build. Include the minimal set of components, main, and anything it depends on.
idf_build_set_property(MINIMAL_BUILD ON)
project(t2v_mod_fw)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DZENOHPICO_PLATFORM_ESP32")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DZENOH_ESPIDF=1")
add_definitions(-DZENOH_FREERTOS_LWIP)