cmake_minimum_required(VERSION 3.22)

add_subdirectory(${CMAKE_SOURCE_DIR}/../../external/gtest googletest)

project(UnitTests)
add_executable(UnitTests)

file(GLOB_RECURSE TEST_SOURCES ${CMAKE_CURRENT_LIST_DIR}/../*.cpp)
target_sources(UnitTests PRIVATE ${TEST_SOURCES})

target_include_directories(UnitTests PRIVATE
    ${CMAKE_SOURCE_DIR}/../mocks
    ${CMAKE_CURRENT_LIST_DIR}/../../Core/Inc/
    ${CMAKE_SOURCE_DIR}/../../external/gtest/googletest/include
)

target_sources(UnitTests PRIVATE
    ${CMAKE_SOURCE_DIR}/../../Core/Src/control/bang_bang.c
    ${CMAKE_SOURCE_DIR}/../../Core/Src/control/coil_controller.c
    ${CMAKE_SOURCE_DIR}/../../Core/Src/control/combined_controller.c
    ${CMAKE_SOURCE_DIR}/../../Core/Src/control/control_reference.c
    ${CMAKE_SOURCE_DIR}/../../Core/Src/control/fan_controller.c
    ${CMAKE_SOURCE_DIR}/../../Core/Src/control/pid.c   

    ${CMAKE_SOURCE_DIR}/../../Core/Src/measurements/speed.c
    ${CMAKE_SOURCE_DIR}/../../Core/Src/measurements/temperature.c

    ${CMAKE_SOURCE_DIR}/../../Core/Src/utils/char_translator.c
    ${CMAKE_SOURCE_DIR}/../../Core/Src/utils/iir_filter.c

    ${CMAKE_SOURCE_DIR}/../mocks/stm32f4xx_hal.c
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

target_link_libraries(UnitTests PRIVATE gtest gtest_main gmock)

enable_testing()
add_test(NAME UnitTests COMMAND UnitTests)
