cmake_minimum_required(VERSION 3.16)
project(geometric_primatives LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(geo INTERFACE)
add_library(geo::geo ALIAS geo)
target_include_directories(geo INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

function(geo_set_warnings target)
    target_compile_options(${target} PRIVATE
        $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic -Wshadow -Werror>)
endfunction()

add_executable(geo_tests tests/primitives_test.cpp)
target_link_libraries(geo_tests PRIVATE geo)
geo_set_warnings(geo_tests)

add_executable(geo_shapes_tests tests/shapes_test.cpp)
target_link_libraries(geo_shapes_tests PRIVATE geo)
geo_set_warnings(geo_shapes_tests)

add_executable(geo_demo demo/demo.cpp demo/scene_view.cpp)
target_link_libraries(geo_demo PRIVATE geo)
geo_set_warnings(geo_demo)
if(NOT MSVC)
    target_compile_options(geo_demo PRIVATE -O2)
endif()

enable_testing()
add_test(NAME primitives COMMAND geo_tests)
add_test(NAME shapes COMMAND geo_shapes_tests)
