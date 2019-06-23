cmake_minimum_required(VERSION 3.9)

# Includes
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Flags
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -Wextra $ENV{STDLIB}")

# Tests
file(GLOB SOURCES "test/*.cpp")
add_executable(hot_lava ${SOURCES})
target_include_directories(hot_lava PUBLIC
    ${PROJECT_SOURCE_DIR}/modules/catch2
    ${CMAKE_CURRENT_SOURCE_DIR}/test
)
