cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(server)

add_executable(server
    server.cpp
    mavlink-bridge.cpp
)

find_package(MAVSDK REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-transport13 REQUIRED)

target_link_libraries(server
    MAVSDK::mavsdk
    gz-msgs10
    gz-transport13
    capnp
    pthread
)

if(NOT MSVC)
    add_compile_options(takeoff_and_land PRIVATE -Wall -Wextra)
else()
    add_compile_options(takeoff_and_land PRIVATE -W2)
endif()
