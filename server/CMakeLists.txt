cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(server)

option(COMMA "Build for comma device" OFF)

add_executable(server
    server.cpp
    mavlink-bridge.cpp
)

if (COMMA)
    target_link_libraries(
        /data/mavsdk/include
        /data/mavsdk/lib
    )
else()
    find_package(MAVSDK REQUIRED)

    target_link_libraries(server
        MAVSDK::mavsdk
        capnp
        pthread
    )
endif()


if(NOT MSVC)
    add_compile_options(takeoff_and_land PRIVATE -Wall -Wextra)
else()
    add_compile_options(takeoff_and_land PRIVATE -W2)
endif()
