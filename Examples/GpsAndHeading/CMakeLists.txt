cmake_minimum_required(VERSION 3.16)
project(GpsAndHeading VERSION 1.0 LANGUAGES C CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(GpsAndHeading
    GpsAndHeading.c
)
target_include_directories(GpsAndHeading PRIVATE
    ../../Communications
    ../../Utils
)

target_link_libraries(GpsAndHeading PRIVATE
    OrionComm
    OrionUtils
)

if(CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(GpsAndHeading PRIVATE
        ../../Communications/debug
        ../../Utils/debug
    )
endif()

if(NOT CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(GpsAndHeading PRIVATE
        ../../Communications/release
        ../../Utils/release
    )
endif()

if(WIN32)
    target_link_libraries(GpsAndHeading PRIVATE
        ws2_32
    )
endif()

install(TARGETS GpsAndHeading
    BUNDLE DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
