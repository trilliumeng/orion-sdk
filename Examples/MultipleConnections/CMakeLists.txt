add_executable(MultipleConnections
    MultipleConnections.c
)
target_include_directories(MultipleConnections PRIVATE
    ../../Communications
    ../../Utils
)

target_link_libraries(MultipleConnections PRIVATE
    OrionComm
    OrionUtils
)

if(CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(MultipleConnections PRIVATE
        ../../Communications/debug
        ../../Utils/debug
    )
endif()

if(NOT CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(MultipleConnections PRIVATE
        ../../Communications/release
        ../../Utils/release
    )
endif()

if(WIN32)
    target_link_libraries(MultipleConnections PRIVATE
        ws2_32
    )
endif()

