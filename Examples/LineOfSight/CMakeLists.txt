add_executable(LineOfSight
    LineOfSight.c
)
target_include_directories(LineOfSight PRIVATE
    ../../Communications
    ../../Utils
)

target_link_libraries(LineOfSight PRIVATE
    OrionComm
    OrionUtils
)

if(CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(LineOfSight PRIVATE
        ../../Communications/debug
        ../../Utils/debug
    )
endif()

if(NOT CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(LineOfSight PRIVATE
        ../../Communications/release
        ../../Utils/release
    )
endif()

if(WIN32)
    target_link_libraries(LineOfSight PRIVATE
        ws2_32
    )
endif()

install(TARGETS LineOfSight
    BUNDLE DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
