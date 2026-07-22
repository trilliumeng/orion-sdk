add_executable(MotionControl
    MotionControl.c
    analysis.c
)

target_include_directories(MotionControl PRIVATE
    ../../Communications
    ../../Utils
)

target_link_libraries(MotionControl PRIVATE
    OrionComm
    OrionUtils
    ws2_32
)

if(CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(MotionControl PRIVATE
        ../../Communications/debug
        ../../Utils/debug
    )
endif()

if(NOT CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(MotionControl PRIVATE
        ../../Communications/release
        ../../Utils/release
    )
endif()

install(TARGETS MotionControl
    BUNDLE DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
