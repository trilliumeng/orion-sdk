add_executable(PathTrack
    PathTrack.c
)
target_include_directories(PathTrack PRIVATE
    ../../Communications
    ../../Utils
)

target_link_libraries(PathTrack PRIVATE
    OrionComm
    OrionUtils
)

if(CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(PathTrack PRIVATE
        ../../Communications/debug
        ../../Utils/debug
    )
endif()

if(NOT CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(PathTrack PRIVATE
        ../../Communications/release
        ../../Utils/release
    )
endif()

if(WIN32)
    target_link_libraries(PathTrack PRIVATE
        ws2_32
    )
endif()

install(TARGETS PathTrack
    BUNDLE DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
