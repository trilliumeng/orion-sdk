add_executable(TrackSize
    TrackSize.c
)
target_include_directories(TrackSize PRIVATE
    ../../Communications
    ../../Utils
)

target_link_libraries(TrackSize PRIVATE
    OrionComm
    OrionUtils
)

if(CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(TrackSize PRIVATE
        ../../Communications/debug
        ../../Utils/debug
    )
endif()

if(NOT CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(TrackSize PRIVATE
        ../../Communications/release
        ../../Utils/release
    )
endif()

if(WIN32)
    target_link_libraries(TrackSize PRIVATE
        ws2_32
    )
endif()

install(TARGETS TrackSize
    BUNDLE DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
