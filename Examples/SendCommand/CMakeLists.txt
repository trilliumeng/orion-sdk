add_executable(SendCommand
    SendCommand.c
)
target_include_directories(SendCommand PRIVATE
    ../../Communications
    ../../Utils
)

target_link_libraries(SendCommand PRIVATE
    OrionComm
    OrionUtils
)

if(CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(SendCommand PRIVATE
        ../../Communications/debug
        ../../Utils/debug
    )
endif()

if(NOT CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(SendCommand PRIVATE
        ../../Communications/release
        ../../Utils/release
    )
endif()

if(WIN32)
    target_link_libraries(SendCommand PRIVATE
        ws2_32
    )
endif()

install(TARGETS SendCommand
    BUNDLE DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
