add_executable(SendConfig
    SendConfig.c
)
target_include_directories(SendConfig PRIVATE
    ../../Communications
    ../../Utils
)

target_link_libraries(SendConfig PRIVATE
    OrionComm
    OrionUtils
)

if(CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(SendConfig PRIVATE
        ../../Communications/debug
        ../../Utils/debug
    )
endif()

if(NOT CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(SendConfig PRIVATE
        ../../Communications/release
        ../../Utils/release
    )
endif()

if(WIN32)
    target_link_libraries(SendConfig PRIVATE
        ws2_32
    )
endif()

install(TARGETS SendConfig
    BUNDLE DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
