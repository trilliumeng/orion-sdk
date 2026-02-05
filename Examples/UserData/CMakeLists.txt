add_executable(UserData
    UserData.c
)
target_include_directories(UserData PRIVATE
    ../../Communications
    ../../Utils
)

target_link_libraries(UserData PRIVATE
    OrionComm
    OrionUtils
)

if(CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(UserData PRIVATE
        ../../Communications/debug
        ../../Utils/debug
    )
endif()

if(NOT CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(UserData PRIVATE
        ../../Communications/release
        ../../Utils/release
    )
endif()

if(WIN32)
    target_link_libraries(UserData PRIVATE
        ws2_32
    )
endif()

install(TARGETS UserData
    BUNDLE DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

