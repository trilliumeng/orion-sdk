add_executable(GeoPoint
    GeoPoint.c
)
target_include_directories(GeoPoint PRIVATE
    ../../Communications
    ../../Utils
)

target_link_libraries(GeoPoint PRIVATE
    OrionComm
    OrionUtils
)

if(CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(GeoPoint PRIVATE
        ../../Communications/debug
        ../../Utils/debug
    )
endif()

if(NOT CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(GeoPoint PRIVATE
        ../../Communications/release
        ../../Utils/release
    )
endif()

if(WIN32)
    target_link_libraries(GeoPoint PRIVATE
        ws2_32
    )
endif()

install(TARGETS GeoPoint
    BUNDLE DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
