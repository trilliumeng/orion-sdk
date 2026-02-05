add_executable(CameraInfo
    CameraInfo.c
)
target_include_directories(CameraInfo PRIVATE
    ../../Communications
    ../../Utils
)

target_link_libraries(CameraInfo PRIVATE
    OrionComm
    OrionUtils
)

if(CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(CameraInfo PRIVATE
        ../../Communications/debug
        ../../Utils/debug
    )
endif()

if(NOT CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(CameraInfo PRIVATE
        ../../Communications/release
        ../../Utils/release
    )
endif()

if(WIN32)
    target_link_libraries(CameraInfo PRIVATE
        ws2_32
    )
endif()

