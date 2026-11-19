# Guard the Video Player from being built if the required dependencies are not found
find_package(PkgConfig REQUIRED)
if(NOT PkgConfig_FOUND)
    message(STATUS "ATTENTION: Video Player example not configured, PkgConfig not found")
    return()
endif()

find_package(JPEG)

if(NOT JPEG_FOUND)
    message(STATUS "ATTENTION: Video Player example not configured, libjpeg development libraries not found")
    return()
endif()

pkg_check_modules(LIBAV QUIET IMPORTED_TARGET
    libavcodec
    libavformat
    libavutil
    libswscale
)

if(NOT LIBAV_FOUND)
    message(STATUS "ATTENTION: Video Player example not configured, libav development libraries not found")
    return()
endif()

add_executable(VideoPlayer
    FFmpeg.c
    KlvParser.c
    KlvTree.c
    StreamDecoder.c
    VideoPlayer.c
)
target_include_directories(VideoPlayer PRIVATE
    ../../Communications
    ../../Utils
)


target_link_libraries(VideoPlayer PRIVATE
    JPEG::JPEG
    OrionComm
    OrionUtils
    PkgConfig::LIBAV
)

if(CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(VideoPlayer PRIVATE
        ../../Communications/debug
        ../../Utils/debug
    )
endif()

if(NOT CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(VideoPlayer PRIVATE
        ../../Communications/release
        ../../Utils/release
    )
endif()

if(WIN32)
    target_link_libraries(VideoPlayer PRIVATE
        ws2_32
    )
endif()

install(TARGETS VideoPlayer
    BUNDLE DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

