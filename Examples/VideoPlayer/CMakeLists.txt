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
find_package(PkgConfig REQUIRED)
find_package(JPEG)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
    libavcodec
    libavformat
    libavutil
    libswscale
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

