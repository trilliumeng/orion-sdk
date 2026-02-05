add_executable(EncodeDecode
    EncodeDecode.c
)
target_include_directories(EncodeDecode PRIVATE
    ../../Communications
    ../../Utils
)

target_link_libraries(EncodeDecode PRIVATE
    OrionComm
    OrionUtils
)

if(CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(EncodeDecode PRIVATE
        ../../Communications/debug
        ../../Utils/debug
    )
endif()

if(NOT CMAKE_BUILD_TYPE STREQUAL Debug)
    target_link_directories(EncodeDecode PRIVATE
        ../../Communications/release
        ../../Utils/release
    )
endif()

install(TARGETS EncodeDecode
    BUNDLE DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

