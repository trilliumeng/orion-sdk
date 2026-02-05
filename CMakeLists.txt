cmake_minimum_required(VERSION 3.16)
project(Public VERSION 1.0 LANGUAGES C CXX)

# Check if the generated files exist, if they do not...
if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/Communications/OrionPublicPacket.c")
    message("Cannot find OrionPublicPacket.c, running protogen")
    # If on linux run shell script, otherwise run the windows bat file
    if(UNIX)
        SET(ProtogenScript GenerateOrionPublicPacket.sh)
    else()
        SET(ProtogenScript GenerateOrionPublicPacketWin.bat)
    endif()
    # run the protogen generation script
    execute_process(
        COMMAND ${CMAKE_CURRENT_LIST_DIR}/${ProtogenScript}
        TIMEOUT 120000
        WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
        OUTPUT_VARIABLE protogenOutput
        RESULT_VARIABLE protogenStatus
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    message(${protogenOutput})

endif()
SET(PROTOGEN_FILES_DIR ${CMAKE_CURRENT_LIST_DIR}/Communications)
Set(PROTOGEN_EXPORTS
    ${PROTOGEN_FILES_DIR}/OrionPublicPacket.c
    ${PROTOGEN_FILES_DIR}/fielddecode.h ${PROTOGEN_FILES_DIR}/fielddecode.c
    ${PROTOGEN_FILES_DIR}/fieldencode.h ${PROTOGEN_FILES_DIR}/fieldencode.c
    ${PROTOGEN_FILES_DIR}/floatspecial.h ${PROTOGEN_FILES_DIR}/floatspecial.c
    ${PROTOGEN_FILES_DIR}/scaleddecode.h ${PROTOGEN_FILES_DIR}/scaleddecode.c
    ${PROTOGEN_FILES_DIR}/scaledencode.h ${PROTOGEN_FILES_DIR}/scaledencode.c
)

add_subdirectory(Communications)
add_subdirectory(Utils)
add_subdirectory(Examples)
