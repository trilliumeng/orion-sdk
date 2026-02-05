set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(OrionComm
    OrionComm.c OrionComm.h
    OrionCommLinux.c
    OrionCommWindows.c
    ${PROTOGEN_EXPORTS}
)
target_include_directories(OrionComm PUBLIC
    ../Utils
)

install(TARGETS OrionComm
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    FRAMEWORK DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
