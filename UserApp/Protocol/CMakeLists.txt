# Protocol: wire-format definitions, frame parsers, checksums, and codecs.
target_sources(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/INS_Protocol.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ms5837_protocol.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/pushrod_protocol.c
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
