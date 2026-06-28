# Application layer: task entry points
target_sources(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/AppMain.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ControlTask.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/IICTask.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MicroRosTask.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MonitorTask.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
