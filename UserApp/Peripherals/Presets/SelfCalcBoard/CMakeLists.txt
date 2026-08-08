# Self calculation-board protocol backend. It implements both depth and
# pushrod contracts because the preset uses one UART4 protocol/link.
target_sources(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/UART_MS5837Backend.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../DepthSensor/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Pushrod/inc
)
