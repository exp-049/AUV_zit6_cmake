# DepthSensor: depth-sensor facade and depth backends only.
target_sources(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Depth_Sensor_Driver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/I2C_DepthBackend.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/M14_UART_Backend.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/UART_DepthBackend.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
