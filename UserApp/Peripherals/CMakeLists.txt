# Peripherals: hardware driver abstraction
target_sources(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/INS_Driver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MotionController_Driver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MS5837_Driver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/USBL_Driver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/I2C_DepthBackend.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/UART_DepthBackend.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
