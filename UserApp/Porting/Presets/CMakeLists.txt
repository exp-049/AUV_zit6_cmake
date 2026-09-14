# The selected preset is the composition root. It owns the complete hardware
# pairing and no generic Shared module is needed.
target_sources(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Peripheral_Preset.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/HAL_UART_RxEventDispatcher.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/SelfCalcBoard/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../DepthSensor/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../Pushrod/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Peripherals/DepthSensor/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Peripherals/Pushrod/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Peripherals/Presets/SelfCalcBoard/inc
)

add_subdirectory(SelfCalcBoard)
