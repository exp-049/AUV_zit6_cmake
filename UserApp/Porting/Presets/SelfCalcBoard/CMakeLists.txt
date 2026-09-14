# Self calculation-board UART4 DMA owner. This is selected by the preset and
# is not instantiated independently by DepthSensor or Pushrod.
target_sources(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Self_CalcBoard_Porting.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Pushrod/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../Peripherals/DepthSensor/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../Peripherals/Pushrod/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../Peripherals/Presets/SelfCalcBoard/inc
)
