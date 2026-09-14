# DepthSensor: depth-sensor-specific board adapters.
target_sources(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/DepthCalcBoard_Porting.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MS5837_Porting.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
