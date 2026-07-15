# Porting: hardware-specific porting layer
target_sources(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/INS_Porting.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MotionController_Porting.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MS5837_Porting.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/DepthCalcBoard_Porting.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/USBL_Porting.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
