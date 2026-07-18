# Debug-only sources are deliberately selected by AUV_APP_MODE. NORMAL does
# not link a debug task or a USART1 reception callback.
if(AUV_APP_MODE STREQUAL "MS5837_CAL_DEBUG")
    target_sources(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/rtt_ms5837_cal_debug/Ms5837CalDebugTask.cpp
    )
elseif(AUV_APP_MODE STREQUAL "USBL_DEBUG")
    target_sources(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/rtt_usbl_debug/UsblDebugTask.cpp
    )
elseif(AUV_APP_MODE STREQUAL "INS_DEBUG")
    target_sources(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/rtt_ins_debug/InsDebugTask.cpp
    )
endif()

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/rtt_ms5837_cal_debug
)
