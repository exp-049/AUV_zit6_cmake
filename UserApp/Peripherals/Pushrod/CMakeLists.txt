# Pushrod: independent pushrod facade and physical backends.
target_sources(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Pushrod_Driver.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Pushrod_GPIO_Backend.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
