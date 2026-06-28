# Algorithm: control algorithms
target_sources(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/KinematicProfile.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/PID_Controller.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
