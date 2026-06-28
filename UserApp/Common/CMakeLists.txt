# Common: shared context structures
target_sources(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/AppContext.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MotionContext.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/SystemContext.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
