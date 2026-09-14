# Thirdparty: external libraries
#   Eigen       - header-only linear algebra library
#   cJSON       - lightweight JSON parser
target_sources(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/cJSON/cJSON.c
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}          # for <Eigen/Core>
    ${CMAKE_CURRENT_SOURCE_DIR}/cJSON    # for <cJSON.h>
)
