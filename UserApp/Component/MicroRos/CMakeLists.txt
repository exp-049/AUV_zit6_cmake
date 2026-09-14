# MicroRos: ROS 2 communication wrappers
target_sources(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/MicroRosPublisher.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MicroRosService.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MicroRosSubscriber.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/MicroRosTransport.cpp
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
