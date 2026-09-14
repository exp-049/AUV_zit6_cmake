# Component: reusable business logic components
target_sources(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Chassis/CascadeController.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Chassis/ChassisManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Chassis/SetpointRouter.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ConfigService/ConfigService.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/RosLogger/RosLogger.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Safety/SafetyMonitor.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Safety/SoftWatchdog.cpp
)

# MicroRos: ROS 2 communication wrappers (formerly in UserApp/MicroRos)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/MicroRos)

# Subfolder include paths so external code can #include "ChassisManager.hpp" etc.
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/Chassis
    ${CMAKE_CURRENT_SOURCE_DIR}/Safety
    ${CMAKE_CURRENT_SOURCE_DIR}/RosLogger
    ${CMAKE_CURRENT_SOURCE_DIR}/ConfigService
    ${CMAKE_CURRENT_SOURCE_DIR}/HitlSimulator
)
