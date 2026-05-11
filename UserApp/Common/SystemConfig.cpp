#include "SystemConfig.hpp"

namespace auv {
namespace config {

SystemConfig sys_config = {
    .chassis = {
        .planner_enabled = false,
        .x = { 1.6, 0.0, 0.1, 0.3, 0.8, 0.6, 0.02, 0.01, 0.03, 0.1, 0.8, 0.4, 0.013, 0.015 },
        .y = { 1.6, 0.0, 0.1, 0.3, 0.6, 0.7, 0.02, 0.01, 0.03, 0.1, 0.6, 0.3, 0.013, 0.025 },
        .z = { 1.8, 0.1, 0.2, 0.2, 0.5, 0.6, 0.04, 0.02, 0.05, 0.15, 0.5, 0.3, 0.016, 0.04 },
        .yaw = { 2.0, 0.0, 0.1, 0.5, 1.5, 0.003, 0.001, 0.001, 0.01, 0.05, 1.0, 0.8, 0.0003, 0.0005 }
    },
    .ins = { 45.7749, 126.6765 },
    .soft_watchdog = { 3000, true, false, false },
    .sensors = { ZDataSource::USE_MS5837_Z },
    .simulation = { false, 35.0, 15.0, 1000.0 }
};

const ParamMeta SYSTEM_PARAMS[] = {
    {"z_data_sourse", &sys_config.sensors.z_data_source, ParamType::ENUM_Z},
    {"soft_watchdog.timeout_ms", &sys_config.soft_watchdog.timeout_ms, ParamType::UINT32},
    {"soft_watchdog.check_microros", &sys_config.soft_watchdog.check_microros, ParamType::BOOL},
    {"soft_watchdog.check_ins", &sys_config.soft_watchdog.check_ins, ParamType::BOOL},
    {"soft_watchdog.check_depth", &sys_config.soft_watchdog.check_depth, ParamType::BOOL},
    {"chassis.planner_enabled", &sys_config.chassis.planner_enabled, ParamType::BOOL},
    {"chassis.x.pos_kp", &sys_config.chassis.x.pos_kp, ParamType::FLOAT},
    {"chassis.x.pos_ki", &sys_config.chassis.x.pos_ki, ParamType::FLOAT},
    {"chassis.x.pos_kd", &sys_config.chassis.x.pos_kd, ParamType::FLOAT},
    {"chassis.x.pos_i_limit", &sys_config.chassis.x.pos_i_limit, ParamType::FLOAT},
    {"chassis.x.pos_output_limit", &sys_config.chassis.x.pos_output_limit, ParamType::FLOAT},
    {"chassis.x.vel_kp", &sys_config.chassis.x.vel_kp, ParamType::FLOAT},
    {"chassis.x.vel_ki", &sys_config.chassis.x.vel_ki, ParamType::FLOAT},
    {"chassis.x.vel_kd", &sys_config.chassis.x.vel_kd, ParamType::FLOAT},
    {"chassis.x.vel_i_limit", &sys_config.chassis.x.vel_i_limit, ParamType::FLOAT},
    {"chassis.x.vel_output_limit", &sys_config.chassis.x.vel_output_limit, ParamType::FLOAT},
    {"chassis.x.max_v", &sys_config.chassis.x.max_v, ParamType::FLOAT},
    {"chassis.x.max_a", &sys_config.chassis.x.max_a, ParamType::FLOAT},
    {"chassis.x.mass", &sys_config.chassis.x.mass, ParamType::FLOAT},
    {"chassis.x.drag", &sys_config.chassis.x.drag, ParamType::FLOAT},
    {"chassis.y.pos_kp", &sys_config.chassis.y.pos_kp, ParamType::FLOAT},
    {"chassis.y.pos_ki", &sys_config.chassis.y.pos_ki, ParamType::FLOAT},
    {"chassis.y.pos_kd", &sys_config.chassis.y.pos_kd, ParamType::FLOAT},
    {"chassis.y.pos_i_limit", &sys_config.chassis.y.pos_i_limit, ParamType::FLOAT},
    {"chassis.y.pos_output_limit", &sys_config.chassis.y.pos_output_limit, ParamType::FLOAT},
    {"chassis.y.vel_kp", &sys_config.chassis.y.vel_kp, ParamType::FLOAT},
    {"chassis.y.vel_ki", &sys_config.chassis.y.vel_ki, ParamType::FLOAT},
    {"chassis.y.vel_kd", &sys_config.chassis.y.vel_kd, ParamType::FLOAT},
    {"chassis.y.vel_i_limit", &sys_config.chassis.y.vel_i_limit, ParamType::FLOAT},
    {"chassis.y.vel_output_limit", &sys_config.chassis.y.vel_output_limit, ParamType::FLOAT},
    {"chassis.y.max_v", &sys_config.chassis.y.max_v, ParamType::FLOAT},
    {"chassis.y.max_a", &sys_config.chassis.y.max_a, ParamType::FLOAT},
    {"chassis.y.mass", &sys_config.chassis.y.mass, ParamType::FLOAT},
    {"chassis.y.drag", &sys_config.chassis.y.drag, ParamType::FLOAT},
    {"chassis.z.pos_kp", &sys_config.chassis.z.pos_kp, ParamType::FLOAT},
    {"chassis.z.pos_ki", &sys_config.chassis.z.pos_ki, ParamType::FLOAT},
    {"chassis.z.pos_kd", &sys_config.chassis.z.pos_kd, ParamType::FLOAT},
    {"chassis.z.pos_i_limit", &sys_config.chassis.z.pos_i_limit, ParamType::FLOAT},
    {"chassis.z.pos_output_limit", &sys_config.chassis.z.pos_output_limit, ParamType::FLOAT},
    {"chassis.z.vel_kp", &sys_config.chassis.z.vel_kp, ParamType::FLOAT},
    {"chassis.z.vel_ki", &sys_config.chassis.z.vel_ki, ParamType::FLOAT},
    {"chassis.z.vel_kd", &sys_config.chassis.z.vel_kd, ParamType::FLOAT},
    {"chassis.z.vel_i_limit", &sys_config.chassis.z.vel_i_limit, ParamType::FLOAT},
    {"chassis.z.vel_output_limit", &sys_config.chassis.z.vel_output_limit, ParamType::FLOAT},
    {"chassis.z.max_v", &sys_config.chassis.z.max_v, ParamType::FLOAT},
    {"chassis.z.max_a", &sys_config.chassis.z.max_a, ParamType::FLOAT},
    {"chassis.z.mass", &sys_config.chassis.z.mass, ParamType::FLOAT},
    {"chassis.z.drag", &sys_config.chassis.z.drag, ParamType::FLOAT},
    {"chassis.yaw.pos_kp", &sys_config.chassis.yaw.pos_kp, ParamType::FLOAT},
    {"chassis.yaw.pos_ki", &sys_config.chassis.yaw.pos_ki, ParamType::FLOAT},
    {"chassis.yaw.pos_kd", &sys_config.chassis.yaw.pos_kd, ParamType::FLOAT},
    {"chassis.yaw.pos_i_limit", &sys_config.chassis.yaw.pos_i_limit, ParamType::FLOAT},
    {"chassis.yaw.pos_output_limit", &sys_config.chassis.yaw.pos_output_limit, ParamType::FLOAT},
    {"chassis.yaw.vel_kp", &sys_config.chassis.yaw.vel_kp, ParamType::FLOAT},
    {"chassis.yaw.vel_ki", &sys_config.chassis.yaw.vel_ki, ParamType::FLOAT},
    {"chassis.yaw.vel_kd", &sys_config.chassis.yaw.vel_kd, ParamType::FLOAT},
    {"chassis.yaw.vel_i_limit", &sys_config.chassis.yaw.vel_i_limit, ParamType::FLOAT},
    {"chassis.yaw.vel_output_limit", &sys_config.chassis.yaw.vel_output_limit, ParamType::FLOAT},
    {"chassis.yaw.max_v", &sys_config.chassis.yaw.max_v, ParamType::FLOAT},
    {"chassis.yaw.max_a", &sys_config.chassis.yaw.max_a, ParamType::FLOAT},
    {"chassis.yaw.mass", &sys_config.chassis.yaw.mass, ParamType::FLOAT},
    {"chassis.yaw.drag", &sys_config.chassis.yaw.drag, ParamType::FLOAT},
    {"ins.init_lat", &sys_config.ins.init_lat, ParamType::FLOAT},
    {"ins.init_lon", &sys_config.ins.init_lon, ParamType::FLOAT},
    {"simulation.hitl_enabled", &sys_config.simulation.hitl_enabled, ParamType::BOOL},
    {"simulation.mass", &sys_config.simulation.mass, ParamType::FLOAT},
    {"simulation.drag", &sys_config.simulation.drag, ParamType::FLOAT},
    {"simulation.thrust_k", &sys_config.simulation.thrust_k, ParamType::FLOAT},
    {NULL, NULL, ParamType::FLOAT}
};

const size_t SYSTEM_PARAMS_COUNT = 68;

} // namespace config
} // namespace auv
