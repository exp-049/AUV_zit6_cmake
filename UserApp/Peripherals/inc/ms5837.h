/*
 * ms5837.h
 *
 *  Created on: Aug 30, 2025
 *      Author: 18743
 */

#ifndef INC_MS5837_H_
#define INC_MS5837_H_

#include "main.h"
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#ifdef __cplusplus
extern "C" {
#endif

/* 深度传感器数据结构体 */
typedef struct {
  float temperature;               // 温度 (°C)
  float depth;                     // 深度 (m)
  uint8_t data_ready;              // 数据就绪标志
  void (*ms5837_send)(char *data); // 发送ms5837
  void (*jetson_send)(char *data); // 发送jetson
} depth_sensor_data_t;

/* 函数声明 */

/**
 * @brief 解析接收到的字符串数据
 * @param data_string: 接收到的字符串，格式: "T=25.27D=1.21"
 * @param sensor_data: 解析后的数据结构体
 * @return 1=解析成功, 0=解析失败
 */
uint8_t DepthSensor_ParseData(char *data_string,
                              depth_sensor_data_t *sensor_data);

/**
 * @brief 发送液体密度设置命令
 * @param huart: UART句柄
 * @param density: 液体密度 (淡水997, 海水1029)
 * @return 1=成功, 0=失败
 */
uint8_t DepthSensor_SetDensity(uint16_t density,
                               depth_sensor_data_t *sensor_data);

/**
 * @brief 发送LED控制命令
 * @param huart: UART句柄
 * @param led_mode: 0=关闭, 1=常亮, 2=闪烁
 * @return 1=成功, 0=失败
 */
uint8_t DepthSensor_SetLED(uint8_t led_mode, depth_sensor_data_t *sensor_data);

/**
 * @brief 发送复位命令
 * @param huart: UART句柄
 * @return 1=成功, 0=失败
 */
uint8_t DepthSensor_Reset(depth_sensor_data_t *sensor_data);

/**
 * @brief 发送深度偏移设置命令
 * @param huart: UART句柄
 * @param offset: 深度偏移值 (浮点数)
 * @return 1=成功, 0=失败
 */
uint8_t DepthSensor_SetDepthOffset(float offset,
                                   depth_sensor_data_t *sensor_data);

/**
 * @brief 发送温度偏移设置命令
 * @param huart: UART句柄
 * @param offset: 温度偏移值 (浮点数)
 * @return 1=成功, 0=失败
 */
uint8_t DepthSensor_SetTempOffset(float offset,
                                  depth_sensor_data_t *sensor_data);

/**
 * @brief 发送查询参数命令
 * @param huart: UART句柄
 * @return 1=成功, 0=失败
 */
uint8_t DepthSensor_QueryParams(depth_sensor_data_t *sensor_data);
/**
 * @brief 发送深度传感器数据到Jetson (标准格式)
 * @param jetson_uart: 发送给Jetson的UART句柄
 * @param sensor_data: 深度传感器数据结构体
 * @return 1=成功, 0=失败
 */
uint8_t DepthSensor_SendDataToJetson(depth_sensor_data_t *sensor_data);

/**
 * @brief 发送深度传感器数据到Jetson (新格式: DEPTH:深度,温度)
 * @param jetson_uart: 发送给Jetson的UART句柄
 * @param depth: 深度值 (m)
 * @param temperature: 温度值 (°C)
 * @return 1=成功, 0=失败
 */
uint8_t DepthSensor_SendNewFormatToJetson(float depth, float temperature,
                                          depth_sensor_data_t *sensor_data);

/**
 * @brief 发送深度传感器数据到Jetson (原有格式: Depth=XXXm Temp=XXX)
 * @param jetson_uart: 发送给Jetson的UART句柄
 * @param depth: 深度值 (m)
 * @param temperature: 温度值 (°C)
 * @return 1=成功, 0=失败
 */
uint8_t
DepthSensor_SendOriginalFormatToJetson(float depth, float temperature,
                                       depth_sensor_data_t *sensor_data);

/**
 * @brief 处理从MS5837传感器接收的数据并转发给Jetson
 * @param sensor_uart: 连接MS5837传感器的UART句柄
 * @param jetson_uart: 发送给Jetson的UART句柄
 * @param received_data: 从传感器接收到的原始数据字符串
 * @return 1=成功, 0=失败
 */
uint8_t DepthSensor_ProcessAndSendToJetson(char *received_data,
                                           depth_sensor_data_t *sensor_data);

/**
 * @brief 发送深度传感器状态信息到Jetson
 * @param jetson_uart: 发送给Jetson的UART句柄
 * @param status: 传感器状态 (0=离线, 1=在线, 2=错误)
 * @return 1=成功, 0=失败
 */
uint8_t DepthSensor_SendStatusToJetson(uint8_t status,
                                       depth_sensor_data_t *sensor_data);

#ifdef __cplusplus
}
#endif

#endif /* INC_MS5837_H_ */
