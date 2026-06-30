#include "ms5837.h"
#include <stdio.h>

/**
 * @brief 解析接收到的字符串数据
 * @param data_string: 接收到的字符串，格式: "Depth=0.89m Temp=37.88"
 * @param sensor_data: 解析后的数据结构体
 * @return 1=解析成功, 0=解析失败
 */
uint8_t DepthSensor_ParseData(char* data_string, depth_sensor_data_t* sensor_data)
{
    // 检查字符串长度和基本有效性
    if (data_string == NULL || strlen(data_string) < 10) {
        return 0;
    }

    // 查找深度数据 "Depth:"
    char* depth_ptr = strstr(data_string, "Depth:");
    if (depth_ptr == NULL) {
        return 0;  // 未找到深度数据
    }

    // 查找温度数据 "Temp:"
    char* temp_ptr = strstr(data_string, "Temp:");
    if (temp_ptr == NULL) {
        return 0;  // 未找到温度数据
    }

    // 解析深度值
    depth_ptr += 6;  // 跳过 "Depth:"
    float depth_val = atof(depth_ptr);

    // 解析温度值
    temp_ptr += 5;   // 跳过 "Temp:"
    float temp_val = atof(temp_ptr);

    // 简单的数值有效性检查
    if (temp_val < -50.0f || temp_val > 100.0f) {
        return 0;  // 温度范围不合理
    }

    if (depth_val < -100.0f || depth_val > 1000.0f) {
        return 0;  // 深度范围不合理（允许负值，因为可能在水面以上）
    }

    // 数据有效，保存结果
    sensor_data->temperature = temp_val;
    sensor_data->depth = depth_val;
    sensor_data->data_ready = 1;

    return 1;  // 解析成功
}

/**
 * @brief 发送液体密度设置命令
 * @param huart: UART句柄
 * @param density: 液体密度 (淡水997, 海水1029)
 * @return 1=成功, 0=失败
 */
uint8_t DepthSensor_SetDensity(uint16_t density, depth_sensor_data_t* sensor_data)
{
    char command[20];
    sprintf(command, "!F%04d\r\n", density);
    if (sensor_data->ms5837_send != NULL) {
        sensor_data->ms5837_send(command);
        return 1;
    }
    /*if (HAL_UART_Transmit(huart, (uint8_t*)command, strlen(command), 1000) == HAL_OK) {
        return 1;
    }*/
    return 0;
}

/**
 * @brief 发送LED控制命令
 * @param huart: UART句柄
 * @param led_mode: 0=关闭, 1=常亮, 2=闪烁
 * @return 1=成功, 0=失败
 */
uint8_t DepthSensor_SetLED(uint8_t led_mode, depth_sensor_data_t* sensor_data)
{
    char command[8];
    sprintf(command, "!L%d\r\n", led_mode);
    if (sensor_data->ms5837_send != NULL) {
        sensor_data->ms5837_send(command);
        return 1;
    }
    /*if (HAL_UART_Transmit(huart, (uint8_t*)command, strlen(command), 1000) == HAL_OK) {
        return 1;
    }*/
    return 0;
}

/**
 * @brief 发送复位命令
 * @param huart: UART句柄
 * @return 1=成功, 0=失败
 */
uint8_t DepthSensor_Reset(depth_sensor_data_t* sensor_data)
{
    char command[] = "!R\r\n";
    if (sensor_data->ms5837_send != NULL) {
        sensor_data->ms5837_send(command);
        return 1;
    }
    /*if (HAL_UART_Transmit(huart, (uint8_t*)command, strlen(command), 1000) == HAL_OK) {
        return 1;
    }*/
    return 0;
}


/**
 * @brief 发送深度偏移设置命令
 * @param huart: UART句柄
 * @param offset: 深度偏移值 (浮点数)
 * @return 1=成功, 0=失败
 */
uint8_t DepthSensor_SetDepthOffset(float offset, depth_sensor_data_t* sensor_data)
{
    char command[20];
    sprintf(command, "!D%+06.2f\r\n", offset);
    if (sensor_data->ms5837_send != NULL) {
        sensor_data->ms5837_send(command);
        return 1;
    }
    /*if (HAL_UART_Transmit(huart, (uint8_t*)command, strlen(command), 1000) == HAL_OK) {
        return 1;
    }*/
    return 0;
}

/**
 * @brief 发送温度偏移设置命令
 * @param huart: UART句柄
 * @param offset: 温度偏移值 (浮点数)
 * @return 1=成功, 0=失败
 */
uint8_t DepthSensor_SetTempOffset(float offset, depth_sensor_data_t* sensor_data)
{
    char command[20];
    sprintf(command, "!T%+06.2f\r\n", offset);
    if (sensor_data->ms5837_send != NULL) {
        sensor_data->ms5837_send(command);
        return 1;
    }
    /*if (HAL_UART_Transmit(huart, (uint8_t*)command, strlen(command), 1000) == HAL_OK) {
        return 1;
    }*/
    return 0;
}

/**
 * @brief 发送查询参数命令
 * @param huart: UART句柄
 * @return 1=成功, 0=失败
 */
uint8_t DepthSensor_QueryParams(depth_sensor_data_t* sensor_data)
{
    char command[] = "!!\r";
    if (sensor_data->ms5837_send != NULL) {
        sensor_data->ms5837_send(command);
        return 1;
    }
    /*if (HAL_UART_Transmit(huart, (uint8_t*)command, strlen(command), 1000) == HAL_OK) {
        return 1;
    }*/
    return 0;
}
/**
 * @brief 发送深度传感器数据到Jetson (标准格式)
 * @param jetson_uart: 发送给Jetson的UART句柄
 * @param sensor_data: 深度传感器数据结构体
 * @return 1=成功, 0=失败
 */
uint8_t DepthSensor_SendDataToJetson(depth_sensor_data_t* sensor_data)
{
    if (sensor_data == NULL || !sensor_data->data_ready) {
        return 0;  // 数据无效或未就绪
    }

    // 构造发送字符串（与Jetson程序期望的新格式匹配）
    char send_buffer[100];
    int len = sprintf(send_buffer, "DEPTH:%.3f,%.2f\n",
                     sensor_data->depth, sensor_data->temperature);
    if (sensor_data->jetson_send != NULL) {
        sensor_data->jetson_send(send_buffer);
        return 1;
    }
    // 发送数据到Jetson
    /*if (HAL_UART_Transmit(jetson_uart, (uint8_t*)send_buffer, len, 1000) == HAL_OK) {
        return 1;
    }*/
    return 0;
}

/**
 * @brief 发送深度传感器数据到Jetson (新格式: DEPTH:深度,温度)
 * @param jetson_uart: 发送给Jetson的UART句柄
 * @param depth: 深度值 (m)
 * @param temperature: 温度值 (°C)
 * @return 1=成功, 0=失败
 */
uint8_t DepthSensor_SendNewFormatToJetson(float depth, float temperature, depth_sensor_data_t* sensor_data)
{
    char send_buffer[100];
    int len = sprintf(send_buffer, "DEPTH:%.3f,%.2f\n", depth, temperature);
    if (sensor_data->jetson_send != NULL) {
        sensor_data->jetson_send(send_buffer);
        return 1;
    }
    /*if (HAL_UART_Transmit(jetson_uart, (uint8_t*)send_buffer, len, 1000) == HAL_OK) {
        return 1;
    }*/
    return 0;
}

/**
 * @brief 发送深度传感器数据到Jetson (原有格式: Depth=XXXm Temp=XXX)
 * @param jetson_uart: 发送给Jetson的UART句柄
 * @param depth: 深度值 (m)
 * @param temperature: 温度值 (°C)
 * @return 1=成功, 0=失败
 */
uint8_t DepthSensor_SendOriginalFormatToJetson(float depth, float temperature, depth_sensor_data_t* sensor_data)
{
    char send_buffer[100];
    int len = sprintf(send_buffer, "Depth=%.2fm Temp=%.2f\n", depth, temperature);
    if (sensor_data->jetson_send != NULL) {
        sensor_data->jetson_send(send_buffer);
        return 1;
    }
    /*if (HAL_UART_Transmit(jetson_uart, (uint8_t*)send_buffer, len, 1000) == HAL_OK) {
        return 1;
    }*/
    return 0;
}

/**
 * @brief 处理从MS5837传感器接收的数据并转发给Jetson
 * @param sensor_uart: 连接MS5837传感器的UART句柄
 * @param jetson_uart: 发送给Jetson的UART句柄
 * @param received_data: 从传感器接收到的原始数据字符串
 * @return 1=成功, 0=失败
 */
uint8_t DepthSensor_ProcessAndSendToJetson(char* received_data, depth_sensor_data_t* sensor_data)
{
    // 解析接收到的数据
    if (DepthSensor_ParseData(received_data, sensor_data)) {
        // 数据解析成功，发送给Jetson（使用新格式）
        return DepthSensor_SendDataToJetson(sensor_data);
    } else {
        // 解析失败，发送错误信息到Jetson
        char error_msg[] = "DEPTH:ERROR,ERROR\n";
        if (sensor_data->jetson_send != NULL) {
            sensor_data->jetson_send(error_msg);
            return 1;
        }
        /*if (HAL_UART_Transmit(jetson_uart, (uint8_t*)error_msg, strlen(error_msg), 1000) == HAL_OK) {
            return 1;
        }*/
    }

    return 0;
}

/**
 * @brief 发送深度传感器状态信息到Jetson
 * @param jetson_uart: 发送给Jetson的UART句柄
 * @param status: 传感器状态 (0=离线, 1=在线, 2=错误)
 * @return 1=成功, 0=失败
 */
uint8_t DepthSensor_SendStatusToJetson(uint8_t status, depth_sensor_data_t* sensor_data)
{
    char send_buffer[50];
    int len;

    switch (status) {
        case 0:
            len = sprintf(send_buffer, "DEPTH_STATUS:OFFLINE\n");
            break;
        case 1:
            len = sprintf(send_buffer, "DEPTH_STATUS:ONLINE\n");
            break;
        case 2:
            len = sprintf(send_buffer, "DEPTH_STATUS:ERROR\n");
            break;
        default:
            len = sprintf(send_buffer, "DEPTH_STATUS:UNKNOWN\n");
            break;
    }
    if (sensor_data->jetson_send != NULL) {
            sensor_data->jetson_send(send_buffer);
            return 1;
        }
    /*if (HAL_UART_Transmit(jetson_uart, (uint8_t*)send_buffer, len, 1000) == HAL_OK) {
        return 1;
    }*/
    return 0;
}
