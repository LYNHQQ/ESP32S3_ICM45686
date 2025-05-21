/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "spi.h"

#ifdef IMU_INT_PINNUM   
// 中断事件队列句柄
static QueueHandle_t imu_queue = NULL;
static void IRAM_ATTR IMU_IRQ_handler(void* arg);
void IRAM_ATTR IMU_IRQ_process(void *pvParameters);
#endif

float accel_mg[3] = {0};
float gyro_dps[3] = {0};
float temp_degc = 0;

void app_main(void)
{
    imu_queue = xQueueCreate(32, sizeof(bool));
    // 创建数据处理任务
    xTaskCreatePinnedToCore(IMU_IRQ_process, "imu", 4096, NULL, 15, NULL, 1);

    if(!setup_imu(1,1,1,IMU_IRQ_handler))
        printf("IMU setup done\n");
    else
        printf("IMU setup failed\n");

    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        // bsp_IcmGetRawData(accel_mg, gyro_dps,&temp_degc);
        printf("accel_mg: %f %f %f\n", accel_mg[0], accel_mg[1], accel_mg[2]);
        printf("gyro_dps: %f %f %f\n", gyro_dps[0], gyro_dps[1], gyro_dps[2]);
        printf("temp_degc: %f\n", temp_degc);
    }
}

#ifdef IMU_INT_PINNUM
static void IMU_IRQ_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE, xResult;
    bool ready = true;
    xResult = xQueueSendFromISR(imu_queue, &ready, &xHigherPriorityTaskWoken);
    // 如果有高优先级任务等待此事件,则进行任务切换
    if (xResult == pdPASS) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    };
}
void IMU_IRQ_process(void *pvParameters)
{
    bool res;
    while (1)
    {
        // 等待中断事件
        xQueueReceive(imu_queue, &res, portMAX_DELAY);
        if (res)
        {
            /*获取ICM45686*/
            bsp_IcmGetRawData(accel_mg, gyro_dps,&temp_degc);
        }
    }
}
#endif