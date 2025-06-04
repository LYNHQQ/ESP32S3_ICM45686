/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "spi.h"
#include "MahonyAHRS.h"

static QueueHandle_t imu_queue = NULL;
float accel_mg[3] = {0};
float gyro_dps[3] = {0};
float gyro_dps_offset[3] = {0};
float temp_degc = 0;
float roll, pitch, yaw;

static void IMU_IRQ_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE, xResult;
    bool ready = true;
    xResult = xQueueSendFromISR(imu_queue, &ready, &xHigherPriorityTaskWoken);
    if (xResult == pdPASS) {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    };
}

static void IMU_IRQ_process(void *pvParameters)
{
    bool res,mahony_ready = false,gyro_offset_ready = false;
    volatile uint16_t gyro_offset_cnt=0;
    imu_queue = xQueueCreate(32, sizeof(bool));
    Mahony_Init(6400.0f); //初始化Mahony算法，设置采样频率为6400Hz
    setup_imu(1,1,1,IMU_IRQ_handler);
    while (1)
    {
        xQueueReceive(imu_queue, &res, portMAX_DELAY);
        if (res)
        {
			bsp_IcmGetRawData(accel_mg, gyro_dps,&temp_degc);
            if(mahony_ready == false)
            {
                MahonyAHRSinit(accel_mg[0],accel_mg[1],accel_mg[2],0,0,0);  
                mahony_ready = true;
            }
            if(gyro_offset_ready == false && gyro_offset_cnt<=1000)
            {
                gyro_offset_cnt ++;
                gyro_dps_offset[0] +=gyro_dps[0];
                gyro_dps_offset[1] +=gyro_dps[1];
                gyro_dps_offset[2] +=gyro_dps[2];         
                if(gyro_offset_cnt == 1000)  
                {
                    gyro_dps_offset[0]/=1000;
                    gyro_dps_offset[1]/=1000;
                    gyro_dps_offset[2]/=1000;
                    gyro_offset_ready = true;
                    gyro_offset_cnt = 0;
                    printf("Gyro_Offset: [%.2f, %.2f, %.2f] dps\n",
                    gyro_dps_offset[0], gyro_dps_offset[1], gyro_dps_offset[2]);
                }
            }
            if(gyro_offset_ready == true)
            {
                gyro_dps[0] -=gyro_dps_offset[0];
                gyro_dps[1] -=gyro_dps_offset[1];
                gyro_dps[2] -=gyro_dps_offset[2];          
                Mahony_update(gyro_dps[0],gyro_dps[1],gyro_dps[2],accel_mg[0],accel_mg[1],accel_mg[2],0,0,0);
                Mahony_computeAngles(); //角度计算   移植到别的平台需要替换掉对应的arm_atan2_f32 和 arm_asin    
            }
            static int count = 0;
            if( ++count >= 100 && gyro_offset_ready == true)//64Hz output rate
            {
                count = 0;
                // printf("Accel: [%.2f, %.2f, %.2f] mg, Gyro: [%.2f, %.2f, %.2f] dps\n",
                // accel_mg[0], accel_mg[1], accel_mg[2],
                // gyro_dps[0], gyro_dps[1], gyro_dps[2]);
                printf("[roll:%.2f,pitch: %.2f,yaw: %.2f]\n",getRoll(),getPitch(),getYaw());
            }
        }
    }
}

void app_main(void)
{
    xTaskCreatePinnedToCore(IMU_IRQ_process, "imu", 4096, NULL, 15, NULL, 1);
	// uint8_t CPU_RunInfo[400];
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        // memset(CPU_RunInfo, 0, 400); /* 信息缓冲区清零 */
 
        // vTaskGetRunTimeStats((char *)&CPU_RunInfo);

        // printf("task_name       run_cnt       usage_rate   \r\n");
        // printf("%s", CPU_RunInfo);
        // printf("----------------------------------------------------\r\n");
    }
}
