#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <queue.h>
#include "pico/stdlib.h"

#include "icm20948.h"

static QueueHandle_t xQueue = NULL;
static QueueHandle_t gyroQueue = NULL;

void led_task(void *pvParameters)
{
    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    uint uIValueToSend = 0;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (true)
    {
        gpio_put(LED_PIN, 1);
        uIValueToSend = 1;
        xQueueSend(xQueue, &uIValueToSend, 0U);
        vTaskDelay(100);

        gpio_put(LED_PIN, 0);
        uIValueToSend = 0;
        xQueueSend(xQueue, &uIValueToSend, 0U);
        vTaskDelay(100);
    }
}

void usb_task(void *pvParameters)
{
    uint uIReceivedValue;
    float MotionVal[6];

    while (1)
    {
        if (xQueueReceive(xQueue, &uIReceivedValue, 0) == pdTRUE)
        {
            if (uIReceivedValue == 1)
            {
                printf("LED is ON! \n");
            }
            if (uIReceivedValue == 0)
            {
                printf("LED is OFF! \n");
            }
        }

        if (xQueueReceive(gyroQueue, &MotionVal[0], 0) == pdTRUE)
        {
            printf("\r\n /-------------------------------------------------------------/ \r\n");
            printf("\r\n[g0]: %.2f     [g1]: %.2f     [g2]: %.2f \r\n", MotionVal[0], MotionVal[1], MotionVal[2]);
            printf("\r\n[a0]: %.2f     [a1]: %.2f     [a2]: %.2f \r\n", MotionVal[3], MotionVal[4], MotionVal[5]);
        }
    }
}

void imu_task(void *pvParameters)
{
    IMU_EN_SENSOR_TYPE enMotionSensorType;
    IMU_ST_SENSOR_DATA stGyroRawData;
    IMU_ST_ANGLES_DATA stAngles;
    IMU_ST_SENSOR_DATA stAccelRawData;
    IMU_ST_SENSOR_DATA stMagnRawData;

    imuInit(&enMotionSensorType);

    if (IMU_EN_SENSOR_TYPE_ICM20948 == enMotionSensorType)
    {
        printf("Motion sersor is ICM-20948\n");
    }
    else
    {
        printf("Motion sersor NULL\n");
    }

    while (1)
    {
        float MotionVal[6];
        int16_t s16Gyro[3];
        int16_t s16Accel[3];

        icm20948GyroRead(&s16Gyro[0], &s16Gyro[1], &s16Gyro[2]);
        icm20948AccelRead(&s16Accel[0], &s16Accel[1], &s16Accel[2]);

        MotionVal[0] = (float)s16Gyro[0] / 131.0F;
        MotionVal[1] = (float)s16Gyro[1] / 131.0F;
        MotionVal[2] = (float)s16Gyro[2] / 131.0F;
        MotionVal[3] = (float)s16Accel[0]/16384.0F;
        MotionVal[4] = (float)s16Accel[1]/16384.0F;
        MotionVal[5] = (float)s16Accel[2]/16384.0F;

        xQueueSend(gyroQueue, &MotionVal[0], 0U);

        vTaskDelay(1);
    }
}

int main()
{
    stdio_init_all();

    while (!stdio_usb_connected())
    {
    }

    sleep_ms(1000);

    xQueue = xQueueCreate(1, sizeof(uint));
    gyroQueue = xQueueCreate(1, 6 * sizeof(float));

    xTaskCreate(led_task, "LED_Task", 256, NULL, 1, NULL);
    xTaskCreate(usb_task, "USB_Task", 256, NULL, 1, NULL);
    xTaskCreate(imu_task, "IMU_Task", 256, NULL, 2, NULL);

    vTaskStartScheduler();

    while (1)
    {
    };
}