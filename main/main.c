#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "mpu6050.h"
#include "Fusion.h"

#define SAMPLE_PERIOD (0.01f) // replace this with actual sample period

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;
QueueHandle_t xQueuePos;

typedef struct coord
{
    int axis;
    int val;
} coord; 
static void mpu6050_reset()
{
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp)
{
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false);

    for (int i = 0; i < 3; i++)
    {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 6, false); // False - finished with bus

    for (int i = 0; i < 3; i++)
    {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
        ;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU_ADDRESS, buffer, 2, false); // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

void mpu6050_task(void *p)
{
    // configuracao do I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    mpu6050_reset();
    int16_t acceleration[3], gyro[3], temp;

    float fator_x = 0.2;
    float fator_y = 0.2;

    while (1)
    {
        // leitura da MPU, sem fusao de dados
        mpu6050_read_raw(acceleration, gyro, &temp);
        // printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
        // printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
        // printf("Temp. = %f\n", (temp / 340.0) + 36.53);

        FusionVector gyroscope = {
            .axis.x = gyro[0] / 131.0f, // Conversão para graus/s
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };

        FusionVector accelerometer = {
            .axis.x = acceleration[0] / 16384.0f, // Conversão para g
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        // printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw\n);

        coord x;
        x.axis = 0;
        x.val = (int) euler.angle.roll *  fator_x;
        printf("X value: %d\n", x.val);

        coord y;
        y.axis =1;
        y.val = (int) euler.angle.pitch * fator_y;
        printf("Y value: %d\n", y.val);

        xQueueSend(xQueuePos, &x, 0);
        xQueueSend(xQueuePos, &y, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void uart_task(void *p)
{
    coord recebido;
    uart_init(uart0, 115200);

    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);

    while (1)
    {
        if (xQueueReceive(xQueuePos, &recebido, portMAX_DELAY))
        {
            printf("Sending axis %d val %d\n", recebido.axis, recebido.val);
            uint8_t vec[4];
            vec[0] = 0xFF; // byte de sincronização QUE O PYTHON TA ESPERANDO rpa começar a ler os dados
            vec[1] = (uint8_t)recebido.axis;
            vec[2] = (uint8_t)(recebido.val & 0xFF);
            vec[3] = (uint8_t)((recebido.val >> 8) & 0xFF);
            uart_write_blocking(uart0, vec, 4); // 4.1.29.7.26. uart_write_blocking do manual da PICO
        }
    }
}

int main()
{
    stdio_init_all();
    
    xQueuePos = xQueueCreate(32, sizeof(coord));
    xTaskCreate(mpu6050_task, "mpu6050_Task 1", 8192, NULL, 1, NULL);
    xTaskCreate(uart_task, "uart task", 8192, NULL, 1, NULL);
    vTaskStartScheduler();

    while (true)
        ;
}
