#include <stdio.h>
#include <string.h>

#include "RP2040.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"


#define TIME_OVER(target, time) ((uint32_t)((time) - (target)) < 0x80000000U) 


static int mpu6050_addr = 0x68;

static void mpu6050_wakeup() {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, mpu6050_addr, buf, 2, false);
}

static void mpu6050_config_sample_rate() {
    uint8_t buf[] = {0x19, 0x00};
    i2c_write_blocking(i2c_default, mpu6050_addr, buf, 2, false);
}

static void mpu6050_config_dlpf() {
    uint8_t buf[] = {0x1A, 0x06};
    i2c_write_blocking(i2c_default, mpu6050_addr, buf, 2, false);
}

static void mpu6050_config_gyro() {
    uint8_t buf[] = {0x1B, 0x00};
    i2c_write_blocking(i2c_default, mpu6050_addr, buf, 2, false);
}

static void mpu6050_config_accel() {
    uint8_t buf[] = {0x1C, 0x00};
    i2c_write_blocking(i2c_default, mpu6050_addr, buf, 2, false);
}

static void mpu6050_enable_interrupt() {
    uint8_t buf[] = {0x38, 0x01};
    i2c_write_blocking(i2c_default, mpu6050_addr, buf, 2, false);
}


static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    //printf("mpu6050_read_raw\n");
    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, mpu6050_addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, mpu6050_addr, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, mpu6050_addr, &val, 1, true);
    i2c_read_blocking(i2c_default, mpu6050_addr, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, mpu6050_addr, &val, 1, true);
    i2c_read_blocking(i2c_default, mpu6050_addr, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

//static inline avg(int16_t val, )

int main() {

    const uint led_pin = 25;

    // Initialize LED pin
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

     // Initialize I2C
    i2c_init(i2c_default, 400E3);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    // Reset MPU6050 to start measuremets
    mpu6050_config_sample_rate();
    mpu6050_config_dlpf();
    mpu6050_config_gyro();
    mpu6050_config_accel();
    mpu6050_enable_interrupt();
    mpu6050_wakeup();

    // Initialize chosen serial port
    stdio_init_all();

    sleep_ms(100);

    int16_t acceleration[3];
    int16_t gyro[3];
    int16_t temp;
    size_t cnt = 0;

    float facc[3];
    float fgyro[3];

    uint32_t target[8] = {0};

    printf("here\n");
    sleep_ms(1000);

    while (true) {

        // Blink LED
        if (TIME_OVER(target[0], time_us_32())) {
            target[0] = time_us_32() + 1E6;
            gpio_put(led_pin, !gpio_get(led_pin));
        }

        if (TIME_OVER(target[1], time_us_32())) {
            target[1] = time_us_32() + 1E5;
            mpu6050_read_raw(acceleration, gyro, &temp);

            //printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
            //printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
            //printf("Temp. = %f\n", (temp / 340.0) + 36.53);

            for (int i=0; i<3; i++) {
                float sens_acc = 16384; // 16384 per g
                float sens_gyro = 131;  // 131 per deg/s
                facc[i] = (float)acceleration[i]/sens_acc; // acc in m/s2
                fgyro[i] = (float)gyro[i]/sens_gyro;        // acc in deg/s2
            }
            printf("a = [%0.3f, %0.3f, %0.3f] g\n", facc[0], facc[1], facc[2]);
            printf("w = [%0.3f, %0.3f, %0.3f] deg/s\n", fgyro[0], fgyro[1], fgyro[2]);
        }
    }
}