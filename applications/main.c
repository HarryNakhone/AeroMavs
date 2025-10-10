/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2020 TDK Invensense
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led2" alias. */
#define LED_NODE DT_NODELABEL(green_led_2)

/* Generic motion sensor node */
#define MOTION_SENSOR_NODE DT_NODELABEL(motion_sensor)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

/* Generic motion sensor device */
static const struct device *motion_sensor = DEVICE_DT_GET(MOTION_SENSOR_NODE);

int main(void)
{
    int ret;
    bool led_state = true;
    struct sensor_value accel[3], gyro[3], temp;

    if (!gpio_is_ready_dt(&led)) {
        LOG_ERR("LED GPIO not ready");
        return 0;
    }

    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_ERR("Failed to configure LED GPIO");
        return 0;
    }

    /* Check if motion sensor is ready */
    if (!device_is_ready(motion_sensor)) {
        LOG_ERR("Motion sensor not ready");
        return 0;
    }

    LOG_INF("Generic motion sensor initialized successfully");

    while (1) {
        /* Toggle LED */
        ret = gpio_pin_toggle_dt(&led);
        if (ret < 0) {
            LOG_ERR("Failed to toggle LED");
            return 0;
        }

        led_state = !led_state;
        printf("LED state: %s\n", led_state ? "ON" : "OFF");

        /* Read sensor data */
        if (sensor_sample_fetch(motion_sensor) < 0) {
            LOG_ERR("Failed to fetch sensor sample");
        } else {
            /* Read accelerometer data */
            sensor_channel_get(motion_sensor, SENSOR_CHAN_ACCEL_XYZ, accel);
            printf("Accel: X=%.2f Y=%.2f Z=%.2f m/s^2\n",
                   sensor_value_to_double(&accel[0]),
                   sensor_value_to_double(&accel[1]),
                   sensor_value_to_double(&accel[2]));

            /* Read gyroscope data */
            sensor_channel_get(motion_sensor, SENSOR_CHAN_GYRO_XYZ, gyro);
            printf("Gyro:  X=%.2f Y=%.2f Z=%.2f rad/s\n",
                   sensor_value_to_double(&gyro[0]),
                   sensor_value_to_double(&gyro[1]),
                   sensor_value_to_double(&gyro[2]));

            /* Read temperature */
            sensor_channel_get(motion_sensor, SENSOR_CHAN_DIE_TEMP, &temp);
            printf("Temp:  %.2f C\n", sensor_value_to_double(&temp));
        }

        printf("---\n");
        k_msleep(SLEEP_TIME_MS);
    }
    return 0;
}
 