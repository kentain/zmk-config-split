#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

struct hall_trackball_config {
    struct gpio_dt_spec xplus;
    struct gpio_dt_spec xminus;
    struct gpio_dt_spec yplus;
    struct gpio_dt_spec yminus;
    struct gpio_dt_spec btn;   /* optional; port==NULL means absent */
    int32_t scale;
};

struct hall_gpio_cb {
    struct gpio_callback cb;
    const struct device *dev;
};

struct hall_trackball_data {
    struct hall_gpio_cb cb_xplus;
    struct hall_gpio_cb cb_xminus;
    struct hall_gpio_cb cb_yplus;
    struct hall_gpio_cb cb_yminus;
    struct hall_gpio_cb cb_btn;

    struct k_work work;

    int16_t dx;
    int16_t dy;

    uint8_t x_state;
    uint8_t y_state;

    bool btn_present;
    bool btn_last_pressed;

    atomic_t enabled;
};
