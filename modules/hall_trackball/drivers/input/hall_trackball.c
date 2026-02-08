#include "hall_trackball.h"

#include <errno.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(hall_trackball, LOG_LEVEL_INF);

/* Index = (prev_state << 2) | curr_state; each state is 2-bit */
static const int8_t quadrature_lut[16] = {
     0,  1,  0, -1,
    -1,  0,  1,  0,
     1,  0,  0,  0,
     0, -1,  1,  0
};

static inline uint8_t read_pair(const struct gpio_dt_spec *a,
                                const struct gpio_dt_spec *b) {
    const int av = gpio_pin_get_dt(a) ? 1 : 0;
    const int bv = gpio_pin_get_dt(b) ? 1 : 0;
    return (uint8_t)((av << 1) | bv);
}

static void hall_emit_work(struct k_work *work) {
    struct hall_trackball_data *data =
        CONTAINER_OF(work, struct hall_trackball_data, work);
    const struct device *dev = data->cb_xplus.dev;
    const struct hall_trackball_config *cfg = dev->config;

    if (!atomic_get(&data->enabled)) {
        return;
    }

    int16_t dx, dy;
    unsigned int key = irq_lock();
    dx = data->dx;
    dy = data->dy;
    data->dx = 0;
    data->dy = 0;
    irq_unlock(key);

    if (dx || dy) {
        /* Zephyr input: emit REL events; sync at end */
        input_report_rel(dev, INPUT_REL_X, dx * cfg->scale, false);
        input_report_rel(dev, INPUT_REL_Y, dy * cfg->scale, true);
    }
}

static inline void hall_process_motion(const struct device *dev,
                                       struct hall_trackball_data *data) {
    const struct hall_trackball_config *cfg = dev->config;

    const uint8_t x_curr = read_pair(&cfg->xplus, &cfg->xminus);
    const uint8_t y_curr = read_pair(&cfg->yplus, &cfg->yminus);

    data->dx += quadrature_lut[(data->x_state << 2) | x_curr];
    data->dy += quadrature_lut[(data->y_state << 2) | y_curr];

    data->x_state = x_curr;
    data->y_state = y_curr;

    k_work_submit(&data->work);
}

static void irq_motion(const struct device *port,
                       struct gpio_callback *cb,
                       uint32_t pins) {
    ARG_UNUSED(port);
    ARG_UNUSED(pins);

    struct hall_gpio_cb *hcb = CONTAINER_OF(cb, struct hall_gpio_cb, cb);
    const struct device *dev = hcb->dev;
    struct hall_trackball_data *data = dev->data;

    if (!atomic_get(&data->enabled)) {
        return;
    }

    hall_process_motion(dev, data);
}

static void irq_btn(const struct device *port,
                    struct gpio_callback *cb,
                    uint32_t pins) {
    ARG_UNUSED(port);
    ARG_UNUSED(pins);

    struct hall_gpio_cb *hcb = CONTAINER_OF(cb, struct hall_gpio_cb, cb);
    const struct device *dev = hcb->dev;
    struct hall_trackball_data *data = dev->data;
    const struct hall_trackball_config *cfg = dev->config;

    if (!atomic_get(&data->enabled) || !data->btn_present) {
        return;
    }

    /* pull-up: pressed when pin reads 0 */
    const bool pressed = (gpio_pin_get_dt(&cfg->btn) == 0);

    if (pressed != data->btn_last_pressed) {
        data->btn_last_pressed = pressed;
        input_report_key(dev, INPUT_BTN_LEFT, pressed ? 1 : 0, true);
    }
}

static int hall_trackball_init(const struct device *dev) {
    const struct hall_trackball_config *cfg = dev->config;
    struct hall_trackball_data *data = dev->data;

    if (!device_is_ready(cfg->xplus.port) || !device_is_ready(cfg->xminus.port) ||
        !device_is_ready(cfg->yplus.port) || !device_is_ready(cfg->yminus.port)) {
        return -ENODEV;
    }

    data->btn_present = (cfg->btn.port != NULL);

    /* Configure Hall pins */
    gpio_pin_configure_dt(&cfg->xplus,  GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure_dt(&cfg->xminus, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure_dt(&cfg->yplus,  GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_configure_dt(&cfg->yminus, GPIO_INPUT | GPIO_PULL_UP);

    gpio_pin_interrupt_configure_dt(&cfg->xplus,  GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&cfg->xminus, GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&cfg->yplus,  GPIO_INT_EDGE_BOTH);
    gpio_pin_interrupt_configure_dt(&cfg->yminus, GPIO_INT_EDGE_BOTH);

    /* Work item to emit input events outside ISR */
    k_work_init(&data->work, hall_emit_work);

    /* Initial quadrature state */
    data->x_state = read_pair(&cfg->xplus, &cfg->xminus);
    data->y_state = read_pair(&cfg->yplus, &cfg->yminus);

    /* Hook callbacks (each callback struct carries dev pointer) */
    data->cb_xplus.dev  = dev;
    data->cb_xminus.dev = dev;
    data->cb_yplus.dev  = dev;
    data->cb_yminus.dev = dev;
    data->cb_btn.dev    = dev;

    gpio_init_callback(&data->cb_xplus.cb,  irq_motion, BIT(cfg->xplus.pin));
    gpio_init_callback(&data->cb_xminus.cb, irq_motion, BIT(cfg->xminus.pin));
    gpio_init_callback(&data->cb_yplus.cb,  irq_motion, BIT(cfg->yplus.pin));
    gpio_init_callback(&data->cb_yminus.cb, irq_motion, BIT(cfg->yminus.pin));

    gpio_add_callback(cfg->xplus.port,  &data->cb_xplus.cb);
    gpio_add_callback(cfg->xminus.port, &data->cb_xminus.cb);
    gpio_add_callback(cfg->yplus.port,  &data->cb_yplus.cb);
    gpio_add_callback(cfg->yminus.port, &data->cb_yminus.cb);

    if (data->btn_present) {
        if (!device_is_ready(cfg->btn.port)) {
            return -ENODEV;
        }

        gpio_pin_configure_dt(&cfg->btn, GPIO_INPUT | GPIO_PULL_UP);
        gpio_pin_interrupt_configure_dt(&cfg->btn, GPIO_INT_EDGE_BOTH);

        gpio_init_callback(&data->cb_btn.cb, irq_btn, BIT(cfg->btn.pin));
        gpio_add_callback(cfg->btn.port, &data->cb_btn.cb);

        data->btn_last_pressed = false;
    }

    atomic_set(&data->enabled, 1);
    return 0;
}

/* Devicetree instantiation */
#define HALL_INST(n)                                                                    \
    static struct hall_trackball_data hall_data_##n;                                    \
    static const struct hall_trackball_config hall_cfg_##n = {                          \
        .xplus  = GPIO_DT_SPEC_INST_GET(n, xplus_gpios),                                \
        .xminus = GPIO_DT_SPEC_INST_GET(n, xminus_gpios),                               \
        .yplus  = GPIO_DT_SPEC_INST_GET(n, yplus_gpios),                                \
        .yminus = GPIO_DT_SPEC_INST_GET(n, yminus_gpios),                               \
        .btn    = GPIO_DT_SPEC_INST_GET_OR(n, btn_gpios, {0}),                          \
        .scale  = DT_INST_PROP(n, scale),                                               \
    };                                                                                  \
    DEVICE_DT_INST_DEFINE(n, hall_trackball_init, NULL,                                 \
                          &hall_data_##n, &hall_cfg_##n,                                \
                          POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);

DT_INST_FOREACH_STATUS_OKAY(HALL_INST)
