#define DT_DRV_COMPAT zmk_input_bb_trackball

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bb_trackball, LOG_LEVEL_INF);

struct bb_trackball_config {
    struct gpio_dt_spec up_gpio;
    struct gpio_dt_spec down_gpio;
    struct gpio_dt_spec left_gpio;
    struct gpio_dt_spec right_gpio;
};

struct bb_trackball_data {
    struct gpio_callback up_cb;
    struct gpio_callback down_cb;
    struct gpio_callback left_cb;
    struct gpio_callback right_cb;
    const struct device *dev;
};

static void bb_on_up(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct bb_trackball_data *data = CONTAINER_OF(cb, struct bb_trackball_data, up_cb);
    LOG_INF("UP pulse pins=0x%08x", pins);
    input_report_rel(data->dev, INPUT_REL_Y, -1, true, K_NO_WAIT);
}

static void bb_on_down(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct bb_trackball_data *data = CONTAINER_OF(cb, struct bb_trackball_data, down_cb);
    LOG_INF("DOWN pulse pins=0x%08x", pins);
    input_report_rel(data->dev, INPUT_REL_Y, 1, true, K_NO_WAIT);
}

static void bb_on_left(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct bb_trackball_data *data = CONTAINER_OF(cb, struct bb_trackball_data, left_cb);
    LOG_INF("LEFT pulse pins=0x%08x", pins);
    input_report_rel(data->dev, INPUT_REL_X, -1, true, K_NO_WAIT);
}

static void bb_on_right(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    struct bb_trackball_data *data = CONTAINER_OF(cb, struct bb_trackball_data, right_cb);
    LOG_INF("RIGHT pulse pins=0x%08x", pins);
    input_report_rel(data->dev, INPUT_REL_X, 1, true, K_NO_WAIT);
}

static int bb_configure_pin(const char *name, const struct gpio_dt_spec *gpio,
                            struct gpio_callback *cb, gpio_callback_handler_t handler) {
    int ret;

    if (!gpio_is_ready_dt(gpio)) {
        LOG_ERR("%s gpio not ready (port=%p pin=%d)", name, gpio->port, gpio->pin);
        return -ENODEV;
    }

    LOG_INF("%s: port=%p pin=%d dt_flags=0x%x", name, gpio->port, gpio->pin, gpio->dt_flags);

    /* This uses dt_flags (ACTIVE_LOW, PULL_UP, etc.) plus the extra flags you pass. */
    ret = gpio_pin_configure_dt(gpio, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("%s gpio_pin_configure_dt failed: %d", name, ret);
        return ret;
    }

    /*
     * Use physical falling edge for active-low pulses.
     * Note: gpio_pin_interrupt_configure_dt() does NOT use dt_flags. [web:17]
     */
    ret = gpio_pin_interrupt_configure_dt(gpio, GPIO_INT_EDGE_FALLING);
    if (ret < 0) {
        LOG_ERR("%s gpio_pin_interrupt_configure_dt failed: %d", name, ret);
        return ret;
    }

    gpio_init_callback(cb, handler, BIT(gpio->pin));
    ret = gpio_add_callback(gpio->port, cb);
    if (ret < 0) {
        LOG_ERR("%s gpio_add_callback failed: %d", name, ret);
        return ret;
    }

    return 0;
}

static int bb_trackball_init(const struct device *dev) {
    const struct bb_trackball_config *cfg = dev->config;
    struct bb_trackball_data *data = dev->data;

    LOG_INF("bb_trackball_init");
    data->dev = dev;

    int ret;

    ret = bb_configure_pin("UP", &cfg->up_gpio, &data->up_cb, bb_on_up);
    if (ret < 0) return ret;

    ret = bb_configure_pin("DOWN", &cfg->down_gpio, &data->down_cb, bb_on_down);
    if (ret < 0) return ret;

    ret = bb_configure_pin("LEFT", &cfg->left_gpio, &data->left_cb, bb_on_left);
    if (ret < 0) return ret;

    ret = bb_configure_pin("RIGHT", &cfg->right_gpio, &data->right_cb, bb_on_right);
    if (ret < 0) return ret;

    LOG_INF("bb_trackball ready");
    return 0;
}

#define BB_TRACKBALL_DEFINE(inst)                                             \
    static const struct bb_trackball_config bb_cfg_##inst = {                 \
        .up_gpio    = GPIO_DT_SPEC_INST_GET(inst, up_gpios),                  \
        .down_gpio  = GPIO_DT_SPEC_INST_GET(inst, down_gpios),                \
        .left_gpio  = GPIO_DT_SPEC_INST_GET(inst, left_gpios),                \
        .right_gpio = GPIO_DT_SPEC_INST_GET(inst, right_gpios),               \
    };                                                                        \
    static struct bb_trackball_data bb_data_##inst;                           \
    DEVICE_DT_INST_DEFINE(inst, bb_trackball_init, NULL,                      \
                          &bb_data_##inst, &bb_cfg_##inst,                    \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(BB_TRACKBALL_DEFINE)

