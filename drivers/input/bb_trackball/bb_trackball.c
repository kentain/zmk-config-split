#define DT_DRV_COMPAT zmk_input_bb_trackball

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>

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

// Interrupt Handlers
static void bb_on_up(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    struct bb_trackball_data *data = CONTAINER_OF(cb, struct bb_trackball_data, up_cb);
    input_report_rel(data->dev, INPUT_REL_Y, -1, true, K_NO_WAIT);
}

static void bb_on_down(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    struct bb_trackball_data *data = CONTAINER_OF(cb, struct bb_trackball_data, down_cb);
    input_report_rel(data->dev, INPUT_REL_Y, 1, true, K_NO_WAIT);
}

static void bb_on_left(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    struct bb_trackball_data *data = CONTAINER_OF(cb, struct bb_trackball_data, left_cb);
    input_report_rel(data->dev, INPUT_REL_X, -1, true, K_NO_WAIT);
}

static void bb_on_right(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    struct bb_trackball_data *data = CONTAINER_OF(cb, struct bb_trackball_data, right_cb);
    input_report_rel(data->dev, INPUT_REL_X, 1, true, K_NO_WAIT);
}

static int bb_configure_pin(const struct gpio_dt_spec *gpio, struct gpio_callback *cb, 
                            gpio_callback_handler_t handler) {
    int ret;
    if (!gpio_is_ready_dt(gpio)) return -ENODEV;

    ret = gpio_pin_configure_dt(gpio, GPIO_INPUT);
    if (ret < 0) return ret;

    ret = gpio_pin_interrupt_configure_dt(gpio, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) return ret;

    gpio_init_callback(cb, handler, BIT(gpio->pin));
    return gpio_add_callback(gpio->port, cb);
}

static int bb_trackball_init(const struct device *dev) {
    const struct bb_trackball_config *cfg = dev->config;
    struct bb_trackball_data *data = dev->data;
    data->dev = dev;

    if (bb_configure_pin(&cfg->up_gpio, &data->up_cb, bb_on_up) < 0 ||
        bb_configure_pin(&cfg->down_gpio, &data->down_cb, bb_on_down) < 0 ||
        bb_configure_pin(&cfg->left_gpio, &data->left_cb, bb_on_left) < 0 ||
        bb_configure_pin(&cfg->right_gpio, &data->right_cb, bb_on_right) < 0) {
        return -EIO;
    }
    return 0;
}

static const struct bb_trackball_config bb_cfg = {
    .up_gpio = GPIO_DT_SPEC_INST_GET(0, up_gpios),
    .down_gpio = GPIO_DT_SPEC_INST_GET(0, down_gpios),
    .left_gpio = GPIO_DT_SPEC_INST_GET(0, left_gpios),
    .right_gpio = GPIO_DT_SPEC_INST_GET(0, right_gpios),
};

static struct bb_trackball_data bb_data;

DEVICE_DT_INST_DEFINE(0, bb_trackball_init, NULL, &bb_data, &bb_cfg,
                      POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);
