/**  
* Blackberry Hall Effect Trackball Driver for ZMK (with Click Button)  
* 4x LUT quadrature + left-click GPIO support  
* 9 pulses/360° → 36 counts/rotation → 0.28mm linear precision  
*/  
  
#include <zephyr/kernel.h>  
#include <zephyr/drivers/gpio.h>  
#include <zephyr/device.h>  
#include <zmk/hid.h>  
#include <zmk/event_manager.h>  
#include <dt-bindings/gpio.h>  
#include <logging/log.h>  
  
LOG_MODULE_REGISTER(hall_trackball, LOG_LEVEL_DBG);  
  
/* PERFECT 4X RESOLUTION QUADRATURE LUT */  
/* Index = (prev_state << 2) | curr_state */  
/* +1 = Right/Up, -1 = Left/Down, 0 = invalid/no motion */  
static const int8_t quadrature_lut[16] = {  
0, 1, 0, -1, // OLD00 → NEWxx (00=0, 01=1, 11=3, 10=2)  
-1, 0, 1, 0, // OLD01 → NEWxx  
1, 0, 0, 0, // OLD11 → NEWxx   
0, -1, 1, 0 // OLD10 → NEWxx  
};  
  
/* Driver private data */  
struct hall_trackball_data {  
struct gpio_callback gpio_cb[5]; // 4 Hall + 1 Button  
int16_t delta_x, delta_y; // Accumulated motion  
uint8_t x_state, y_state; // Previous quadrature states (0-3)  
bool btn_last_state; // Button debounce state  
int64_t last_report; // 200Hz rate limiting  
const struct gpio_dt_spec *xplus; // GPIO DT specs  
const struct gpio_dt_spec *xminus;  
const struct gpio_dt_spec *yplus;  
const struct gpio_dt_spec *yminus;  
const struct gpio_dt_spec *btn; // CLICK BUTTON GPIO  
atomic_t enabled;  
};  
  
/* SHARED IRQ HANDLER - 6 CPU CYCLES! */  
static void hall_irq_handler(const struct device *dev, void *user_data) {  
struct hall_trackball_data *data = (struct hall_trackball_data *)user_data;  
  
if (!atomic_get(&data->enabled)) return;  
  
/* QUADRATURE DECODING (runs on Hall edges) */  
bool xp = gpio_pin_get_dt(data->xplus);  
bool xm = gpio_pin_get_dt(data->xminus);  
bool yp = gpio_pin_get_dt(data->yplus);  
bool ym = gpio_pin_get_dt(data->yminus);  
  
uint8_t x_curr = (xp << 1) | xm;  
uint8_t y_curr = (yp << 1) | ym;  
  
/* LUT: 1 lookup = direction + distance */  
data->delta_x += quadrature_lut[(data->x_state << 2) | x_curr];  
data->delta_y += quadrature_lut[(data->y_state << 2) | y_curr];  
  
data->x_state = x_curr;  
data->y_state = y_curr;  
  
/* BUTTON CLICK HANDLING (runs on button edge) */  
bool btn_curr = gpio_pin_get_dt(data->btn);  
if (btn_curr != data->btn_last_state) {  
/* 20ms DEBOUNCE */  
k_sleep(K_MSEC(20));  
btn_curr = gpio_pin_get_dt(data->btn);  
  
if (!btn_curr && data->btn_last_state) { // Falling edge = press  
/* LEFT CLICK (HID button 1) */  
zmk_hid_send_mouse_report(0, 0, 0, 1); // Press  
zmk_hid_send_mouse_report(0, 0, 0, 0); // Release  
}  
data->btn_last_state = btn_curr;  
}  
  
/* MOTION REPORT (200Hz rate limit) */  
int64_t now = k_uptime_get();  
if (now - data->last_report >= 5 && (data->delta_x || data->delta_y)) {  
zmk_hid_send_mouse_report(data->delta_x * 4, data->delta_y * 4, 0, 0);  
data->delta_x = data->delta_y = 0;  
data->last_report = now;  
}  
}  
  
/* GPIO SHARED CALLBACK */  
static void gpio_callback(const struct device *port, struct gpio_callback *cb, uint32_t pins) {  
struct hall_trackball_data *data = CONTAINER_OF(cb, struct hall_trackball_data, gpio_cb[0]);  
hall_irq_handler(port, data);  
}  
  
/* INITIALIZATION */  
static int hall_trackball_init(const struct device *dev) {  
const struct hall_trackball_config *cfg = dev->config;  
struct hall_trackball_data *data = dev->data;  
  
/* MAP DT GPIO SPECS */  
data->xplus = &cfg->xplus_gpios;  
data->xminus = &cfg->xminus_gpios;  
data->yplus = &cfg->yplus_gpios;  
data->yminus = &cfg->yminus_gpios;  
data->btn = &cfg->btn_gpios;  
  
/* CONFIGURE HALL SENSORS: BOTH-EDGE interrupts */  
gpio_pin_configure_dt(data->xplus, GPIO_INPUT | GPIO_PULL_UP);  
gpio_pin_interrupt_configure_dt(data->xplus, GPIO_INT_EDGE_BOTH);  
  
gpio_pin_configure_dt(data->xminus, GPIO_INPUT | GPIO_PULL_UP);  
gpio_pin_interrupt_configure_dt(data->xminus, GPIO_INT_EDGE_BOTH);  
  
gpio_pin_configure_dt(data->yplus, GPIO_INPUT | GPIO_PULL_UP);  
gpio_pin_interrupt_configure_dt(data->yplus, GPIO_INT_EDGE_BOTH);  
  
gpio_pin_configure_dt(data->yminus, GPIO_INPUT | GPIO_PULL_UP);  
gpio_pin_interrupt_configure_dt(data->yminus, GPIO_INT_EDGE_BOTH);  
  
/* CONFIGURE CLICK BUTTON: FALLING EDGE only (press detection) */  
gpio_pin_configure_dt(data->btn, GPIO_INPUT | GPIO_PULL_UP);  
gpio_pin_interrupt_configure_dt(data->btn, GPIO_INT_EDGE_FALLING);  
  
/* SHARED CALLBACKS (RAM optimization: 5 GPIOs) */  
for (int i = 0; i < 5; i++) {  
gpio_init_callback(&data->gpio_cb[i], gpio_callback, NULL);  
}  
  
/* REGISTER INTERRUPTS */  
gpio_add_callback(data->xplus->port, &data->gpio_cb[0]);  
gpio_add_callback(data->xminus->port, &data->gpio_cb[1]);  
gpio_add_callback(data->yplus->port, &data->gpio_cb[2]);  
gpio_add_callback(data->yminus->port, &data->gpio_cb[3]);  
gpio_add_callback(data->btn->port, &data->gpio_cb[4]);  
  
/* RESET STATE */  
data->x_state = data->y_state = 0;  
data->delta_x = data->delta_y = 0;  
data->btn_last_state = true; // Button up (pullup)  
data->last_report = 0;  
atomic_set(&data->enabled, 1);  
  
LOG_INF("Hall trackball + click ready: X%d.%d Y%d.%d BTN%d",   
data->xplus->pin, data->xminus->pin,   
data->yplus->pin, data->yminus->pin, data->btn->pin);  
return 0;  
}  
  
static const struct hall_trackball_driver_api api;  
  
/* ZEPHYR DEVICE DEFINITION */  
#define HALL_TRACKBALL_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(my_hall_trackball)  
DEVICE_DT_DEFINE(HALL_TRACKBALL_NODE, hall_trackball_init, NULL,  
.data = &(struct hall_trackball_data){0},  
.config = &(struct hall_trackball_config){0},  
POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &api);