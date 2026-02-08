#ifndef ZMK_HALL_TRACKBALL_H  
#define ZMK_HALL_TRACKBALL_H  
  
#include <zephyr/drivers/gpio.h>  
  
struct hall_trackball_config {  
struct gpio_dt_spec xplus_gpios; // Right motion (X+)  
struct gpio_dt_spec xminus_gpios; // Left motion (X-)  
struct gpio_dt_spec yplus_gpios; // Up motion (Y+)  
struct gpio_dt_spec yminus_gpios; // Down motion (Y-)  
struct gpio_dt_spec btn_gpios; // Left click (GPIO 10)  
};  
  
#endif  