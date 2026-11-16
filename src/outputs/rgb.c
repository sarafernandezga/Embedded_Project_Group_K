#include <zephyr/drivers/gpio.h>

#include "rgb.h"

/* Leds Variables */
static const struct gpio_dt_spec rgb_blue = GPIO_DT_SPEC_GET(DT_ALIAS(rgbblue), gpios);
static const struct gpio_dt_spec rgb_red = GPIO_DT_SPEC_GET(DT_ALIAS(rgbred), gpios);
static const struct gpio_dt_spec rgb_green = GPIO_DT_SPEC_GET(DT_ALIAS(rgbgreen), gpios);

static int initialize_RGB_pins(const struct gpio_dt_spec *pin_rgb)
{
    if (!device_is_ready(pin_rgb->port)) {
        printk("RGB: LED device not ready\n");
        return -1;
    }

    /* Configure the LED pin as output, initially inactive */
    if (gpio_pin_configure_dt(pin_rgb, GPIO_OUTPUT_INACTIVE) < 0) {
        printk("RGB: error configuring LED\n");
        return -1;
    }

    return 0;
}

int initialize_RGB(void){
    /* Initialize the three RGB pins */
    if (initialize_RGB_pins(&rgb_blue) != 0 ||
        initialize_RGB_pins(&rgb_red) != 0 ||
        initialize_RGB_pins(&rgb_green) != 0) 
    {
        return -1;
    }

    return 0;
}

static void rgb_apply(enum rgb_modes mode)
{
    /* Turn off all LEDs before selecting the new color */
    gpio_pin_set_dt(&rgb_blue, 0);
    gpio_pin_set_dt(&rgb_red, 0);
    gpio_pin_set_dt(&rgb_green, 0);

    /* Turn on the LEDs required for the selected color */
    switch (mode) {
        case OFF_LED:
            break;

        case RED_LED:
            gpio_pin_set_dt(&rgb_red, 1);
            break;

        case GREEN_LED:
            gpio_pin_set_dt(&rgb_green, 1);
            break;

        case BLUE_LED:
            gpio_pin_set_dt(&rgb_blue, 1);
            break;

        case YELLOW_LED:
            gpio_pin_set_dt(&rgb_green, 1);
            gpio_pin_set_dt(&rgb_red, 1);
            break;

        case PURPLE_LED:
            gpio_pin_set_dt(&rgb_red, 1);
            gpio_pin_set_dt(&rgb_blue, 1);
            break;

        case CIAN_LED:
            gpio_pin_set_dt(&rgb_blue, 1);
            gpio_pin_set_dt(&rgb_green, 1);

            break;

        case WHITE_LED:
            gpio_pin_set_dt(&rgb_red, 1);
            gpio_pin_set_dt(&rgb_blue, 1);
            gpio_pin_set_dt(&rgb_green, 1);
            break;
        
        default:
            printk("RGB: Invalid color mode\n");
            break;
    }

}

#define RGB_ON_MS      3
#define RGB_PERIOD_MS  10

static enum rgb_modes rgb_current = OFF_LED;
static bool rgb_pulsed;

static void rgb_off_cb(struct k_timer *t) { ARG_UNUSED(t); rgb_apply(OFF_LED); }
K_TIMER_DEFINE(rgb_off_timer, rgb_off_cb, NULL);

static void rgb_period_cb(struct k_timer *t)
{
    ARG_UNUSED(t);
    rgb_apply(rgb_current);
    k_timer_start(&rgb_off_timer, K_MSEC(RGB_ON_MS), K_NO_WAIT);
}
K_TIMER_DEFINE(rgb_period_timer, rgb_period_cb, NULL);

void set_rgb_color(enum rgb_modes mode)
{
    rgb_current = mode;
    if (!rgb_pulsed) {
        rgb_apply(mode);
    }
}

void rgb_set_pulsed(bool en)
{
    rgb_pulsed = en;
    if (en) {
        k_timer_start(&rgb_period_timer, K_NO_WAIT, K_MSEC(RGB_PERIOD_MS));
    } else {
        k_timer_stop(&rgb_period_timer);
        k_timer_stop(&rgb_off_timer);
        rgb_apply(rgb_current);
    }
}
