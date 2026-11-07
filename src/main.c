#include <zephyr/drivers/gpio.h>

#include "includes.h"
#include "main.h"

/* Leds Variables */
static const struct gpio_dt_spec rgb_blue = GPIO_DT_SPEC_GET(DT_ALIAS(rgbblue), gpios);
static const struct gpio_dt_spec rgb_red = GPIO_DT_SPEC_GET(DT_ALIAS(rgbred), gpios);
static const struct gpio_dt_spec rgb_green = GPIO_DT_SPEC_GET(DT_ALIAS(rgbgreen), gpios);

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(userbt), gpios);

/* Button Callback */
static struct gpio_callback button_cb_data;

K_SEM_DEFINE(sem_main_gate, 0, 1);


static void button_isr(const struct device *port,
                       struct gpio_callback *cb,
                       gpio_port_pins_t pins)
{   
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

   /* Notifies the thread when the button state changes */
    k_sem_give(&sem_main_gate);
    
}

/* Initialize button */
int init_button(void){
    /* Check if the GPIO device is ready */
    if (!gpio_is_ready_dt(&button)) {
        printk("BUTTON: Error initializing GPIO\n");
        return -ENODEV;
    }

    /* Configure the button as input */
    int ret = gpio_pin_configure_dt(&button, GPIO_INPUT);

    if (ret != 0){
        printk("BUTTON: Error configuring GPIO\n");
        return ret;
    }

    /* Register the GPIO callback */
    gpio_init_callback(&button_cb_data, button_isr, BIT(button.pin));

    ret = gpio_add_callback(button.port, &button_cb_data);

    if (ret != 0){
        printk("BUTTON: Error adding callback\n");
        return ret;
    }

    /* Enable interrupts on both button edges */
    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);  //en vez de edge to activate both, asi devuelve o 0 o 1
    
    if (ret != 0){
        printk("BUTTON: Error configuring interruption\n");
        return ret;
    }

    return ret;
}

static void main_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    k_sem_give(&sem_main_gate);
}

K_TIMER_DEFINE(main_timer, main_timer_handler, NULL);

static int initialize_RGB(const struct gpio_dt_spec *pin_rgb)
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

static void set_rgb_color(enum rgb_modes mode)
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



int main(void)
{
    
    k_timer_start(&main_timer, K_SECONDS(2), K_SECONDS(2));

    /* Initialize the three RGB pins */
    if (initialize_RGB(&rgb_blue) != 0 ||
        initialize_RGB(&rgb_red) != 0 ||
        initialize_RGB(&rgb_green) != 0) 
    {
        printk("RGB: Initialization failed\n");
    }

    if (init_button() != 0){
        printk("BUTTON: Initialization failed\n");
    }

    enum operating_mode op_mode = TEST_MODE;
    set_rgb_color(BLUE_LED);

    while (1) {
        
        k_sem_take(&sem_main_gate, K_FOREVER);

        switch (op_mode)
        {
            case TEST_MODE:

                break;

            case NORMAL_MODE:

                break;

            case SIMU_MODE:

                break;

            default:

                break;
        }
        



    }
}



