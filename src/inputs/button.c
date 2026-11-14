#include <zephyr/drivers/gpio.h>

#include "button.h"
#include "main.h"

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(userbt), gpios);

/* Button Callback */
static struct gpio_callback button_cb_data;

static void button_isr(const struct device *port,
                       struct gpio_callback *cb,
                       gpio_port_pins_t pins)
{   
    ARG_UNUSED(port);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    /* Notifies the thread when the button state changes */
    k_event_post(&main_events, EVT_BUTTON);
    
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