#include <zephyr/drivers/gpio.h>

#include "includes.h"
#include "main.h"
#include "rgb.h"
#include "button.h"

static void main_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    //k_sem_give(&sem_main_gate);
}

K_TIMER_DEFINE(main_timer, main_timer_handler, NULL);


int main(void)
{

    if (initialize_RGB() != 0){
        printk("RGB: Initialization failed\n");
    }

    if (init_button() != 0){
        printk("BUTTON: Initialization failed\n");
    }

    enum operating_mode op_mode = TEST_MODE;
    set_rgb_color(BLUE_LED);
    k_timer_start(&main_timer, K_SECONDS(2), K_SECONDS(2));

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



