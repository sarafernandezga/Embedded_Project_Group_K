#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>  

#include "includes.h"
#include "main.h"
#include "rgb.h"
#include "button.h"
#include "tests.h"

K_EVENT_DEFINE(main_events);

static void main_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    k_event_post(&main_events, EVT_TIMER);
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

    while (1){

        uint32_t ev = k_event_wait(&main_events,
                                    EVT_BUTTON | EVT_TIMER,
                                    false,        /* no borrar eventos previos */
                                    K_FOREVER);


        if(ev & EVT_BUTTON){

        }

        if(ev & EVT_TIMER){

            switch (op_mode){
                case TEST_MODE:
                    run_test_mode();      /* un "tick" de test */
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
}