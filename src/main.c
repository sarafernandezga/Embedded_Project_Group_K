#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>  

#include "includes.h"
#include "main.h"
#include "rgb.h"
#include "button.h"
#include "tests.h"
#include "enviroment.h"
#include "IMU.h"
#include "gps.h"

static void main_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    k_event_post(&main_events, EVT_TIMER);
}

K_TIMER_DEFINE(main_timer, main_timer_handler, NULL);


K_EVENT_DEFINE(main_events);

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
    k_timer_start(&main_timer, K_NO_WAIT, K_SECONDS(2));
    k_timer_start(&enviroment_timer, K_NO_WAIT, K_SECONDS(2));
    k_timer_start(&imu_timer, K_NO_WAIT, K_SECONDS(2));
    gps_set_update_rate(2000);
    printk("Todo ha sido inicializado \n");

    while (1){

        uint32_t ev = k_event_wait(&main_events,
                                    EVT_BUTTON | EVT_TIMER,
                                    false,        /* no borrar eventos previos */
                                    K_FOREVER);

        k_event_clear(&main_events, ev);

        if(ev & EVT_BUTTON){

            switch (op_mode){
                case TEST_MODE:
                    op_mode = NORMAL_MODE;
                    k_timer_start(&main_timer, K_NO_WAIT, K_SECONDS(5));
                    k_timer_start(&enviroment_timer, K_NO_WAIT, K_MSEC(2500));
                    k_timer_start(&imu_timer, K_NO_WAIT, K_MSEC(25));
                    gps_set_update_rate(1000);
                    printk("MODO NORMAL\n");
                    break;

                case NORMAL_MODE:
                    op_mode = SIMU_MODE;
                    printk("MODO SIMU\n");
                    break;

                case SIMU_MODE:
                    op_mode = TEST_MODE;
                    k_timer_start(&enviroment_timer, K_NO_WAIT, K_SECONDS(2));
                    k_timer_start(&imu_timer, K_NO_WAIT, K_SECONDS(2));
                    gps_set_update_rate(1000);
                    printk("MODO TEST\n");
                    break;

                default:
                    break;
                }

        }


        if(ev & EVT_TIMER){
            switch (op_mode){
                case TEST_MODE:
                    run_test_mode();     
                    
                    break;

                case NORMAL_MODE:
                    run_normal_mode(); 
                    
                    break;

                case SIMU_MODE:
                    run_simu_mode();
                    
                    break;

                default:
                    break;
                }
            }
        
    }
}