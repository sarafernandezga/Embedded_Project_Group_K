#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include "includes.h"
#include "main.h"
#include "rgb.h"
#include "button.h"
#include "states.h"
#include "enviroment.h"
#include "IMU.h"
#include "gps.h"

K_EVENT_DEFINE(main_events);

/* Timer callback runs in ISR context, so it only notifies main */
static void main_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    k_event_post(&main_events, EVT_TIMER);
}

K_TIMER_DEFINE(main_timer, main_timer_handler, NULL);

/* Get the latest message from each queue. If a queue is empty we keep the old data */
static void queue_update(struct system_messages *data_sensors)
{
    if (k_msgq_get(&imu_q, &data_sensors->imu, K_NO_WAIT) == 0) {
        data_sensors->imu_valid = (data_sensors->imu.status == 0);
    }

    /* gps_valid: 1 = fix, 0 = no fix, -1 = error */
    if (k_msgq_get(&gps_msgq, &data_sensors->gps, K_NO_WAIT) == 0) {
        if (data_sensors->gps.status == GPS_OK) {
            data_sensors->gps_valid = 1;
        } else if (data_sensors->gps.status == GPS_NO_FIX) {
            data_sensors->gps_valid = 0;
        } else {
            data_sensors->gps_valid = -1;
        }
    }

    if (k_msgq_get(&env_q, &data_sensors->env, K_NO_WAIT) == 0) {
        data_sensors->env_valid = (data_sensors->env.status == ALL_OK);
    }
}

int main(void)
{

    /* Initialize GPIOs */
    if (initialize_RGB() != 0) {
        printk("RGB: Initialization failed\n");
    }

    if (init_button() != 0) {
        printk("BUTTON: Initialization failed\n");
    }

    /* Initialize in TEST MODE */
    enum operating_mode op_mode = TEST_MODE;
    struct system_messages data_sensors = {0};

    set_rgb_color(BLUE_LED);

    k_timer_start(&enviroment_timer, K_NO_WAIT, K_SECONDS(2));
    k_timer_start(&imu_timer, K_NO_WAIT, K_SECONDS(2));
    gps_set_update_rate(2000);
    k_timer_start(&main_timer, K_NO_WAIT, K_SECONDS(2));

    while (1) {

        /* Wait for an event: button or timer */
        uint32_t ev = k_event_wait(&main_events,
                                   EVT_BUTTON | EVT_TIMER | EVT_DRONE,
                                   false,
                                   K_FOREVER);

        /* wait doesn't clear the bits, so we do it here */
        k_event_clear(&main_events, ev);

        /* Button: next mode and update timers frecuency */
        if (ev & EVT_BUTTON) {

            switch (op_mode) {
                case TEST_MODE:
                    /* Change Mode*/
                    op_mode = NORMAL_MODE;

                    /* Initialize RGB */
                    change_drone_mode(&data_sensors);

                    /* Initialize timers for this mode */
                    gps_set_update_rate(1000);
                    k_timer_start(&enviroment_timer, K_NO_WAIT, K_MSEC(2500));
                    k_timer_start(&imu_timer, K_NO_WAIT, K_MSEC(25));
                    k_timer_start(&main_timer, K_NO_WAIT, K_SECONDS(5));

                    /* Inform actual mode*/
                    printk("MODO NORMAL\n");
                    break;

                case NORMAL_MODE:
                    /* Change Mode*/
                    op_mode = SIMU_MODE;

                    /* Initialize timers for this mode */
                    gps_set_update_rate(1000);
                    k_timer_start(&enviroment_timer, K_NO_WAIT, K_MSEC(100));
                    k_timer_start(&imu_timer, K_NO_WAIT, K_MSEC(100));
                    k_timer_start(&main_timer, K_NO_WAIT, K_MSEC(100));

                    /* Inform actual mode*/
                    printk("MODO SIMU\n");
                    break;

                case SIMU_MODE:
                    /* Change Mode*/
                    op_mode = TEST_MODE;

                    /* Initialize RGB */
                    set_rgb_color(BLUE_LED);

                    /* Initialize timers for this mode */
                    gps_set_update_rate(2000);
                    k_timer_start(&enviroment_timer, K_NO_WAIT, K_SECONDS(2));
                    k_timer_start(&imu_timer, K_NO_WAIT, K_SECONDS(2));
                    k_timer_start(&main_timer, K_NO_WAIT, K_SECONDS(2));

                    /* Inform actual mode*/
                    printk("MODO TEST\n");
                    break;

                default:
                    break;
            }
        }

        /* Timer: read the queues and run the current mode */
        if (ev & EVT_TIMER) {
            queue_update(&data_sensors);

            switch (op_mode) {
                case TEST_MODE:   run_test_mode(&data_sensors);   break;
                case NORMAL_MODE: run_normal_mode(&data_sensors); break;
                case SIMU_MODE:   run_simu_mode(&data_sensors);   break;
                default: break;
            }
        }

        /* Timer: read the queues and run the current mode */
        if (ev & EVT_TIMER) {
            if (op_mode == NORMAL_MODE){
                queue_update(&data_sensors);
                change_drone_mode(&data_sensors);
            }
            
        }
    }
}