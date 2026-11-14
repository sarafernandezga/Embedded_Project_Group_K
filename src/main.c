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

/* At boot the sensor threads need a moment to publish their first sample */
#define BOOT_SETTLE_MS 1200

K_EVENT_DEFINE(main_events);

/* Timer callback runs in ISR context, so it only notifies main */
static void main_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    k_event_post(&main_events, EVT_TIMER);
}

// To get to the next mode when the button is pressed
static enum operating_mode next_mode(enum operating_mode mode)
{
    return (mode == SIMU_MODE) ? TEST_MODE : (enum operating_mode)(mode + 1);
}

/* Periods of each module for every operating mode */
struct mode_cfg {
    const char *name;
    uint16_t gps_ms;
    uint16_t env_ms;
    uint16_t imu_ms;
    uint16_t main_ms;     /* report / output period */
};

static const struct mode_cfg mode_table[] = {
    [TEST_MODE]   = { "TEST",   2000, 2000, 50, 2000 },
    [NORMAL_MODE] = { "NORMAL", 1000, 2500, 20, 5000 },
    [SIMU_MODE]   = { "SIMU",   1000, 1000, 20,  100 },
};

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

/* Configure timers, GPS rate and RGB for the new mode */
static void enter_mode(enum operating_mode mode, struct system_messages *data_sensors, bool boot)
{
    const struct mode_cfg *cfg = &mode_table[mode];

    gps_set_update_rate(cfg->gps_ms);
    k_timer_start(&enviroment_timer, K_NO_WAIT, K_MSEC(cfg->env_ms));
    k_timer_start(&imu_timer,        K_NO_WAIT, K_MSEC(cfg->imu_ms));
    k_timer_start(&main_timer, boot ? K_MSEC(BOOT_SETTLE_MS) : K_NO_WAIT, K_MSEC(cfg->main_ms));

    normal_exit();     /* the NORMAL case below sets it again */
    switch (mode) {
        case TEST_MODE:
            set_rgb_color(BLUE_LED);
            break;
        case NORMAL_MODE:
            normal_enter();                    /* restart the 30 s statistics window */
            change_drone_mode(data_sensors);   /* green / yellow / red */
            break;
        case SIMU_MODE:
            set_rgb_color(PURPLE_LED);         /* dedicated SIMU colour (SM7) */
            break;
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

    enter_mode(op_mode, &data_sensors, true);

    while (1) {

        /* Wait for an event: button or timer. wait_safe clears the received bits itself */
        uint32_t ev = k_event_wait_safe(&main_events,
                                        EVT_BUTTON | EVT_TIMER | EVT_DRONE,
                                        false,
                                        K_FOREVER);

        /* Button: next mode and update timers frecuency */
        if (ev & EVT_BUTTON) {
            op_mode = next_mode(op_mode);
            enter_mode(op_mode, &data_sensors, false);
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

        /* Tracker state or sensor health changed: refresh the LED right away */
        if ((ev & EVT_DRONE) && op_mode == NORMAL_MODE) {
            queue_update(&data_sensors);
            change_drone_mode(&data_sensors);
        }
    }
}