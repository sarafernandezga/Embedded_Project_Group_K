#include "includes.h"
#include "states.h"
#include "rgb.h"


void run_normal_mode(struct system_messages *data_sensors)
{
    printk("Implementar modo normal\n");

   
}


void change_drone_mode(struct system_messages *data_sensors)
{
    /* PROBLEM has priority over everything (NM5).
     * GPS_NO_FIX is not a fault (gps_valid == 0), only gps_valid < 0 is. */
    bool problem = !data_sensors->env_valid ||
                   !data_sensors->imu_valid ||
                   (data_sensors->gps_valid < 0) ||
                   (data_sensors->imu.state == PROBLEM);

    if (problem) {
        set_rgb_color(RED_LED);
    } else if (data_sensors->imu.state == FLYING) {
        set_rgb_color(YELLOW_LED);
    } else {
        set_rgb_color(GREEN_LED);
    }
}