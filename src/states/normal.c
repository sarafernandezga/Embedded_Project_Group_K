#include "includes.h"
#include "states.h"
#include "rgb.h"


// ============================================================
// TEST MODE
// ============================================================
/* Converts the tracker state to text so it can be printed */
static const char *state_to_str(struct system_messages *data_sensors)
{
    /* PROBLEM has priority over everything (NM5).
     * GPS_NO_FIX is not a fault (gps_valid == 0), only gps_valid < 0 is. */
    bool problem = !data_sensors->env_valid ||
                   !data_sensors->imu_valid ||
                   (data_sensors->gps_valid < 0) ||
                   (data_sensors->imu.state == PROBLEM);

    if (problem) {
        return "PROBLEM";
    } else if (data_sensors->imu.state == FLYING) {
        return "FLYING";
    }

    return "IDLE";
}

/* Prints the NORMAL mode report (every 5 s) with the latest data of each sensor */
static void basic_report(struct system_messages *data_sensors){

    /* GPS: only print the position if there is a fix */
    if (data_sensors->gps_valid == 1) {
        printk("GPS: fix=%u sats=%02u lat=%+010.6f lon=%+011.6f alt=%+08.1f m time=%s\n",
               (unsigned)data_sensors->gps.fix_quality, (unsigned)data_sensors->gps.satellites,
               (double)data_sensors->gps.latitude, (double)data_sensors->gps.longitude,
               (double)data_sensors->gps.altitude, data_sensors->gps.utc_time);
    } else if (data_sensors->gps_valid == 0) {
        printk("GPS: no fix\n");
    } else {
        printk("GPS: error\n");
    }

    /* IMU: acceleration, angular velocity and integrated angles */
    if (data_sensors->imu_valid) {
        printk("IMU: ax=%+07.2f m/s2 ay=%+07.2f m/s2 az=%+07.2f m/s2\n",
               (double)data_sensors->imu.ax, (double)data_sensors->imu.ay, (double)data_sensors->imu.az);
        printk("IMU: gx=%+07.1f deg/s gy=%+07.1f deg/s gz=%+07.1f deg/s\n",
               (double)data_sensors->imu.gx, (double)data_sensors->imu.gy, (double)data_sensors->imu.gz);
        printk("ATT: roll=%+07.2f deg pitch=%+07.2f deg yaw=%+07.2f deg\n",
               (double)data_sensors->imu.roll, (double)data_sensors->imu.pitch, (double)data_sensors->imu.yaw);
    } else {
        printk("IMU: error\n");
    }

    /* Light (LDR) and temperature/humidity sensor */
    if (data_sensors->env_valid) {
        printk("LIGHT: %05.1f %%\n", (double)data_sensors->env.light);
        printk("ENV: temp=%+06.1f C hum=%05.1f %%\n",
               (double)data_sensors->env.temp, (double)data_sensors->env.hum);
    } else {
        printk("ENV: error\n");
    }

    printk("STATUS: %s\n\n", state_to_str(data_sensors));

}

/* Prints the NORMAL mode report (every 30 s) with medium, maximum and minimum of each sensor */
static void complete_report(struct system_messages *data_sensors){
    printk("Implement report 30s");

}


void run_normal_mode(struct system_messages *data_sensors)
{
    static uint8_t report_count = 0;

    basic_report(data_sensors);

    if (++report_count >= 6) {            /* 6 x 5 s = 30 s */
        complete_report(data_sensors);    /* prints mean / max / min */
        report_count = 0;
    }
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