#include "includes.h"
#include "states.h"
#include "rgb.h"
#include "statistics.h"

// ============================================================
// Tracker state
// ============================================================
/* PROBLEM has priority over everything (NM5).
 * GPS_NO_FIX is not a fault (gps_valid == 0), only gps_valid < 0 is. */
enum drone_state get_drone_state(const struct system_messages *d)
{
    if (!d->env_valid || !d->imu_valid ||
        d->gps_valid < 0 || d->imu.state == PROBLEM) {
        return PROBLEM;
    }

    return d->imu.state;      /* IDLE or FLYING */
}

const char *state_str(enum drone_state s)
{
    switch (s) {
    case IDLE:    return "IDLE";
    case FLYING:  return "FLYING";
    case PROBLEM: return "PROBLEM";
    default:      return "UNKNOWN";
    }
}

/* RGB: green = IDLE, yellow = FLYING, red = PROBLEM (NM6) */
void change_drone_mode(struct system_messages *data_sensors)
{
    switch (get_drone_state(data_sensors)) {
        case PROBLEM: set_rgb_color(RED_LED);    break;
        case FLYING:  set_rgb_color(YELLOW_LED); break;
        default:      set_rgb_color(GREEN_LED);  break;
    }
}

// ============================================================
// Reports
// ============================================================
/* Prints the NORMAL mode report (every 5 s) with the latest data of each sensor */
void print_report(const char *mode, const struct system_messages *data_sensors)
{
    printk("MODE: %s\n", mode);

    /* GPS: only print the position if there is a fix */
    if (data_sensors->gps_valid == 1) {
        printk("GPS: fix=%u sats=%02u lat=%+010.6f lon=%+011.6f alt=%+08.1f m time=%s\n",
               (unsigned)data_sensors->gps.fix_quality, (unsigned)data_sensors->gps.satellites,
               (double)data_sensors->gps.latitude, (double)data_sensors->gps.longitude,
               (double)data_sensors->gps.altitude, data_sensors->gps.utc_time);
        } else if (data_sensors->gps_valid == 0) {
                printk("GPS: fix=0 sats=%02u time=%s (no fix)\n",
                    (unsigned)data_sensors->gps.satellites,
                    data_sensors->gps.utc_time[0] ? data_sensors->gps.utc_time : "--:--:--");
            } else {
                printk("GPS: error (no data / bad checksum)\n");
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

    printk("STATUS: %s\n\n", state_str(get_drone_state(data_sensors)));
}

static void print_stat(const char *name, const char *unit, const struct stat_f *s)
{
    if (s->n == 0) {
        printk("  %-6s: no data\n", name);
        return;
    }

    printk("  %-6s: mean=%+8.2f max=%+8.2f min=%+8.2f %s\n",
           name, (double)s->mean, (double)s->max, (double)s->min, unit);
}

/* Prints the 30 s statistics when a sensor thread has published a new window */
static void stats_report(void)
{
    struct imu_stats imu_s;
    struct env_stats env_s;

    if (k_msgq_get(&imu_stats_q, &imu_s, K_NO_WAIT) == 0) {
        printk("STATS IMU (last 30 s):\n");
        print_stat("|ACC|",  "m/s2",  &imu_s.acc);
        print_stat("|GYR|",  "deg/s", &imu_s.gyr);
        printk("\n");
    }

    if (k_msgq_get(&env_stats_q, &env_s, K_NO_WAIT) == 0) {
        printk("STATS ENV (last 30 s):\n");
        print_stat("LIGHT", "%",  &env_s.light);
        print_stat("TEMP",  "C",  &env_s.temp);
        print_stat("HUM",   "%",  &env_s.hum);
        printk("\n");
    }
}

// ============================================================
// Functions called by the FSM in main.c
// ============================================================
/* Called when NORMAL mode starts: drop old statistics from other modes */
volatile bool normal_active = false;

/* Called when NORMAL mode starts */
void normal_enter(void)
{
    k_msgq_purge(&imu_stats_q);       /* drop old results */
    k_msgq_purge(&env_stats_q);
    normal_active = true;             /* sensor threads start a new window */
}

/* Called when NORMAL mode ends */
void normal_exit(void)
{
    normal_active = false;
}
/* Called every 5 s */
void run_normal_mode(struct system_messages *data_sensors)
{
    print_report("NORMAL", data_sensors);
    change_drone_mode(data_sensors);
    stats_report();
}