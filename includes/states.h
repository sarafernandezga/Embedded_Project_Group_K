#ifndef STATES_H
#define STATES_H

#include "IMU.h"            /* struct imu_m, enum drone_state */
#include "gps.h"            /* struct gps_data */
#include "enviroment.h"     /* struct env_m */

struct system_messages {
    struct imu_m    imu;
    struct gps_data gps;
    struct env_m    env;

    uint16_t flags;
};

enum drone_state get_drone_state(const struct system_messages *d);
void change_drone_mode(struct system_messages *data_sensors);
const char *state_str(enum drone_state s);

void run_test_mode(struct system_messages *data_sensors);
void run_normal_mode(struct system_messages *data_sensors);

void normal_enter(void);        // restart 30 s window
void normal_exit(void);         // para salir hay que apagar la flag 

#endif