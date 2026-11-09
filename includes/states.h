#ifndef STATES_H
#define STATES_H

#include "imu.h"            /* struct imu_m, enum drone_state */
#include "gps.h"            /* struct gps_data */
#include "enviroment.h"     /* struct env_m */

struct system_messages {
    struct imu_m    imu;
    struct gps_data gps;
    struct env_m    env;
    bool imu_valid;
    int gps_valid;
    bool env_valid;
};


void test_gps(void);
void test_enviroment(void);
void test_IMU(void);

extern struct k_sem sem_main_gate;
void run_test_mode(struct system_messages *data_sensors);
void run_simu_mode(struct system_messages *data_sensors);
void run_normal_mode(struct system_messages *data_sensors);

#endif