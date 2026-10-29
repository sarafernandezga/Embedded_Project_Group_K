#ifndef ENVIROMENT_H
#define ENVIROMENT_H

/* ----------- Si7021 -----------*/
/* Si7021 direction */
#define SI7021_ADDR               0x40

/* Si7021 commands */
#define SI7021_READ_USER_REG      0xE7
#define SI7021_WRITE_USER_REG     0xE6

#define SI7021_MEASURE_RH         0xE5
#define SI7021_MEASURE_TEMP       0xE3

#define SI7021_READ_TEMP_FROM_RH  0xE0

/* CRC */
#define CRC_POLINOMIAL            0x31

/* ----------- LCD -----------*/
#define MAX_CALIBRATE_LCD   95
#define MIN_CALIBRATE_LCD   25

/* Thread Status */
enum env_status {
    ALL_OK,
    LIGHT_FAIL,
    SI7021_FAIL,
    TEMP_FAIL,
    MULTIPLE_FAIL,
    ADC_INIT_FAIL,
    SI7021_INIT_FAIL
};

/* Struct ADC Message */
struct env_m {
    float temp;
    float hum;
    float light;
    enum env_status status;
};

/* Message Queue ID */
extern struct k_msgq env_q;

/* Semaphore ID */
extern struct k_sem sem_env_gate;

#endif