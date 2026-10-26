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

/* ----------- LCD -----------*/
#define MAX_CALIBRATE_LCD   95
#define MIN_CALIBRATE_LCD   25

/* Struct ADC Message */
struct env_m {
    float temp;
    float hum;
    float light;
};

/* Message Queue ID */
extern struct k_msgq env_q;

/* Semaphore ID */
extern struct k_sem sem_env_gate;

/* ADC Pause Control */
void set_state_adc(bool pause);

#endif