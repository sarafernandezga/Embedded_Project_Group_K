#ifndef ENVIROMENT_H
#define ENVIROMENT_H

/* Struct ADC Message */
struct env_m {
    uint32_t temp;
    uint32_t hum;
    uint32_t light;
};

/* Message Queue ID */
extern struct k_msgq env_q;

/* Semaphore ID */
extern struct k_sem sem_env_gate;

/* ADC Pause Control */
void set_state_adc(bool pause);

#endif