#ifndef TEST_H
#define TEST_H

void test_gps(void);
void test_enviroment(void);
void test_IMU(void);

extern struct k_sem sem_main_gate;
void run_test_mode(void);

#endif