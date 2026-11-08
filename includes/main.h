#ifndef MAIN_H
#define MAIN_H

#define EVT_BUTTON 0x01
#define EVT_TIMER  0X02

enum operating_mode{
    TEST_MODE,
    NORMAL_MODE,
    SIMU_MODE
};

extern struct k_event app_events;


#endif