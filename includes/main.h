#ifndef MAIN_H
#define MAIN_H

#define EVT_BUTTON      BIT(0)   // 0x01
#define EVT_TIMER       BIT(1)   // 0x02
#define EVT_DRONE       BIT(2)   // 0x04

enum operating_mode{
    TEST_MODE,
    NORMAL_MODE,
    SIMU_MODE
};

extern struct k_event main_events;


#endif