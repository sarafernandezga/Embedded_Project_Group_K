#ifndef MAIN_H
#define MAIN_H

#define EVT_BUTTON      BIT(0)   // 0x01
#define EVT_TIMER       BIT(1)   // 0x02
#define EVT_IMU         BIT(2)   // 0x04
#define EVT_GPS         BIT(3)   // 0x08
#define EVT_ENVIROMENT  BIT(4)

#define EVT_ALL_SENSORS (EVT_ENVIROMENT | EVT_IMU | EVT_GPS)

enum operating_mode{
    TEST_MODE,
    NORMAL_MODE,
    SIMU_MODE
};

extern struct k_event main_events;


#endif