#ifndef MAIN_H
#define MAIN_H

/* Leds Modes */
enum rgb_modes {
    OFF_LED,
    BLUE_LED,
    RED_LED,
    GREEN_LED,
    YELLOW_LED,
    PURPLE_LED,
    CIAN_LED,
    WHITE_LED
};


enum dron_mode {
    IDLE,
    FLYING,
    PROBLEM
};

enum operating_mode{
    TEST_MODE,
    NORMAL_MODE,
    SIMU_MODE
};


#endif