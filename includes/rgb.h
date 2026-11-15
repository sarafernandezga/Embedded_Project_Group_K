#ifndef RGB_H
#define RGB_H
#include <stdbool.h>

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

int initialize_RGB(void);
void set_rgb_color(enum rgb_modes mode);
void rgb_set_pulsed(bool en);

#endif