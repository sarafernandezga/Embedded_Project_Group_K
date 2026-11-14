#include "includes.h"
#include "states.h"

/* TEST MODE: all measured values every 2 s (TM2, TM3, TM5) */
void run_test_mode(struct system_messages *data_sensors)
{
    print_report("TEST", data_sensors);
}