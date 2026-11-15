#ifndef SIMU_H
#define SIMU_H

#include "states.h"

#define SIMU_MSGQ_MAX_MSGS  1

#define SIMU_FRAME_LENGTH       221
#define SIMU_FRAME_BUFFER_SIZE  (SIMU_FRAME_LENGTH + 1)

void run_simu_mode(struct system_messages *data_sensors);

#endif