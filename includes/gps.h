#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include <zephyr/kernel.h>

/* GPS receive buffer */
#define GPS_RX_BUFFER_SIZE 256
#define GPS_MAX_FIELDS 20
#define GPS_NMEA_BUFFER_SIZE 128
#define GPS_MSGQ_MAX_MSGS  1

enum gps_status {
    ALL_GPS_OK,
    GPS_NO_FIX, 
    GPS_NO_DATA,
    INIT_GPS_FAIL
};

struct gps_data {
    char utc_time[9];

    float latitude;
    float longitude;
    float altitude;
    float hdop;

    uint8_t satellites;
    uint8_t fix_quality;

    enum gps_status status;
};

/* Message Queue ID */
extern struct k_msgq gps_msgq;

void gps_set_update_rate(uint16_t interval_ms);

#endif