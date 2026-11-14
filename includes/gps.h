#ifndef GPS_H
#define GPS_H

#include <stdint.h>
#include <zephyr/kernel.h>

/* GPS receive buffer */
#define GPS_RX_BUFFER_SIZE 256
#define GPS_MAX_FIELDS 20
#define GPS_NMEA_BUFFER_SIZE 128
#define GPS_MSGQ_MAX_MSGS  1
/* Consecutive bad sentences before a communication error is reported */
#define GPS_CHK_ERR_LIMIT  5

enum gps_status {
    GPS_OK,
    GPS_ERR_NO_DATA,   /* silencio: cable o alimentación */
    GPS_ERR_CHECKSUM,  /* datos corruptos */
    GPS_NO_FIX,         /* comunica, pero sin posición */
};

struct gps_data {
    float latitude, longitude, altitude, hdop;   
    enum gps_status status;                      
    uint8_t satellites, fix_quality;             
    char utc_time[9];                           
};

/* Message Queue ID */
extern struct k_msgq gps_msgq;

void gps_set_update_rate(uint16_t interval_ms);

#endif