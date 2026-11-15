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

struct gps_data {
    double latitude, longitude, altitude, hdop;              
    uint8_t satellites, fix_quality;             
    char utc_time[9];    
    int gps_valid;                        
};

/* Message Queue ID */
extern struct k_msgq gps_msgq;

void gps_set_update_rate(uint16_t interval_ms);

void gps_set_duty_cycle(uint32_t standby_ms);

#endif