#ifndef GPS_H
#define GPS_H

/* GPS receive buffer */
#define GPS_RX_BUFFER_SIZE 256
#define GPS_MAX_FIELDS 20
#define GPS_NMEA_BUFFER_SIZE 128
#define GPS_MSGQ_MAX_MSGS  1

struct gps_data {
    float latitude;
    float longitude;
    float altitude;
    float hdop;
    uint8_t satellites;
    uint8_t fix_quality;
};

/* Message Queue ID */
extern struct k_msgq gps_msgq;

#endif