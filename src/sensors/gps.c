#include <stdio.h> 
#include <string.h> 
#include <stdlib.h> 

#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>

#include "includes.h"
#include "gps.h"
#include "main.h"

/* GPS UART device configuration */
#define GPS_UART_NODE DT_NODELABEL(usart1)
static const struct device *gps_uart = DEVICE_DT_GET(GPS_UART_NODE);

/* GPS Semaphore */
K_SEM_DEFINE(gps_sem, 0, 1);

/* GPS Message Queue */
K_MSGQ_DEFINE(gps_msgq, sizeof(struct gps_data), GPS_MSGQ_MAX_MSGS, 4);

/* Ring buffer */
static uint8_t gps_rx_buffer[GPS_RX_BUFFER_SIZE];
static struct ring_buf gps_rx_ringbuf;

/* Buffer Rebuild NMEA sentence */
static char gps_nmea_buffer[GPS_NMEA_BUFFER_SIZE];
static uint16_t gps_nmea_index = 0;

// ============================================================
// UART interrupt callback
// ============================================================
/*
 * Runs every time the UART receives data.
 * Copies the bytes from the UART FIFO into the ring buffer and wakes up
 * the GPS thread. It does no parsing, to keep the ISR as short as possible.
 */
static void gps_uart_isr(const struct device *dev,
                         void *user_data)
{
    ARG_UNUSED(user_data);

    uint8_t rx_data[32] = {0};
    int received;
    uint32_t stored;
    bool data_available = false;
    static uint32_t gps_rx_overflow_count;   /* Bytes lost because the ring buffer was full */

    /* Update UART interrupt status */
    uart_irq_update(dev);

    /* Check if received data is available */
    while (uart_irq_rx_ready(dev)) {

        /* Read available bytes from UART FIFO */
        received = uart_fifo_read(
            dev,
            rx_data,
            sizeof(rx_data)
        );

        /* Stop if no more bytes are available */
        if (received <= 0) {
            break;
        }

        if (received > 0) {

            /* Store received bytes in circular buffer */
            stored = ring_buf_put(
                &gps_rx_ringbuf,
                rx_data,
                received
            );

            if (stored < received) {
                /* Update the number of lost bytes */
                gps_rx_overflow_count += (received - stored);
            }

            /* Notify GPS thread if data was stored */
            if (stored > 0) {
            data_available = true;
            }
        }
    }

    /* Wake up the GPS thread once, after draining the FIFO */
    if (data_available) {
        k_sem_give(&gps_sem);
    }
}

// ============================================================
// Configure GPS
// ============================================================
/* Send a null-terminated string to the GPS, byte by byte (blocking, no interrupts) */
static void gps_send_command(const char *command)
{
    while (*command != '\0') {
        uart_poll_out(gps_uart, *command);
        command++;
    }
}

/*
 * Set how often the GPS sends a position (PMTK220 command).
 * Builds "$PMTK220,<ms>*<checksum>\r\n", where the checksum is the XOR
 * of every character between '$' and '*', written as two hex digits.
 */
void gps_set_update_rate(uint16_t interval_ms)
{
    char command[32];
    char payload[20];
    uint8_t checksum = 0;

    /* Command body without '$' and checksum */
    snprintf(payload, sizeof(payload),"PMTK220,%u", interval_ms);

    /* NMEA checksum: XOR of all payload characters */
    for (int i = 0; payload[i] != '\0'; i++) {
        checksum ^= payload[i];
    }

    /* Final frame: $payload*CS\r\n */
    snprintf(command, sizeof(command), "$%s*%02X\r\n", payload, checksum);

    gps_send_command(command);
}


// ============================================================
// Initialize GPS UART interface
// ============================================================
/*
 * Prepares the UART: checks the device, registers the ISR, enables RX
 * interrupts and configures the GPS (GGA sentences only, 0.5 Hz).
 * Returns 0 on success or a negative errno on failure.
 */
static int gps_uart_init(void)
{
    /* Check if the UART device has been successfully initialized */
    if (!device_is_ready(gps_uart)) {
        printk("GPS: UART no esta preparado\n");
        return -ENODEV;
    }

    /* Register the UART interrupt callback function */
    int ret = uart_irq_callback_user_data_set(
        gps_uart,
        gps_uart_isr,
        NULL
    );

    /* Check if the callback registration was successful */
    if (ret != 0) {
        printk("GPS: Error registrando callback: %d\n", ret);
        return ret;
    }

    /* Enable UART receive interrupts */
    uart_irq_rx_enable(gps_uart);

    /* Enable only GGA sentences (PMTK314 selects which sentences are sent) */
    gps_send_command("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n");

    /* Give the GPS time to process the command before sending the next one */
    k_sleep(K_MSEC(100));

    /* Set GPS update rate 0.5 Hz */
    gps_set_update_rate(2000);

    return 0;
}

/* Public init: prepares the ring buffer first, then the UART and the GPS */
int gps_init(void){
    
    ring_buf_init(
        &gps_rx_ringbuf,       /* Ring buffer management structure */
        sizeof(gps_rx_buffer), /* Available memory size in bytes */
        gps_rx_buffer          /* Memory area used to store data */
    );

    return gps_uart_init();
}



// ============================================================
// Reconstruct NMEA sentence byte a byte
// ============================================================
/*
 * Feeds one received byte into the sentence buffer.
 * A sentence starts at '$' and ends at '\n'.
 * Returns: 1 = sentence complete (*sentence points to it),
 *          0 = still incomplete or byte ignored,
 *         <0 = error (buffer full, sentence discarded).
 */
static int gps_process_byte(uint8_t byte, char **sentence)
{
    if (sentence == NULL) {
        return -EINVAL;
    }

    /* Detect the beginning of a new NMEA sentence */
    if (byte == '$') {
        gps_nmea_index = 0;
    }

    /* Ignore bytes until a sentence starts */
    if (gps_nmea_index == 0 && byte != '$') {
        return 0;
    }

    /* Check buffer capacity */
    if (gps_nmea_index >= GPS_NMEA_BUFFER_SIZE - 1) {
        gps_nmea_index = 0;
        return -ENOMEM;
    }

    /* Store received byte */
    gps_nmea_buffer[gps_nmea_index++] = (char)byte;

    /* Check if the sentence is complete */
    if (byte == '\n') {
        gps_nmea_buffer[gps_nmea_index] = '\0';   /* Close the string */
        gps_nmea_index = 0;                        /* Ready for the next sentence */

        *sentence = gps_nmea_buffer;
        return 1;
    }

    return 0;
}
// ============================================================
// Transform NMEA sentence into data
// ============================================================
/*
 * Splits a complete sentence into its comma-separated fields.
 * Each ',' is replaced by '\0', so fields[] ends up pointing to
 * independent strings inside the original buffer (it modifies the buffer).
 * Returns the number of fields, or a negative errno on error.
 */
static int gps_split_nmea(char *sentence,     // Point to the first character of the NMEA buffer 
                          char *fields[],     // Point buffer to save fields
                          int max_fields)
{
    int field_count = 0;

    /* Check input parameters */
    if (sentence == NULL || fields == NULL || max_fields <= 0) {
        return -EINVAL;
    }

    /* First field starts at the beginning of the sentence */
    fields[field_count++] = sentence;

    /* Search for field separators */
    for (char *ptr = sentence; *ptr != '\0'; ptr++) {

        if (*ptr == ',') {

            /* Replace separator with string terminator */
            *ptr = '\0';

            /* Check available field slots */
            if (field_count >= max_fields) {
                return -ENOMEM;
            }

            /* Store the beginning of the next field */
            fields[field_count++] = ptr + 1;
        }
    }

    return field_count;
}

/*
 * Converts an NMEA coordinate (ddmm.mmmm) to decimal degrees.
 * Example: 4024.1234 -> 40 degrees + 24.1234 / 60 = 40.402057.
 * South and West give negative values.
 */
static float gps_convert_coordinate(float coordinate, char direction)
{
    int degrees = (int)(coordinate / 100.0f);               /* Whole degrees */
    float minutes = coordinate - (degrees * 100.0f);        /* Remaining minutes */

    float decimal_degrees = degrees + (minutes / 60.0f);

    if (direction == 'S' || direction == 'W') {
        decimal_degrees *= -1.0f;
    }

    return decimal_degrees;
}

/*
 * Fills a gps_data struct from the fields of a GGA sentence.
 * Empty fields are skipped, so they stay at 0.
 * Returns: 1 = valid data (fix available),
 *          0 = not a GGA sentence or no fix,
 *         <0 = invalid arguments.
 */
static int gps_process_fields(char *fields[], int field_count,
                              struct gps_data *data_out)
{
    if (fields == NULL || data_out == NULL || field_count <= 0) {
        return -EINVAL;
    }

    /* Only GGA sentences are handled */
    if (strcmp(fields[0], "$GPGGA") != 0 &&
        strcmp(fields[0], "$GNGGA") != 0) {
        return 0;
    }

    /* Start clean so no stale values from a previous sentence remain */
    memset(data_out, 0, sizeof(*data_out));

    /* Extract UTC time (field 1: hhmmss.sss) and format it as HH:MM:SS */
    if (field_count > 1 && fields[1][0] != '\0') {
        int hours;
        int minutes;
        float seconds;

        if (sscanf(fields[1], "%2d%2d%f", &hours, &minutes, &seconds) == 3) {
            snprintf(data_out->utc_time, sizeof(data_out->utc_time),
                     "%02d:%02d:%02d", hours, minutes, (int)seconds);
        }
    }

    /* Extract latitude and longitude (fields 2-5: value + N/S, value + E/W) */
    if (field_count > 5 &&
        fields[2][0] != '\0' && fields[3][0] != '\0' &&
        fields[4][0] != '\0' && fields[5][0] != '\0') {

        data_out->latitude  = gps_convert_coordinate(strtof(fields[2], NULL), fields[3][0]);
        data_out->longitude = gps_convert_coordinate(strtof(fields[4], NULL), fields[5][0]);
    }

    /* Extract fix quality (field 6, 0 = no fix) */
    if (field_count > 6 && fields[6][0] != '\0') {
        data_out->fix_quality = atoi(fields[6]);
        data_out->status = (data_out->fix_quality > 0) ? GPS_OK : GPS_NO_FIX;
    }

    /* Extract number of satellites (field 7) */
    if (field_count > 7 && fields[7][0] != '\0') {
        data_out->satellites = atoi(fields[7]);
    }

    /* Extract HDOP, horizontal precision (field 8) */
    if (field_count > 8 && fields[8][0] != '\0') {
        data_out->hdop = strtof(fields[8], NULL);
    }

    /* Extract altitude in meters (field 9) */
    if (field_count > 9 && fields[9][0] != '\0') {
        data_out->altitude = strtof(fields[9], NULL);
    }

    return 1;
}


/*
 * Converts a hexadecimal character into its decimal value.
 * Returns:
 *   0-15 = valid hexadecimal character
 *     -1 = invalid character
 */
static int gps_hex_to_int(char char_num)
{
    /* Check if the character is a decimal digit (0-9) */
    if (char_num >= '0' && char_num <= '9') {
        return char_num - '0';
    }

    /* Check if the character is an uppercase hexadecimal letter (A-F) */
    if (char_num >= 'A' && char_num <= 'F') {
        return char_num - 'A' + 10;
    }

    /* Check if the character is a lowercase hexadecimal letter (a-f) */
    if (char_num >= 'a' && char_num <= 'f') {
        return char_num - 'a' + 10;
    }

    /* Return -1 if the character is not hexadecimal */
    return -1;
}


/*
 * Verifies the checksum of an NMEA sentence.
 * Returns:
 *   1 = valid checksum
 *   0 = invalid checksum
 *  <0 = malformed sentence or invalid arguments
 */
static int gps_validate_checksum(const char *sentence)
{
    /* Check that the input pointer is valid and the sentence starts with '$' */
    if (sentence == NULL || sentence[0] != '$') {
        return -EINVAL;
    }

    /* Find the '*' character that separates the data from the checksum */
    const char *checksum_ptr = strchr(sentence, '*');

    /* Check that the '*' character exists */
    if (checksum_ptr == NULL) {
        return -EINVAL;
    }

    /* Check that two characters are available after '*' */
    if (checksum_ptr[1] == '\0' ||
        checksum_ptr[2] == '\0') {
        return -EINVAL;
    }

    /* Check that the sentence ends after the two checksum characters */
    char end_char = checksum_ptr[3];

    if (end_char != '\0' && end_char != '\r' && end_char != '\n') {
        return -EINVAL;
    }

    /* Calculate the checksum using XOR of all characters between '$' and '*' */
    uint8_t calculated_checksum = 0;

    for (const char *ptr = sentence + 1; ptr < checksum_ptr; ptr++) {
        calculated_checksum ^= (uint8_t)*ptr;
    }

    /* Convert hexadecimal characters into its numeric value */
    int high_nibble = gps_hex_to_int(checksum_ptr[1]);
    int low_nibble = gps_hex_to_int(checksum_ptr[2]);

    /* Check that both checksum characters are valid hexadecimal digits */
    if (high_nibble < 0 || low_nibble < 0) {
        return -EINVAL;
    }

    /* Combine both hexadecimal digits to reconstruct the received checksum byte */
    uint8_t received_checksum = (high_nibble << 4) | low_nibble;

    /* Compare the calculated checksum with the received checksum */
    return (calculated_checksum == received_checksum) ? 1 : 0;
}


/*
 * Reads the ring buffer byte by byte until a valid GGA sentence is found.
 * Pipeline: rebuild sentence -> (checksum) -> split into fields -> extract data.
 * Returns 1 if gps_data was filled with new data, 0 if the buffer ran out
 * of bytes without finding one.
 */
static int gps_get_data(struct gps_data *gps_data)
{
    uint8_t byte;
    int field_count;
    char *sentence;
    char *fields[GPS_MAX_FIELDS];
    int ret;

    while (ring_buf_get(&gps_rx_ringbuf, &byte, 1) == 1) {

         /* 1. Rebuild the sentence byte by byte */
        ret = gps_process_byte(byte, &sentence);

        if (ret <= 0) { /* 0: incomplete, <0: discarded. Keep reading bytes */
            continue;
        } 

        /* 2. Validate checksum (must be before split, which modifies the buffer) */
        ret = gps_validate_checksum(sentence);

        if (ret <= 0) {
            gps_data->status = GPS_ERR_CHECKSUM;
            return -1;
        }

        /* 3. Split into fields */
        field_count = gps_split_nmea(sentence, fields, GPS_MAX_FIELDS);

        if (field_count <= 0) {  /* No fields detected */
           gps_data->status = GPS_ERR_NO_DATA;
           return -1;
        }

        /* 4. Extract and convert data */
        ret = gps_process_fields(fields, field_count, gps_data);

        if (ret < 0) {
            gps_data->status = GPS_ERR_NO_DATA;
            return -1;
        }
        
        return 1;
    }

    return 0;
}

/* Publica un mensaje del GPS (dato o error) en un solo sitio */
static void gps_publish(struct gps_data *data)
{
    static bool last_fault = false;
    bool fault = (data->status != GPS_OK && data->status != GPS_NO_FIX);

    k_msgq_purge(&gps_msgq);
    k_msgq_put(&gps_msgq, data, K_NO_WAIT);

    /* Notify only on the transition healthy -> faulty */
    if (fault && !last_fault) {
        k_event_post(&main_events, EVT_DRONE);
    }
    
    last_fault = fault;
}

// ============================================================
// Thread GPS
// ============================================================
/*
 * GPS thread: initializes the GPS, then sleeps until the ISR signals new
 * bytes. When woken, it parses them and publishes any valid position
 * in gps_msgq for the rest of the application.
 */
static void my_thread_gps(void *p1, void *p2, void *p3)
{
    struct gps_data data;

    if (gps_init() != 0) {
        data.status = GPS_ERR_NO_DATA;
        gps_publish(&data);
        return;
    }

    while (1) {
        if (k_sem_take(&gps_sem, K_SECONDS(3)) != 0) {
            data.status = GPS_ERR_NO_DATA;
            gps_publish(&data);
            continue;
        }

        while (gps_get_data(&data) != 0) {
            gps_publish(&data);
        }
    }

}

/* Start gps thread automatically at boot */
K_THREAD_DEFINE(
    my_thread_gps_id,           /* Thread identifier */
    1024,                       /* Stack size */
    my_thread_gps,              /* Entry function */
    NULL, NULL, NULL,           /* Parameters */
    4,                          /* Priority */
    0,                          /* Options */
    0                           /* Start delay */
);