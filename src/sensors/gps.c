#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>

#include "includes.h"
#include "gps.h"

/* GPS UART device configuration */
#define GPS_UART_NODE DT_NODELABEL(usart1)

/* Get the UART device instance from the Device Tree */
static const struct device *gps_uart = DEVICE_DT_GET(GPS_UART_NODE);

/* Semaphore used to notify the GPS thread about incoming data */
K_SEM_DEFINE(gps_sem, 0, 1);

/* Message Queue */
K_MSGQ_DEFINE(gps_msgq, sizeof(struct gps_data), GPS_MSGQ_MAX_MSGS, 4);

/* Ring buffer */
static uint8_t gps_rx_buffer[GPS_RX_BUFFER_SIZE];
static struct ring_buf gps_rx_ringbuf;

/* NMEA buffer reconstruct */
static char gps_nmea_buffer[GPS_NMEA_BUFFER_SIZE];
static uint16_t gps_nmea_index = 0;

/* GPS data struct */
static struct gps_data gps_current_data;

/* UART interrupt callback */
static void gps_uart_isr(const struct device *dev,
                         void *user_data)
{
    ARG_UNUSED(user_data);

     uint8_t rx_data[32] = {0};
    int received;
    uint32_t stored;
    bool data_available = false;
    static uint32_t gps_rx_overflow_count;

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

    if (data_available) {
        k_sem_give(&gps_sem);
    }
}

static void gps_send_command(const char *command)
{
    while (*command != '\0') {
        uart_poll_out(gps_uart, *command);
        command++;
    }
}

/* Initialize GPS UART interface */
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

    /* Enable only GGA sentences */
    gps_send_command("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n");

    k_sleep(K_MSEC(100));

    /* Set GPS update rate to 1 Hz */
    gps_send_command("$PMTK220,1000*1F\r\n");

    printk("GPS: UART inicializado correctamente\n");

    return 0;
}

/* Initialize GPS circular buffer */
static void gps_buffer_init(void)
{
    ring_buf_init(
        &gps_rx_ringbuf,       /* Ring buffer management structure */
        sizeof(gps_rx_buffer), /* Available memory size in bytes */
        gps_rx_buffer          /* Memory area used to store data */
    );
}

/* Split a complete NMEA sentence into individual fields */
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

/* Convert an NMEA coordinate to decimal degrees */
static float gps_convert_coordinate(float coordinate, char direction)
{
    int degrees = (int)(coordinate / 100.0f);
    float minutes = coordinate - (degrees * 100.0f);

    float decimal_degrees = degrees + (minutes / 60.0f);

    if (direction == 'S' || direction == 'W') {
        decimal_degrees *= -1.0f;
    }

    return decimal_degrees;
}



/* Process the fields extracted from an NMEA sentence */
static void gps_process_fields(char *fields[], int field_count)
{
    /* Check if the sentence contains any fields */
    if (field_count <= 0) {
        return;
    }

    /* Identify the received NMEA sentence */
    if (strcmp(fields[0], "$GPGGA") == 0 ||
        strcmp(fields[0], "$GNGGA") == 0) {

        /* Extract UTC time */
        if (field_count > 1 && fields[1][0] != '\0') {
            int hours;
            int minutes;
            float seconds;

            if (sscanf(fields[1], "%2d%2d%f",
                    &hours, &minutes, &seconds) == 3) {

                snprintf(gps_current_data.utc_time,
                        sizeof(gps_current_data.utc_time),
                        "%02d:%02d:%02d",
                        hours, minutes, (int)seconds);
            }
        }

        if (field_count > 5 &&
            fields[2][0] != '\0' &&
            fields[3][0] != '\0' &&
            fields[4][0] != '\0' &&
            fields[5][0] != '\0') {

            gps_current_data.latitude =
                gps_convert_coordinate(strtof(fields[2], NULL), fields[3][0]);

            gps_current_data.longitude =
                gps_convert_coordinate(strtof(fields[4], NULL), fields[5][0]);
        }

        /* Extract fix quality */
        if (field_count > 6 && fields[6][0] != '\0') {
            gps_current_data.fix_quality = atoi(fields[6]);
        }

        /* Extract number of satellites */
        if (field_count > 7 && fields[7][0] != '\0') {
            gps_current_data.satellites = atoi(fields[7]);
        }

        /* Extract HDOP */
        if (field_count > 8 && fields[8][0] != '\0') {
            gps_current_data.hdop = strtof(fields[8], NULL);
        }

        /* Extract altitude */
        if (field_count > 9 && fields[9][0] != '\0') {
            gps_current_data.altitude = strtof(fields[9], NULL);
        }

        /* Send GPS data only when position fix is valid */
        if (gps_current_data.fix_quality > 0) {
            k_msgq_put(&gps_msgq, &gps_current_data, K_NO_WAIT);
        }

    }
}

static void gps_process_byte(uint8_t byte)
{
    /* Detect the beginning of a new NMEA sentence */
    if (byte == '$') {
        gps_nmea_index = 0;

    }

    /* Ignore bytes until a sentence starts */
    if (gps_nmea_index == 0 && byte != '$') {
        return;
    }

    /* Check buffer capacity */
    if (gps_nmea_index >= GPS_NMEA_BUFFER_SIZE - 1) {
        printk("GPS: NMEA buffer overflow\n");
        gps_nmea_index = 0;
        return;
    }

    /* Store received byte */
    gps_nmea_buffer[gps_nmea_index++] = (char)byte;

    /* Check if the sentence is complete */
    if (byte == '\n') {

        /* Add string terminator */
        gps_nmea_buffer[gps_nmea_index] = '\0';
        printk("GPS: Complete sentence:\n%s \n", gps_nmea_buffer);

        /* Split complete NMEA sentence into fields */
        char *fields[GPS_MAX_FIELDS];

        int field_count = gps_split_nmea(
            gps_nmea_buffer,
            fields,
            GPS_MAX_FIELDS
        );

        if (field_count > 0) {
            gps_process_fields(fields, field_count);
        }

        /* Reset buffer for next sentence */
        gps_nmea_index = 0;
    }
}



// ============================================================
// Thread
// ============================================================
void my_thread_gps(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    uint8_t rx_data[32];
    uint32_t received;

    /* Initialize GPS circular buffer */
    gps_buffer_init();

    /* Initialize GPS device */
    int ret = gps_uart_init();

    if (ret < 0){

    }

    while (1) {
        /* Wait until new GPS data is available */
        k_sem_take(&gps_sem, K_FOREVER);

        /* Read all available bytes from circular buffer */
        do {
            received = ring_buf_get(
                &gps_rx_ringbuf,
                rx_data,
                sizeof(rx_data)
            );

            /* Process each received byte */
            for (uint32_t i = 0; i < received; i++) {
                //printk("%c", rx_data[i]);
                gps_process_byte(rx_data[i]);
            }
        
        } while (received > 0);

    }

}

/* Start gps thread */
K_THREAD_DEFINE(
    my_thread_gps_id,           /* Thread identifier */
    1024,                        /* Stack size */
    my_thread_gps,              /* Entry function */
    NULL, NULL, NULL,           /* Parameters */
    4,                          /* Priority */
    0,                          /* Options */
    0                           /* Start delay */
);