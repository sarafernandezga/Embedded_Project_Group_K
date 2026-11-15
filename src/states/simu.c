#include "includes.h"
#include "states.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>

#define SIMU_UART_NODE DT_NODELABEL(lpuart1)

static const struct device *simu_uart = DEVICE_DT_GET(SIMU_UART_NODE);

K_MSGQ_DEFINE(simu_msgq, sizeof(struct system_messages), SIMU_MSGQ_MAX_MSGS, 4);


/* SIMU frame: pending. In SIMU mode nothing else may be printed (SM6, GR5) */
void run_simu_mode(struct system_messages *data_sensors)
{
    if (data_sensors == NULL) {
        return;
    }

    k_msgq_purge(&simu_msgq);
    k_msgq_put(&simu_msgq, data_sensors, K_NO_WAIT);

}


static double clamp_float(float value, float min, float max)
{
    if (value < min) {
        return (double)min;
    }

    if (value > max) {
        return (double)max;
    }

    return (double)value;
}

int simu_build_frame(const struct system_messages *data_sensors,
                     uint16_t sequence,
                     uint32_t timestamp_ms,
                     char *frame,
                     size_t frame_size)
{
    int len;
    uint8_t checksum = 0;
    uint16_t flags;
    const char *state;
    char state_field[9];

    if (data_sensors == NULL || frame == NULL) {
        return -EINVAL;
    }

    if (frame_size < SIMU_FRAME_BUFFER_SIZE) {
        return -ENOMEM;
    }

    /*
     * STATE: 8 caracteres, completando con '_'.
     */
    state = state_str(get_drone_state(data_sensors));

    memset(state_field, '_', 8);
    state_field[8] = '\0';

    for (size_t i = 0; i < 8 && state[i] != '\0'; i++) {
        state_field[i] = state[i];
    }

    /*
     * FLAGS:
     * bit 0: IMU inválida
     * bit 1: GPS inválido
     * bit 2: LDR inválido
     * bit 3: Si7021 inválido
     *
     * Los bits 4-7 se conservan.
     * Los bits 8-15 permanecen a cero.
     */
    flags = data_sensors->flags & 0x00F0;

    if (!data_sensors->imu.valid_imu) {
        flags |= 0x0001;
    }

    if (data_sensors->gps.gps_valid != 1) {
        flags |= 0x0002;
    }

    if (!data_sensors->env.ldr_valid) {
        flags |= 0x0004;
    }

    if (!data_sensors->env.si7021_valid) {
        flags |= 0x0008;
    }

    /* Estimación inercial sin corrección GPS */
    flags |= 0x0040;

    /*
     * Construcción de la trama hasta FLAGS.
     * Los anchos de campo corresponden al protocolo.
     */
    len = snprintf(
        frame,
        frame_size,
        "$SIMU1,"
        "%05u,"
        "%010u,"
        "%u,"
        "%02u,"
        "%+010.6f,"
        "%+011.6f,"
        "%+08.1f,"
        "%+08.2f,%+08.2f,%+08.2f,"
        "%+07.2f,%+07.2f,%+07.2f,"
        "%+07.2f,%+07.2f,%+07.2f,"
        "%+07.2f,%+07.2f,%+07.2f,"
        "%+07.1f,%+07.1f,%+07.1f,"
        "%05.1f,"
        "%+06.1f,"
        "%05.1f,"
        "%s,"
        "%04X",

        (unsigned int)sequence,
        (unsigned int)timestamp_ms,

        (unsigned int)(data_sensors->gps.fix_quality == 1),
        (unsigned int)data_sensors->gps.satellites,

        data_sensors->gps.latitude,
        data_sensors->gps.longitude,
        data_sensors->gps.altitude,

        clamp_float(data_sensors->imu.x, -9999.99f, 9999.99f),
        clamp_float(data_sensors->imu.y, -9999.99f, 9999.99f),
        clamp_float(data_sensors->imu.z, -9999.99f, 9999.99f),

        (double)data_sensors->imu.vx,
        (double)data_sensors->imu.vy,
        (double)data_sensors->imu.vz,

        (double)data_sensors->imu.roll,
        (double)data_sensors->imu.pitch,
        (double)data_sensors->imu.yaw,

        (double)data_sensors->imu.ax,
        (double)data_sensors->imu.ay,
        (double)data_sensors->imu.az,

        (double)data_sensors->imu.gx,
        (double)data_sensors->imu.gy,
        (double)data_sensors->imu.gz,

        (double)data_sensors->env.light,
        (double)data_sensors->env.temp,
        (double)data_sensors->env.hum,

        state_field,
        (unsigned int)flags
    );

    if (len < 0 || (size_t)len >= frame_size) {
        return -EINVAL;
    }

    /*
     * El prefijo debe tener exactamente 216 bytes.
     * Si no, algún campo ha excedido su ancho.
     */
    if (len != 216) {
        return -ERANGE;
    }

    /*
     * Checksum XOR desde después de '$'
     * hasta el último carácter anterior a '*'.
     */
    for (int i = 1; i < len; i++) {
        checksum ^= (uint8_t)frame[i];
    }

    /*
     * Añadir *CS\r\n
     */
    int trailer_len = snprintf(
        &frame[len],
        frame_size - (size_t)len,
        "*%02X\r\n",
        checksum
    );

    if (trailer_len != 5) {
        return -EINVAL;
    }

    if (len + trailer_len != SIMU_FRAME_LENGTH) {
        return -ERANGE;
    }

    return SIMU_FRAME_LENGTH;
}




int simu_send_frame(const char *frame, size_t len)
{
    if (frame == NULL) {
        return -EINVAL;
    }

    if (!device_is_ready(simu_uart)) {
        return -ENODEV;
    }

    for (size_t i = 0; i < len; i++) {
        uart_poll_out(simu_uart, (uint8_t)frame[i]);
    }

    return 0;
}


void my_thread_simu(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct system_messages simu_data = {0};
    char frame[SIMU_FRAME_BUFFER_SIZE];

    static uint16_t sequence = 0;


    while (1) {

        /* Obtener los datos más recientes */
         if (k_msgq_get(&simu_msgq, &simu_data, K_FOREVER) == 0) {

            int len = simu_build_frame(
                &simu_data,
                sequence++,
                (uint32_t)k_uptime_get_32(),
                frame,
                sizeof(frame)
            );

            if (len == SIMU_FRAME_LENGTH) {
                simu_send_frame(frame, (size_t)len);
            }
        }


       
    }
}

/* Start simu thread */
K_THREAD_DEFINE(
    my_thread_simu_id,    /* Thread identifier */
    2048,                        /* Stack size */
    my_thread_simu,       /* Entry function */
    NULL, NULL, NULL,           /* Parameters */
    4,                          /* Priority */
    0,                          /* Options */
    0                           /* Start delay */
);