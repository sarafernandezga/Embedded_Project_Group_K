#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <math.h> // mirar si queremos meterlo o si realmente no 
#include "IMU.h" // must be uppercase 
#include "main.h"
#include "statistics.h"

#define I2C_NODE DT_NODELABEL(i2c2) 

// Definimos la cola de mensajes para el IMU (guarda el último mensaje disponible)
K_MSGQ_DEFINE(imu_q, sizeof(struct imu_m), 1, 4);
K_MSGQ_DEFINE(imu_stats_q, sizeof(struct imu_stats), 1, 4);
// Semáforo que el timer libera cada 20 ms 
K_SEM_DEFINE(imu_sem, 0, 1);

static void imu_sem_timer(struct k_timer *timer)
{
    ARG_UNUSED(timer);
    k_sem_give(&imu_sem);
}

K_TIMER_DEFINE(imu_timer, imu_sem_timer, NULL);

int IMU_initialization(const struct device *i2c_dev){
    if (!device_is_ready(i2c_dev)) {
            printk("Error: El bus I2C no está listo.\n");
            return -ENODEV;
        }

    int ret;

    // PWR_MGMT_1: Despertar (SLEEP=0) y reloj a PLL Gyro X 
    // revisar pero pone en el sensor que es la mejor cosa que hacer
    ret = i2c_reg_write_byte(i2c_dev, MPU6050_ADDR, MPU6050_PWR_MGMT_1_REG, 0x09);
    if (ret < 0) return ret;

    // Activa el filtro paso bajo a un ancho de banda de 
    ret = i2c_reg_write_byte(i2c_dev, MPU6050_ADDR, MPU6050_CONFIG_REG, 0x03);
    if (ret < 0) return ret;

    // SMPLRT_DIV: Muestreo a 50Hz -> 1kHz / (1 + 19) 
    // fórmula en la página 12, 1kHz de numerador ya que es la frecuencia del 
    // giroscopio al tener activado el paso bajo 
    ret = i2c_reg_write_byte(i2c_dev, MPU6050_ADDR, MPU6050_SMPLRT_DIV_REG, 19);
    if (ret < 0) return ret;

    // GYRO_CONFIG: Escala +-500 deg/s (FS_SEL=0)
    // los drones de seguimiento suelen tener entre 200 y 300 grados por segundo de rotacion 
    // para verlas con precision podemos usar el de 500 grados ya que no llegaria facilmente al maximo
    // y a la vez mantendriamos 65.5 LSB en cada grado por segundo 
    ret = i2c_reg_write_byte(i2c_dev, MPU6050_ADDR, MPU6050_GYRO_CONFIG_REG, 0x08);
    if (ret < 0) return ret;

    // revisar y hablar de las configuraciones y de la sensibilidad de cada uno 
    // ACCEL_CONFIG: Escala +-4g (AFS_SEL=0)
    // 8192 LSB/g esto quiere decir que cada g (fuerza de la gravedad)
    // tienen un rango de 8192
    ret = i2c_reg_write_byte(i2c_dev, MPU6050_ADDR, MPU6050_ACCEL_CONFIG_REG, 0x08);
    if (ret < 0) return ret;

    return 0;
}

enum drone_state calcular_estado(float ax, float ay, float az, float gx, float gy, float gz) {
    float accel_mag = sqrtf(ax*ax + ay*ay + az*az);
    float gyro_mag  = sqrtf(gx*gx + gy*gy + gz*gz);

    // revisar umbrales, son simplemente indicativos
    float umbral_movimiento_acc = 0.5f; // +- 0.5 m/s^2 respecto a G
    float umbral_movimiento_gyr = 5.0f; // > 5 deg/s de giro
    float umbral_choque = 25.0f;        // > 25 m/s^2 (impacto o caída)

    if (accel_mag > umbral_choque) {
        return PROBLEM;
    } 
    else if (fabsf(accel_mag - 9.81f) > umbral_movimiento_acc || gyro_mag > umbral_movimiento_gyr) {
        return FLYING;
    } 
    else {
        return IDLE;
    }
}

int IMU_read_and_process(const struct device *i2c_dev, struct imu_m *mimu, float dt) {
    uint8_t buf[14];

    if (i2c_burst_read(i2c_dev, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H_REG, buf, 14) != 0) {
        mimu->status = -1;
        return -EIO;
    }

    // Reconstrucción de 16 bits (Big-Endian)
    // de izquierda a derecha, primeras direcciones los bytes de mayor valor
    int16_t raw_ax = (int16_t)((buf[0] << 8) | buf[1]); //escribimos los primeros bits a la derecha y luego el siguiente byte a la derecha
    int16_t raw_ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t raw_az = (int16_t)((buf[4] << 8) | buf[5]);
    // buf[6] y buf[7] son temperatura (ignorados)
    int16_t raw_gx = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t raw_gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

    // Conversiones físicas coherentes con +-4g (8192 LSB/g) y +-500 deg/s (65.5 LSB/deg/s)
    float ax = (raw_ax / 8192.0f) * 9.81f;
    float ay = (raw_ay / 8192.0f) * 9.81f;
    float az = (raw_az / 8192.0f) * 9.81f; 

    float gx = raw_gx / 65.5f;
    float gy = raw_gy / 65.5f;
    float gz = raw_gz / 65.5f;

    mimu->vx += ax * dt;
    mimu->vy += ay * dt;
    mimu->vz += az * dt;

    mimu->x += mimu->vx * dt;
    mimu->y += mimu->vy * dt;
    mimu->z += mimu->vz * dt;

    // Integración numérica
    mimu->roll  += gx * dt;
    mimu->pitch += gy * dt;
    mimu->yaw   += gz * dt;

    mimu->ax = ax; mimu->ay = ay; mimu->az = az;
    mimu->gx = gx; mimu->gy = gy; mimu->gz = gz;
    mimu->state = calcular_estado(ax, ay, az, gx, gy, gz);
    mimu->status = 0;

    return 0;
}


/* Publishes the latest IMU message and wakes main only when the
 * tracker state changes (IDLE <-> FLYING <-> PROBLEM) */
static void imu_change_mode(struct imu_m *mimu)
{
    static enum drone_state last_state = IDLE;

    if (mimu->state != last_state) {
        last_state = mimu->state;
        k_event_post(&main_events, EVT_DRONE);
    }
}



void IMU_measurement(void *p1, void *p2, void *p3) {
    const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);

    struct imu_m mimu = {0};
    uint8_t err_count = 0;
    int64_t last_t;

    /* 30 s statistics of |acceleration| and |angular velocity| */
    struct stat_acc acc_stat, gyr_stat;
    int64_t window_start;
    bool was_active = false;

    stat_reset(&acc_stat);
    stat_reset(&gyr_stat);

    while (IMU_initialization(i2c_dev) < 0) {
        mimu.status = -1;
        mimu.state = PROBLEM;
        k_msgq_purge(&imu_q);
        k_msgq_put(&imu_q, &mimu, K_NO_WAIT);
        imu_change_mode(&mimu);       /* tell main (red LED) */
        k_sleep(K_SECONDS(2));
    }
    last_t = k_uptime_get();          /* dt must not include the retries */
    window_start = last_t;

    while (1) {
        k_sem_take(&imu_sem, K_FOREVER);
        int64_t now = k_uptime_get();
        float dt = (now - last_t) / 1000.0f;
        last_t = now;

        bool active = normal_active;

        /* Just entered NORMAL mode: start a new 30 s window */
        if (active && !was_active) {
            stat_reset(&acc_stat);
            stat_reset(&gyr_stat);
            window_start = now;
        }
        was_active = active;

        /* Close the 30 s window: publish the result and start again */
        if (active && (now - window_start) >= STATS_WINDOW_MS) {
            struct imu_stats out;

            stat_result(&acc_stat, &out.acc);
            stat_result(&gyr_stat, &out.gyr);
            k_msgq_purge(&imu_stats_q);
            k_msgq_put(&imu_stats_q, &out, K_NO_WAIT);

            stat_reset(&acc_stat);
            stat_reset(&gyr_stat);
            window_start = now;
        }

        if (IMU_read_and_process(i2c_dev, &mimu, dt) == 0) {
            err_count = 0;
            mimu.status = 0;

            if (active) {
                stat_add(&acc_stat, sqrtf(mimu.ax*mimu.ax + mimu.ay*mimu.ay + mimu.az*mimu.az));
                stat_add(&gyr_stat, sqrtf(mimu.gx*mimu.gx + mimu.gy*mimu.gy + mimu.gz*mimu.gz));
            }
        } else {
            if (err_count < IMU_ERR_LIMIT) { err_count++; }
            if (err_count < IMU_ERR_LIMIT) {
                continue;             /* isolated error: keep the last data */
            }
            mimu.status = -1;         /* 5 failures in a row: PROBLEM */
            mimu.state = PROBLEM;
        }

        k_msgq_purge(&imu_q);
        k_msgq_put(&imu_q, &mimu, K_NO_WAIT);

        imu_change_mode(&mimu);
    }
}
/* Start enviroment thread */
K_THREAD_DEFINE(
    IMU_measurement_id,    /* Thread identifier */
    2048,                       /* Stack size */
    IMU_measurement,       /* Entry function */
    NULL, NULL, NULL,           /* Parameters */
    4,                          /* Priority */
    0,                          /* Options */
    1000                            /* Start delay */
);