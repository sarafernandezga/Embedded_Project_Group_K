#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <math.h> // mirar si queremos meterlo o si realmente no 
#include "imu.h"

#define I2C_NODE DT_NODELABEL(i2c1) 

// Definimos la cola de mensajes para el IMU (guarda el último mensaje disponible)
K_MSGQ_DEFINE(imu_q, sizeof(struct imu_m), 1, 4);

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
        mimu->state = PROBLEM;
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
    float az = (raw_az / 8192.0f) * 9.81f; // quitar ?? 
    float az_sin_g = az - 9.81f;

    float gx = raw_gx / 65.5f;
    float gy = raw_gy / 65.5f;
    float gz = raw_gz / 65.5f;

    mimu->vx += ax * dt;
    mimu->vy += ay * dt;
    mimu->vz += az_sin_g * dt;

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

void IMU_measurement(void *p1, void *p2, void *p3) {
    const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);

    // Variables globales para los ángulos (asumiendo inicio en 0)
    struct imu_m mimu = {0};
    float dt = 0.02f; // 50 Hz = 20 ms
    uint8_t print_counter = 0; 

    if (IMU_initialization(i2c_dev) < 0) {
        printk("Error al inicializar el MPU6050\n");
        mimu.status = -1;
        mimu.state = PROBLEM;
        k_msgq_put(&imu_q, &mimu, K_NO_WAIT);
        return;
    }

    while (1) {

        if (IMU_read_and_process(i2c_dev, &mimu, dt) == 0) {
            // Imprimir cada 10 muestras (cada 200 ms) para no saturar la UART y perder los 50 Hz
            if (++print_counter >= 10) {
                print_counter = 0;
                printk("ACC [m/s2]: X=%.2f Y=%.2f Z=%.2f | GYR [deg/s]: X=%.1f Y=%.1f Z=%.1f | ANG [deg]: R=%.1f P=%.1f Y=%.1f\n",
                       (double)mimu.ax, (double)mimu.ay, (double)mimu.az,
                       (double)mimu.gx, (double)mimu.gy, (double)mimu.gz,
                       (double)mimu.roll, (double)mimu.pitch, (double)mimu.yaw);
            }
        } else {
            printk("Error en lectura burst I2C\n");
        }

        k_msgq_purge(&imu_q);
        k_msgq_put(&imu_q, &mimu, K_NO_WAIT);

        k_msleep(20);
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