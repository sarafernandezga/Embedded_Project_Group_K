#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <math.h> // mirar si queremos meterlo o si realmente no 

#define I2C_NODE DT_NODELABEL(i2c1) 

#define MPU6050_ADDR    0x68    // Dirección I2C

// Direcciones de registros
#define SMPLRT_DIV_REG  0x19 // donde podemos cambiar la frecuencia del giroscopio
#define CONFIG_REG      0x1A // sincronización de trama y DLPF
#define GYRO_CONFIG_REG 0x1B // define la escala del giroscopio mediante FS_SEL
#define ACCEL_CONFIG_REG 0x1C // rango del acelerometro mediante AFS_SEL
#define PWR_MGMT_1_REG  0x6B // si usar o no el sensor de temperatura integrado
                             // lo vamos a desactivar ya que es redundante
#define ACCEL_XOUT_H_REG 0x3B // Primer registro de datos (Acelerómetro X byte alto)
                             
void actualizar_estado(float ax, float ay, float az, float gx, float gy, float gz) {
    // Cálculo de magnitudes (módulo de los vectores)
    float accel_mag = sqrtf(ax*ax + ay*ay + az*az);
    float gyro_mag = sqrtf(gx*gx + gy*gy + gz*gz);
    
    // Umbrales sugeridos (se deben ajustar todavía)
    // La gravedad constante es ~9.81 m/s^2. Si el módulo difiere mucho de 9.81, hay movimiento o impacto.
    float umbral_movimiento_acc = 0.5f; // +- 0.5 m/s^2 respecto a G
    float umbral_movimiento_gyr = 5.0f; // > 5 deg/s de giro
    float umbral_choque = 25.0f; // > 25 m/s^2 (impacto o caída)
    
    if (accel_mag > umbral_choque) {
        // Estado de problema (Impacto detectado)
        // menv.state = PROBLEM; 
    } 
    else if (fabsf(accel_mag - 9.81f) > umbral_movimiento_acc || gyro_mag > umbral_movimiento_gyr) {
        // El dron está en movimiento / vuelo
        // menv.state = FLYING;
    } 
    else {
        // Valores cercanos al reposo
        // menv.state = IDLE;
    }
}

void IMU_measurement(void *p1, void *p2, void *p3) {
    const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);

    if (!device_is_ready(i2c_dev)) {
        printk("Error: El bus I2C no está listo.\n");
        return;
    }

    // PWR_MGMT_1: Despertar (SLEEP=0) y reloj a PLL Gyro X 
    // revisar pero pone en el sensor que es la mejor cosa que hacer
    uint8_t pwr_cmd[] = {PWR_MGMT_1_REG, 0x01}; 
    // en el write mandamos a la funcion (dispositivo, buffer, num bytes, direccion)
    i2c_write(i2c_dev, pwr_cmd, 2, MPU6050_ADDR);

    // Activa el filtro paso bajo a un ancho de banda de 
    uint8_t config_cmd[] = {CONFIG_REG, 0x03};
    i2c_write(i2c_dev, config_cmd, 2, MPU6050_ADDR);

    // SMPLRT_DIV: Muestreo a 50Hz -> 1kHz / (1 + 19) 
    // fórmula en la página 12, 1kHz de numerador ya que es la frecuencia del 
    // giroscopio al tener activado el paso bajo 
    uint8_t smplrt_cmd[] = {SMPLRT_DIV_REG, 19}; 
    i2c_write(i2c_dev, smplrt_cmd, 2, MPU6050_ADDR);

    // GYRO_CONFIG: Escala +-500 deg/s (FS_SEL=0)
    // los drones de seguimiento suelen tener entre 200 y 300 grados por segundo de rotacion 
    // para verlas con precision podemos usar el de 500 grados ya que no llegaria facilmente al maximo
    // y a la vez mantendriamos 65.5 LSB en cada grado por segundo 
    uint8_t gyro_cmd[] = {GYRO_CONFIG_REG, 0x08};
    i2c_write(i2c_dev, gyro_cmd, 2, MPU6050_ADDR);

    // revisar y hablar de las configuraciones y de la sensibilidad de cada uno 
    // ACCEL_CONFIG: Escala +-4g (AFS_SEL=0)
    // 8192 LSB/g esto quiere decir que cada g (fuerza de la gravedad)
    // tienen un rango de 8192
    uint8_t accel_cmd[] = {ACCEL_CONFIG_REG, 0x08};
    i2c_write(i2c_dev, accel_cmd, 2, MPU6050_ADDR);

    // Variables globales para los ángulos (asumiendo inicio en 0)
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    float dt = 0.02f; // 50 Hz = 20 ms

    uint8_t buf[14];
    uint8_t print_counter=0; 

    while (1) {
        
        if (i2c_burst_read(i2c_dev, MPU6050_ADDR, ACCEL_XOUT_H_REG, buf, 14) == 0) {
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

            // Integración numérica
            roll  += gx * dt;
            pitch += gy * dt;
            yaw   += gz * dt;

            actualizar_estado(ax, ay, az, gx, gy, gz);

            // Imprimir cada 10 muestras (cada 200 ms) para no saturar la UART y perder los 50 Hz
            if (++print_counter >= 10) {
                print_counter = 0;
                printk("ACC [m/s2]: X=%.2f Y=%.2f Z=%.2f | GYR [deg/s]: X=%.1f Y=%.1f Z=%.1f | ANG [deg]: R=%.1f P=%.1f Y=%.1f\n",
                       (double)ax, (double)ay, (double)az,
                       (double)gx, (double)gy, (double)gz,
                       (double)roll, (double)pitch, (double)yaw);
            }
        } else {
            printk("Error en lectura burst I2C\n");
        }

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
    1000                           /* Start delay */
);