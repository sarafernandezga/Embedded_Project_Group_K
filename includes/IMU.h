#ifndef IMU_H_
#define IMU_H_

// REGISTROS
#define MPU6050_ADDR         0x68 // Dirección I2C
#define MPU6050_SMPLRT_DIV_REG   0x19 // donde podemos cambiar la frecuencia del giroscopio
#define MPU6050_CONFIG_REG       0x1A // sincronización de trama y DLPF
#define MPU6050_GYRO_CONFIG_REG  0x1B // define la escala del giroscopio mediante FS_SEL
#define MPU6050_ACCEL_CONFIG_REG 0x1C // rango del acelerometro mediante AFS_SEL
#define MPU6050_PWR_MGMT_1_REG   0x6B // si usar o no el sensor de temperatura integrado
                                      // lo vamos a desactivar ya que es redundante
#define MPU6050_ACCEL_XOUT_H_REG 0x3B // Primer registro de datos (Acelerómetro X byte alto)

// Estados posibles del dron (Requisito SR7)
enum drone_state { 
    IDLE = 0,
    FLYING,
    PROBLEM
};

// Estructura del mensaje compartido
struct imu_m {
    float ax, ay, az;          // Aceleración [m/s^2]
    float gx, gy, gz;          // Giroscopio [deg/s]
    float roll, pitch, yaw;    // Ángulos [deg]
    float vx, vy, vz;          // Velocidades [m/s]
    float x, y, z;             // Posición local [m]
    enum drone_state state;    // Estado (IDLE, FLYING, PROBLEM)
    int status;                // 0 = OK, -1 = Error I2C
};

// Declaración externa de la cola de mensajes (para que acceda main.c)
extern struct k_msgq imu_q;

#endif 