#include "includes.h"
#include "enviroment.h"
#include "gps.h"
#include "IMU.h"
#include "tests.h"

void test_enviroment(void){
    struct env_m menv;
    
    /* Request environmental measurements */
    k_sem_give(&sem_env_gate);

    /* Receive environmental data if available */
    int ret = k_msgq_get(&env_q, &menv, K_NO_WAIT);

    if (ret == 0)
    {

        switch (menv.status)
        {
        case ALL_OK:
            printk("Luz: %.2f %% | Humedad: %.2f %% | Temperatura: %.2f C\n",
                    (double)menv.light,
                    (double)menv.hum,
                    (double)menv.temp);
            break;

        case TEMP_FAIL:
            printk("Error al medir temperatura\n");
            break;

        case LIGHT_FAIL:
            printk("Error al medir luz\n");
            break;

        case SI7021_FAIL:
            printk("Fallo del sensor Si7021\n");
            break;

        case MULTIPLE_FAIL:
            printk("Multiples sensores o medidas fallidas\n");
            break;

        case ADC_INIT_FAIL:
            printk("ERROR CRITICO: no se pudo inicializar el ADC\n");
            break;

        case SI7021_INIT_FAIL:
            printk("ERROR CRITICO: no se pudo inicializar el Si7021/I2C\n");
            break;
        }
    }

}


void test_gps(void){
    struct gps_data gps_data;
    int ret;

    /* Receive GPS data if available */
    ret = k_msgq_get(&gps_msgq, &gps_data, K_NO_WAIT);

    if (ret == 0)
    {

        switch (gps_data.status)
        {
        case ALL_OK:
             printk("GPS: Hora UTC: %s\n", gps_data.utc_time);
            
            printk("GPS: Latitud: %.6f | Longitud: %.6f | Altitud: %.2f m\n",
                    (double)gps_data.latitude,
                    (double)gps_data.longitude,
                    (double)gps_data.altitude);

            printk("GPS: Satelites: %u | HDOP: %.2f | Fix: %u\n",
                    (unsigned int)gps_data.satellites,
                    (double)gps_data.hdop,
                    (unsigned int)gps_data.fix_quality);
            break;

        case GPS_ERR_NO_DATA:
            printk("Error fisico, no llegan datos\n");
            break;

        case GPS_ERR_CHECKSUM:
            printk("Error en la comunicacion\n");
            break;

        case GPS_NO_FIX:
            printk("Warning: Datos no validos\n");
            break;
        }
    }

}


void test_IMU(void){
    struct imu_m mimu;
    int ret;

    /* Receive GPS data if available */
    ret = k_msgq_get(&imu_q, &mimu, K_NO_WAIT);

    if (ret == 0)
    {

        switch (mimu.status)
        {
        case 0:
            printk("ACC [m/s2]: X=%.2f Y=%.2f Z=%.2f | GYR [deg/s]: X=%.1f Y=%.1f Z=%.1f | ANG [deg]: R=%.1f P=%.1f Y=%.1f\n",
                       (double)mimu.ax, (double)mimu.ay, (double)mimu.az,
                       (double)mimu.gx, (double)mimu.gy, (double)mimu.gz,
                       (double)mimu.roll, (double)mimu.pitch, (double)mimu.yaw);
            break;

        case -1:
            printk("Error fisico, no llegan datos\n");
            break;
        }
    }

}

// K_SEM_DEFINE(sem_main_gate, 0, 1);

struct system_messages {
    struct imu_m    imu;
    struct gps_data gps;
    struct env_m    env;
    bool imu_valid;
    bool gps_valid;
    bool env_valid;
};

static struct system_messages g_state;

static const char *state_to_str(enum drone_state s)
{
    switch (s) {
    case IDLE:    return "IDLE";
    case FLYING:  return "FLYING";
    case PROBLEM: return "PROBLEM";
    default:      return "UNKNOWN";
    }
}

static void queue_update(void)
{
    struct imu_m    imu_tmp;
    struct gps_data gps_tmp;
    struct env_m    env_tmp;

    if (k_msgq_get(&imu_q, &imu_tmp, K_NO_WAIT) == 0) {
        g_state.imu = imu_tmp;
        g_state.imu_valid = (imu_tmp.status == 0);
    }
    if (k_msgq_get(&gps_msgq, &gps_tmp, K_NO_WAIT) == 0) {
        g_state.gps = gps_tmp;
        g_state.gps_valid = ((int)gps_tmp.status == (int)ALL_OK);
    }
    if (k_msgq_get(&env_q, &env_tmp, K_NO_WAIT) == 0) {
        g_state.env = env_tmp;
        g_state.env_valid = ((int)env_tmp.status == (int)ALL_OK);
    }
}

static void print_test_report(void)
{
    printk("MODE: TEST\n");

    if (g_state.gps_valid) {
        printk("GPS: fix=%u sats=%02u lat=%.6f lon=%.6f alt=%.1f m time=%s\n",
               (unsigned)g_state.gps.fix_quality, (unsigned)g_state.gps.satellites,
               (double)g_state.gps.latitude, (double)g_state.gps.longitude,
               (double)g_state.gps.altitude, g_state.gps.utc_time);
    } else {
        printk("GPS: sin fix\n");
    }

    if (g_state.imu_valid) {
        printk("IMU: ax=%.2f ay=%.2f az=%.2f m/s2 | gx=%.1f gy=%.1f gz=%.1f deg/s\n",
               (double)g_state.imu.ax, (double)g_state.imu.ay, (double)g_state.imu.az,
               (double)g_state.imu.gx, (double)g_state.imu.gy, (double)g_state.imu.gz);
        printk("ATT: roll=%.2f pitch=%.2f yaw=%.2f deg\n",
               (double)g_state.imu.roll, (double)g_state.imu.pitch, (double)g_state.imu.yaw);
    } else {
        printk("IMU: error\n");
    }

    if (g_state.env_valid) {
        printk("LIGHT: %.1f %% | ENV: temp=%.1f C hum=%.1f %%\n",
               (double)g_state.env.light,
               (double)g_state.env.temp,
               (double)g_state.env.hum);
    } else {
        printk("ENV: error\n");
    }

    printk("STATUS: %s\n\n", state_to_str(g_state.imu.state));
}

void run_test_mode(void)
{
    k_sem_give(&sem_env_gate);
    k_msleep(30);
    queue_update();
    print_test_report();
}