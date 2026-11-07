#include "includes.h"
#include "enviroment.h"
#include "gps.h"
#include "IMU.h"

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
