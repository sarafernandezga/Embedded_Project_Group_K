
#include "includes.h"
#include "enviroment.h"
#include "gps.h"

/* Timer callback */
static void env_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    /* Wakes up thread to take a measurement */
    k_sem_give(&sem_env_gate);
}

K_TIMER_DEFINE(env_timer, env_timer_handler, NULL);

int test_enviroment(void){
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
    return ret;

}

int test_gps(void){
    struct gps_data gps_data;
    int ret;


    /* Receive GPS data if available */
    ret = k_msgq_get(&gps_msgq, &gps_data, K_NO_WAIT);

    if (ret == 0) {
        printk("GPS: Latitud: %.6f | Longitud: %.6f | Altitud: %.2f m\n",
                (double)gps_data.latitude,
                (double)gps_data.longitude,
                (double)gps_data.altitude);

        printk("GPS: Satelites: %u | HDOP: %.2f | Fix: %u\n",
                (unsigned int)gps_data.satellites,
                (double)gps_data.hdop,
                (unsigned int)gps_data.fix_quality);
    }
    return ret;
}



int main(void)
{
    int ret;
    while (1) {

        // ret = test_enviroment();
        ret = test_gps();

        k_sleep(K_SECONDS(2));
    }
}


