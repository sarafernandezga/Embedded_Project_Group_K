#include "includes.h"
#include "enviroment.h"

/* Timer callback */
static void env_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    /* Wakes up thread to take a measurement */
    k_sem_give(&sem_env_gate);
}

K_TIMER_DEFINE(env_timer, env_timer_handler, NULL);
                
int main(void) {
    struct env_m  menv;

    while (1) {

        k_sem_give(&sem_env_gate);
        int ret = k_msgq_get(&env_q, &menv, K_FOREVER);

        if (ret == 0) {        

            switch (menv.status) {
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

        } else {
            printk("Error al recibir datos: %d\n", ret);
        }

        k_sleep(K_SECONDS(2));
    }
}