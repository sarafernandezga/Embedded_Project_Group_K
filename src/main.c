#include "includes.h"
#include "enviroment.h"
                
int main(void) {
    struct env_m  menv;

    while (1) {

        k_sem_give(&sem_env_gate);
        int ret = k_msgq_get(&env_q, &menv, K_FOREVER);

        if (ret == 0) {
            printk("Luz: %.2f %% | Humedad: %.2f %% | Temperatura: %.2f C\n",
                (double)menv.light,
                (double)menv.hum,
                (double)menv.temp);
        } else {
            printk("Error al recibir datos: %d\n", ret);
        }

        k_sleep(K_SECONDS(2));
    }
}