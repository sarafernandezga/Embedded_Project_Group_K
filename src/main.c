#include "includes.h"
#include "tests.h"

K_SEM_DEFINE(sem_main_gate, 0, 1);

static void main_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    k_sem_give(&sem_main_gate);
}

K_TIMER_DEFINE(main_timer, main_timer_handler, NULL);


int main(void)
{

    k_timer_start(&main_timer, K_SECONDS(2), K_SECONDS(2));

    while (1) {
        k_sem_take(&sem_main_gate, K_FOREVER);
        test_enviroment();
        test_gps();
    }
}
