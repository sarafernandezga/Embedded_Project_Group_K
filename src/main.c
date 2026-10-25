#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

/* Thread function */
void my_thread_func(void *p1, void *p2, void *p3) {
    while (1) {
        printk("Hello from my_thread!\n");
        k_sleep(K_SECONDS(1));
    }
}
/* Define the thread at compile-time */
K_THREAD_DEFINE(my_thread_id, /* Thread identifier */
                1024, /* Stack size */
                my_thread_func, /* Entry function */
                NULL, NULL, NULL, /* Parameters */
                5, /* Priority */
                0, /* Options */
                0); /* Starts immediately */
                
int main(void) {
    printk("Zephyr main thread started\n");
    while (1) {
        printk("Hello from main thread!\n");
        k_sleep(K_SECONDS(2));
    }
}