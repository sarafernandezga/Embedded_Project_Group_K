#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

#define I2C_NODE DT_NODELABEL(i2c2) 

#define MPU6050_ADDR    0x68    // Dirección I2C
#define WHO_AM_I_REG    0x75    // Registro WHO_AM_I
#define PWR_MGMT_1_REG  0x6B    // Registro PWR_MGMT_1

void who_am_i_funct(void)
{
    const struct device *i2c_dev = DEVICE_DT_GET(I2C_NODE);

    if (!device_is_ready(i2c_dev)) {
        printk("Error: El bus I2C no está listo.\n");
        return;
    }

    uint8_t who_am_i = 0;
    int ret = i2c_reg_read_byte(i2c_dev, MPU6050_ADDR, WHO_AM_I_REG, &who_am_i);
    
    if (ret == 0) {
        printk("WHO_AM_I = 0x%02X\n", who_am_i);
    } else {
        printk("Error de I2C al leer WHO_AM_I\n");
    }

    // Bucle infinito para que el hilo no muera y ceda el control
    while (1) {
        // Aquí irán las futuras lecturas de aceleración y giroscopio
        //printk("Hilo del acelerómetro activo...\n");
        
        k_msleep(2000); 
    }
}

/* Start enviroment thread */
K_THREAD_DEFINE(
    who_am_i_funct_id,    /* Thread identifier */
    1024,                       /* Stack size */
    who_am_i_funct,       /* Entry function */
    NULL, NULL, NULL,           /* Parameters */
    4,                          /* Priority */
    0,                          /* Options */
    1000                           /* Start delay */
);