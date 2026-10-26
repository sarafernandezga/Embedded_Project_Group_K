#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/adc.h>
#include <errno.h>
#include <stdint.h>
#include <stddef.h>

#include "enviroment.h"

/* ADC Variables */
#define ADC_NODE DT_PATH(zephyr_user)

static const struct adc_dt_spec adc_ch =
    ADC_DT_SPEC_GET_BY_IDX(ADC_NODE, 0);

static int16_t sample;

K_SEM_DEFINE(sem_env_gate, 0, 1);

/* Message Queue */
K_MSGQ_DEFINE(env_q, sizeof(struct env_m), 1, 4);

/* Timer callback */
static void env_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    /* Wakes up thread to take a measurement */
    k_sem_give(&sem_env_gate);
}

K_TIMER_DEFINE(env_timer, env_timer_handler, NULL);


/* ------ I2C ------*/
#define I2C_NODE DT_ALIAS(si7021) // Set it up in the device tree file

static const struct device *i2c = DEVICE_DT_GET(I2C_NODE);

// ============================================================
// Light monitoring (LDC)
// ============================================================

/* ADC Initialization */
static int initialize_adc(struct adc_sequence *sequence)
{
    int ret;

    /* Check if the ADC device is ready */
    if (!adc_is_ready_dt(&adc_ch)) {
        printk("ADC: device not prepared\n");
        return -ENODEV;
    }

    /* Configure the ADC channel */
    ret = adc_channel_setup_dt(&adc_ch);
    if (ret < 0) {
        printk("ADC: configuration error: %d\n", ret);
        return ret;
    }

    /* Initialize the ADC sequence */
    ret = adc_sequence_init_dt(&adc_ch, sequence);
    if (ret < 0) {
        printk("ADC: error initializing sequence: %d\n", ret);
        return ret;
    }

    return ret;
}

/* ADC Measurement */
/* Reads the ADC value */
static int take_LDC_measure(struct adc_sequence *sequence,
                            float *light)
{
    int ret;
    float light_raw;

    if (light == NULL) {
        return -EINVAL;
    }

    ret = adc_read_dt(&adc_ch, sequence);

    if (ret < 0) {
        printk("ADC: Error reading: %d\n", ret);
        return ret;
    }

    /* Convert raw ADC value to percentage */
    light_raw = 100.0f - ((float)sample * 100.0f / 4095.0f);

    /* Calibrate */
    *light = (light_raw - 25.0f) * 100.0f / (85.0f - 25.0f);

    /* Limitate range between 0-100 % */
    if (*light < 0.0f) {
        *light = 0.0f;
    } else if (*light > 100.0f) {
        *light = 100.0f;
    }

    return 0;
}

/* ADC Pause Control */
void set_state_adc(bool pause)
{

    if (pause) {                 // Activate pause mode
        /* Stop measurement timer */
        printk("ADC paused\n");
        k_timer_stop(&env_timer);

    } else {                    // Resume measurementes  
        /* Take a measure and restart timer */
        k_sem_give(&sem_env_gate);
        k_timer_start(&env_timer, K_SECONDS(2), K_SECONDS(2));

    }
}

// ============================================================
// Temperature and Humidity (Si7021)
// ============================================================

static int initialize_Si7021(void){
    if (!device_is_ready(i2c)) {
        return -ENODEV;
    }

    return 0;

}


static int measure_humidity(float *humidity)
{
    uint8_t cmd = SI7021_MEASURE_RH;
    uint8_t data[3];
    uint16_t raw;
    int ret;

    if (humidity == NULL) {
        return -EINVAL;
    }

    /* Send measure command */
    ret = i2c_write(i2c, &cmd, 1, SI7021_ADDR);
    if (ret < 0) {
        return ret;
    }

    /* Read two bytes and CRC */
    ret = i2c_read(i2c, data, sizeof(data), SI7021_ADDR);
    if (ret < 0) {
        return ret;
    }

    /* Check CRC */
    // IMPORTANTE: Comprobar el CRC 
    //if (data[2] != si7021_crc8(data, 2)) {
      //  return -EIO;
    //}

    /* Combine MSB and LSB */
    raw = ((uint16_t)data[0] << 8) | data[1];

    /* Convert to relative humidity */
    *humidity = 125.0f * (float)raw / 65536.0f - 6.0f;

    return 0;
}


static int measure_temperature(float *temperature)
{
    uint8_t cmd = SI7021_READ_TEMP_FROM_RH;
    uint8_t data[2];
    uint16_t raw;
    int ret;

    if (temperature == NULL) {
        return -EINVAL;
    }

    /* Send command to read previous temperature */
    ret = i2c_write(i2c, &cmd, 1, SI7021_ADDR);
    if (ret < 0) {
        return ret;
    }

    /* Read temperature: only 2 bytes, no CRC */
    ret = i2c_read(i2c, data, sizeof(data), SI7021_ADDR);
    if (ret < 0) {
        return ret;
    }

    /* Combine MSB and LSB */
    raw = ((uint16_t)data[0] << 8) | data[1];

    /* Convert to Celsius */
    *temperature = 175.72f * (float)raw / 65536.0f - 46.85f;

    return 0;
}

// ============================================================
// Thread
// ============================================================
void my_thread_enviroment(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    int ret;

    float light;
    float temperature;
    float humidity;

     /* Variables */
    struct adc_sequence sequence = {
        .buffer = &sample,
        .buffer_size = sizeof(sample),
    };

    /* Initialize ADC */
    ret = initialize_adc(&sequence);

    if (ret < 0) {
        printk("ADC: error de inicializacion: %d\n", ret);
        return;
    }

    /* Initialize Si0721 */
    ret = initialize_Si7021();

    if (ret < 0) {
        printk("Error inicializando I2C/Si7021: %d\n", ret);
        return;
    }

    /* Initialize measures */
    set_state_adc(false);

    while (1) {
        /* Wait for a measurement request */
        k_sem_take(&sem_env_gate, K_FOREVER);

        /* Take measurements from sensors */
        ret = take_LDC_measure(&sequence, &light);
        if (ret < 0) {
            printk("Error en luz: %d\n", ret);
            continue;
        }

        ret = measure_humidity(&humidity);
        if (ret < 0) {
            printk("Error en humedad: %d\n", ret);
            continue;
        }

        ret = measure_temperature(&temperature);
        if (ret < 0) {
            printk("Error en temperatura: %d\n", ret);
            continue;
        }


        /* Update message */
        struct env_m menv = {
            .temp = temperature,
            .hum = humidity,
            .light = light
        };
        
        /* Send the measurements to the message queue */
        k_msgq_put(&env_q, &menv, K_NO_WAIT);

    }
}

/* Start enviroment thread */
K_THREAD_DEFINE(
    my_thread_enviroment_id,    /* Thread identifier */
    1024,                       /* Stack size */
    my_thread_enviroment,       /* Entry function */
    NULL, NULL, NULL,           /* Parameters */
    4,                          /* Priority */
    0,                          /* Options */
    0                           /* Start delay */
);