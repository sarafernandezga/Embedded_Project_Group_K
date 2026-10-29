#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/adc.h>

#include "includes.h"
#include "enviroment.h"

/* ------ ADC ------*/
#define ADC_NODE DT_PATH(zephyr_user)
static const struct adc_dt_spec adc_ch = ADC_DT_SPEC_GET_BY_IDX(ADC_NODE, 0);

static int16_t sample;

/* ------ I2C ------*/
#define I2C_NODE DT_ALIAS(si7021)
static const struct device *i2c = DEVICE_DT_GET(I2C_NODE);

/* ----- Thread ----- */
K_SEM_DEFINE(sem_env_gate, 0, 1);                    // Semaphore definition
K_MSGQ_DEFINE(env_q, sizeof(struct env_m), 1, 4);    // Message queue definition

// ============================================================
// Light monitoring (LDC)
// ============================================================

/* ADC Initialization */
static int initialize_adc(struct adc_sequence *sequence)
{
    /* Variables */
    int ret;

    /* Check if the ADC device is ready */
    if (!adc_is_ready_dt(&adc_ch)) {
        return -ENODEV;
    }

    /* Configure the ADC channel */
    ret = adc_channel_setup_dt(&adc_ch);

    if (ret < 0) {
        return ret;
    }

    /* Initialize the ADC sequence */
    ret = adc_sequence_init_dt(&adc_ch, sequence);

    if (ret < 0) {
        return ret;
    }

    return ret;
}

/* Reads the ADC value */
static int take_LDC_measure(struct adc_sequence *sequence,
                            float *light)
{
    /* Variables */
    int ret;
    float light_raw;

    /* Check light variable is not null*/
    if (light == NULL) {
        return -EINVAL;
    }

    /* Read raw data from ADC */
    ret = adc_read_dt(&adc_ch, sequence);

    if (ret < 0) {
        return ret;
    }

    /* Convert raw ADC value to percentage */
    light_raw = 100.0f - ((float)sample * 100.0f / 4095.0f);

    /* Calibrate light data manually */
    *light = (light_raw - MIN_CALIBRATE_LCD) * 100.0f / (MAX_CALIBRATE_LCD - MIN_CALIBRATE_LCD);

    /* Limitate range between 0-100 % */
    if (*light < 0.0f) {
        *light = 0.0f;
    } else if (*light > 100.0f) {
        *light = 100.0f;
    }

    return 0;
}

// ============================================================
// Temperature and Humidity (Si7021)
// ============================================================

static int initialize_Si7021(void){

    /* Initialize I2C device */
    if (!device_is_ready(i2c)) {
        return -ENODEV;
    }

    return 0;
}

static uint8_t si7021_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00; // Initialize CRC

    // Iterate over data bytes
    for (uint8_t i = 0; i < len; i++) { 
        crc = crc ^ data[i]; // XOR CRC with current byte

        // Process each bit
        for (uint8_t bit = 0; bit < 8; bit++) { 
            
            // Check most significant bit
            if (crc & 0x80) {                       
                crc = (crc << 1) ^ CRC_POLINOMIAL;  // Shift and XOR polynomial
            } else {
                crc <<= 1;                          // Shift left one bit
            }
        }
    }

    return crc;
}


static int measure_humidity(float *humidity)
{
    /* Variables */
    uint8_t cmd = SI7021_MEASURE_RH;
    uint8_t data[3];
    uint16_t raw;
    int ret;

    /* Check humidity variable is not null */
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

    /* Check if CRC matches */
    if (data[2] != si7021_crc8(data, 2)) {
        return -EIO;
    }

    /* Combine MSB and LSB */
    raw = ((uint16_t)data[0] << 8) | data[1];

    /* Convert to relative humidity (%) */
    *humidity = 125.0f * (float)raw / 65536.0f - 6.0f;

    return 0;
}


static int measure_temperature(float *temperature)
{
    /* Variables */
    uint8_t cmd = SI7021_READ_TEMP_FROM_RH;
    uint8_t data[2];
    uint16_t raw;
    int ret;

    /* Check temperature variable is not null */
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

    /* Data variables */
    float light = 0.0f;
    float humidity = 0.0f;
    float temperature = 0.0f;

    /* Error variables */
    int ret_ADC;
    int ret_si7021;
    int ret_temp;
    int ret_queue;

    /* Status variables */
    enum env_status env_status = ALL_OK;

    /* ADC struct */
    struct adc_sequence sequence = {
        .buffer = &sample,
        .buffer_size = sizeof(sample),
    };


    /* Initialize ADC */
    ret_ADC = initialize_adc(&sequence);

    if (ret_ADC < 0) { env_status = ADC_INIT_FAIL; }

    /* Initialize Si7021 */
    ret_si7021 = initialize_Si7021();

    if (ret_si7021 < 0) {
        env_status = (env_status == ADC_INIT_FAIL)
                ? MULTIPLE_FAIL
                : SI7021_INIT_FAIL;
    }

    while (1) {
        /* Wait for an instruction to measure */
        k_sem_take(&sem_env_gate, K_FOREVER);

        /* Reset values before each measurement */
        light = 0.0f;
        humidity = 0.0f;
        temperature = 0.0f;

        /* Read light */
        ret_ADC = take_LDC_measure(&sequence, &light);

        /* Read humidity */
        ret_si7021 = measure_humidity(&humidity);

        /* Read associated temperature only if humidity succeeded */
        if (ret_si7021 == 0) {
            ret_temp = measure_temperature(&temperature);

        } else {
            ret_temp = ret_si7021;
        }

        /* Determine measurement status */
        if (ret_ADC < 0 && ret_si7021 < 0) {
            env_status = MULTIPLE_FAIL;
        } else if (ret_si7021 < 0) {
            env_status = SI7021_FAIL;
        } else if (ret_temp < 0) {
            env_status = TEMP_FAIL;
        } else if (ret_ADC < 0) {
            env_status = LIGHT_FAIL;
        } else {
            env_status = ALL_OK;
        }

        /* Construct message */
        struct env_m menv = {
            .temp = temperature,
            .hum = humidity,
            .light = light,
            .status = env_status
        };

        /* Send message with data and status */
        ret_queue = k_msgq_put(&env_q, &menv, K_NO_WAIT);

    }

}

/* Start enviroment thread */
K_THREAD_DEFINE(
    my_thread_enviroment_id,    /* Thread identifier */
    512,                        /* Stack size */
    my_thread_enviroment,       /* Entry function */
    NULL, NULL, NULL,           /* Parameters */
    4,                          /* Priority */
    0,                          /* Options */
    0                           /* Start delay */
);