#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/adc.h>

#include "includes.h"
#include "enviroment.h"
#include "main.h"
#include "statistics.h"

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
K_MSGQ_DEFINE(env_stats_q, sizeof(struct env_stats), 1, 4); // Statistics for normal mode

static void enviroment_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    k_sem_give(&sem_env_gate);
}

K_TIMER_DEFINE(enviroment_timer, enviroment_timer_handler, NULL);


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

    uint8_t cmd = SI7021_READ_USER_REG;
    uint8_t reg;
    int ret;

    /* Initialize I2C device */
    if (!device_is_ready(i2c)) {
        return -ENODEV;
    }

    /* Read the user register: it also detects a missing sensor */
    ret = i2c_write_read(i2c, SI7021_ADDR, &cmd, 1, &reg, 1);

    if (ret < 0) {
        return ret;
    }

    /* Force 12-bit RH / 14-bit temperature (bits 7 and 0 = 0) and heater off (bit 2 = 0) */
    // it should be default but to be sure 
    if (reg & 0x85) {
        uint8_t wr[2] = { SI7021_WRITE_USER_REG, (uint8_t)(reg & ~0x85) };

        ret = i2c_write(i2c, wr, sizeof(wr), SI7021_ADDR);

        if (ret < 0) {
            return ret;
        }
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

    /* Send measure command and read two bytes + CRC in ONE I2C transaction */
    ret = i2c_write_read(i2c, SI7021_ADDR, &cmd, 1, data, sizeof(data));

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

    /* The datasheet allows values slightly outside 0-100 %: limit them */
    // page 21 datasheet formula 
    if (*humidity < 0.0f) {
        *humidity = 0.0f;
    } else if (*humidity > 100.0f) {
        *humidity = 100.0f;
    }

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

    /* Read the temperature measured with the previous humidity: 2 bytes, no CRC */
    ret = i2c_write_read(i2c, SI7021_ADDR, &cmd, 1, data, sizeof(data));

    if (ret < 0) {
        return ret;
    }

    /* Combine MSB and LSB */
    raw = ((uint16_t)data[0] << 8) | data[1];

    /* Convert to Celsius */
    *temperature = 175.72f * (float)raw / 65536.0f - 46.85f;

    return 0;
}

/* Computes the status, publishes the message and notifies main only
 * when the status changes (OK -> error or error -> OK).
 * tolerate = true: an isolated failure is hidden (the last valid values are kept)
 * until ENV_ERR_LIMIT failed cycles in a row. */
static void env_publish(int ret_adc, int ret_hum, int ret_temp,
                        struct env_m *data, bool tolerate)
{
    static bool last_fault = false;
    static uint8_t fail_streak = 0;

    /* The temperature is only read if the humidity read was OK, so it counts once */
    int n_fail = (ret_adc < 0) + (ret_hum < 0) +
                 ((ret_hum == 0 && ret_temp < 0) ? 1 : 0);

    /* The status must be computed BEFORE deciding if there is a fault */
    if (n_fail >= 2) {
        data->status = MULTIPLE_FAIL;
    } else if (ret_hum < 0) {
        data->status = SI7021_FAIL;
    } else if (ret_temp < 0) {
        data->status = TEMP_FAIL;
    } else if (ret_adc < 0) {
        data->status = LIGHT_FAIL;
    } else {
        data->status = ALL_OK;
    }

    if (data->status == ALL_OK) {
        fail_streak = 0;
    } else if (tolerate) {
        if (fail_streak < ENV_ERR_LIMIT) {
            fail_streak++;              /* saturates at ENV_ERR_LIMIT */
        }
        if (fail_streak < ENV_ERR_LIMIT) {
            data->status = ALL_OK;      /* isolated error: keep the last valid values */
        }
    }

    bool fault = (data->status != ALL_OK);

    /* Keep only the latest message (put first, then notify) */
    k_msgq_purge(&env_q);
    k_msgq_put(&env_q, data, K_NO_WAIT);

    /* Wake main only when the sensor goes healthy -> faulty or back */
    if (fault != last_fault) {
        last_fault = fault;
        k_event_post(&main_events, EVT_DRONE);
    }
}



void my_thread_enviroment(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    struct adc_sequence sequence = {
        .buffer = &sample,
        .buffer_size = sizeof(sample),
    };

    /* 30 s statistics */
    struct stat_acc light_stat, temp_stat, hum_stat;
    int64_t window_start = k_uptime_get();
    bool was_active = false;

    stat_reset(&light_stat);
    stat_reset(&temp_stat);
    stat_reset(&hum_stat);

    /* Init failed: report the error and retry every 2 s until both respond */
    for (;;) {
        int init_adc = initialize_adc(&sequence);
        int init_hum = initialize_Si7021();

        if (init_adc == 0 && init_hum == 0) {
            break;
        }

        struct env_m menv_err = {0};

        env_publish(init_adc, init_hum, init_hum, &menv_err, false);
        // maybe it can be change for the semaphore of the sensor
        k_sleep(K_SECONDS(2)); // this is only in case an error 
    }

    struct env_m last = {0};   /* last valid values */

    while (1) {
        k_sem_take(&sem_env_gate, K_FOREVER);

        struct env_m menv = last;  /* a failed value keeps the last valid one */

        int ret_adc  = take_LDC_measure(&sequence, &menv.light);
        int ret_hum  = measure_humidity(&menv.hum);
        int ret_temp = (ret_hum == 0) ? measure_temperature(&menv.temp) : ret_hum;

        env_publish(ret_adc, ret_hum, ret_temp, &menv, true);
        last = menv;
        int64_t now = k_uptime_get();
        bool active = normal_active;

        /* Just entered NORMAL mode: start a new 30 s window */
        if (active && !was_active) {
            stat_reset(&light_stat);
            stat_reset(&temp_stat);
            stat_reset(&hum_stat);
            window_start = now;
        }
        was_active = active;

        if (active) {
            /* Only good readings go into the statistics */
            if (ret_adc == 0)  { stat_add(&light_stat, menv.light); }
            if (ret_hum == 0)  { stat_add(&hum_stat,   menv.hum);   }
            if (ret_temp == 0) { stat_add(&temp_stat,  menv.temp);  }

            /* Close the 30 s window: publish the result and start again */
            if ((now - window_start) >= STATS_WINDOW_MS) {
                struct env_stats out;

                stat_result(&light_stat, &out.light);
                stat_result(&temp_stat,  &out.temp);
                stat_result(&hum_stat,   &out.hum);
                k_msgq_purge(&env_stats_q);
                k_msgq_put(&env_stats_q, &out, K_NO_WAIT);

                stat_reset(&light_stat);
                stat_reset(&temp_stat);
                stat_reset(&hum_stat);
                window_start = now;
            }
        }
    }
}

/* Start enviroment thread */
K_THREAD_DEFINE(
    my_thread_enviroment_id,    /* Thread identifier */
    1024,                        /* Stack size */
    my_thread_enviroment,       /* Entry function */
    NULL, NULL, NULL,           /* Parameters */
    4,                          /* Priority */
    0,                          /* Options */
    0                           /* Start delay */
);