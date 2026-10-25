#include "../includes/includes.h"
#include "../sensors/enviroment.h"

/* ADC Variables */
#define ADC_NODE DT_PATH(zephyr_user)

static const struct adc_dt_spec adc_ch =
    ADC_DT_SPEC_GET_BY_IDX(ADC_NODE, 0);

static int16_t sample;

K_SEM_DEFINE(sem_env_gate, 0, 1);

/* Message Queue */
K_MSGQ_DEFINE_TYPE(env_q, struct env_m , 1);

/* Timer callback */
static void env_timer_handler(struct k_timer *timer)
{
    ARG_UNUSED(timer);

    /* Wakes up thread to take a measurement */
    k_sem_give(&sem_env_gate);
}

K_TIMER_DEFINE(env_timer, env_timer_handler, NULL);

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
/* Reads the ADC value and sends the result to the message queue */
static int take_LDC_measure(struct adc_sequence *sequence){
    /* Take raw measurement from ADC */
    int ret = adc_read_dt(&adc_ch, sequence);

    if (ret != 0) {
        printk("ADC: Error reading: %d\n", ret);
        return ret;
    }

    /* Convert the ADC value to a percentage */
    uint32_t light = 100 - ((int)sample * 100 / 4095);

    return light;
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



// ============================================================
// Thread
// ============================================================
void my_thread_enviroment(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    uint32_t light;
    uint32_t temp = 0;
    uint32_t hum = 0;
    int ret;

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

    /* Initialize measures */
    set_state_adc(false);

    while (1) {
        /* Wait for a measurement request */
        k_sem_take(&sem_env_gate, K_FOREVER);

        /* Take measurements from sensors */
        light = take_LDC_measure(&sequence);
        //take_temp_hum_measure();

        /* Update message */
        struct env_m menv = {
            .temp = temp,
            .hum = hum,
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