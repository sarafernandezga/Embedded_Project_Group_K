#ifndef STATS_H
#define STATS_H

#include <stdint.h>
#include <float.h>
#include <zephyr/kernel.h>

#define STATS_WINDOW_MS 30000

/* Accumulator used inside a sensor thread */
struct stat_acc {
    float sum;
    float min;
    float max;
    uint32_t n;
};

/* Result of one 30 s window */
struct stat_f {
    float mean;
    float min;
    float max;
    uint32_t n;       /* 0 = no valid samples in the window */
};

struct imu_stats {
    struct stat_f acc;    /* |acceleration| [m/s2] */
    struct stat_f gyr;    /* |angular velocity| [deg/s] */
};

struct env_stats {
    struct stat_f light;
    struct stat_f temp;
    struct stat_f hum;
};

extern struct k_msgq imu_stats_q;
extern struct k_msgq env_stats_q;

static inline void stat_reset(struct stat_acc *s)
{
    s->sum = 0.0f;
    s->min = FLT_MAX;
    s->max = -FLT_MAX;
    s->n = 0;
}

static inline void stat_add(struct stat_acc *s, float v)
{
    s->sum += v;
    if (v < s->min) { s->min = v; }
    if (v > s->max) { s->max = v; }
    s->n++;
}

static inline void stat_result(const struct stat_acc *s, struct stat_f *out)
{
    out->n = s->n;

    if (s->n == 0) {
        out->mean = 0.0f;
        out->min = 0.0f;
        out->max = 0.0f;
        return;
    }

    out->mean = s->sum / (float)s->n;
    out->min = s->min;
    out->max = s->max;
}

extern volatile bool normal_active;    /* true while in NORMAL mode (written by main) */

#endif