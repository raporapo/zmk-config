/*
 * TMAG5273 magnetic scroll ring for ZMK.
 *
 * A TMAG5273 3D Hall sensor sits under a ring of axially magnetised magnets with alternating
 * poles. Seen from the sensor, the field vector (tangential axis vs. Z) turns a full 360 deg for
 * every pole pair that passes, so each sample gives the ring position within one pole pair
 * directly. The driver unwraps that angle and reports one REL_WHEEL step for every
 * 360 * pole-pairs / steps-per-rev degrees of field angle, with hysteresis, so a ring resting on a
 * step boundary cannot chatter and a missed sample cannot lose a step.
 *
 * Power: between samples the sensor sleeps (5 nA). Each sample wakes it (INT pulse, or an I2C
 * address if INT is not wired), triggers one conversion, reads both axes with a 1-byte read
 * command and puts it back to sleep. The sample rate drops in steps when the ring is still.
 */

#define DT_DRV_COMPAT raporapo_tmag5273_ring

#include <math.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(tmag5273_ring, CONFIG_TMAG5273_RING_LOG_LEVEL);

#define REG_DEVICE_CONFIG_1 0x00
#define REG_DEVICE_CONFIG_2 0x01
#define REG_SENSOR_CONFIG_1 0x02
#define REG_SENSOR_CONFIG_2 0x03
#define REG_MANUFACTURER_ID_LSB 0x0E
#define TRIGGER_BIT BIT(7)

#define MODE_STANDBY 0x00 /* OPERATING_MODE: standby, conversion on trigger */
#define MODE_SLEEP 0x01
#define I2C_RD_STANDARD 0x00
#define I2C_RD_1BYTE_16BIT 0x01 /* reads <axis1 MSB, LSB, axis2 MSB, LSB, CONV_STATUS> */
#define MAG_CH_ZX 0x5           /* X and Z, sent as X then Z */
#define MAG_CH_YZ 0x6           /* Y and Z, sent as Y then Z */
#define CONV_STATUS_RESULT BIT(0)
#define MANUFACTURER_ID 0x5449 /* "TI" */

#define T_START_SLEEP_US 60   /* sleep -> standby, 50 us typ */
#define MIN_FIELD_LSB 400     /* ~0.5 mT at +/-40 mT: no ring / magnets too far */
#define MOTION_DEG 2.0f       /* field-angle change that counts as the ring being touched */
#define MAX_JUMP_DEG 100.0f   /* larger jumps between samples are treated as a resync */

/* conversion time for two axes, per CONV_AVG setting (datasheet table 6-4: 13.3k .. 0.6k SPS) */
static const uint16_t conv_us[] = {75, 125, 227, 417, 833, 1667};

struct tmag_ring_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec int_gpio;
    uint16_t pole_pairs;
    uint16_t steps_per_rev;
    uint8_t hysteresis_percent;
    uint8_t tangential_axis;
    uint16_t tangential_gain_percent;
    uint8_t conv_avg;
    uint16_t active_ms;
    uint16_t idle_ms;
    uint16_t rest_ms;
    uint32_t idle_after_ms;
    uint32_t rest_after_ms;
    bool invert;
    uint16_t input_code;
};

struct tmag_ring_data {
    const struct device *dev;
    struct k_work_delayable work;
    bool configured;
    bool have_prev;
    float prev_deg;
    float acc_deg;
    int64_t last_motion;
    uint32_t errors;
    uint32_t samples;
};

K_THREAD_STACK_DEFINE(tmag_ring_stack, CONFIG_TMAG5273_RING_THREAD_STACK_SIZE);
static struct k_work_q tmag_ring_q;
static bool tmag_ring_q_started;

static void tmag_wake(const struct device *dev) {
    const struct tmag_ring_config *cfg = dev->config;

    if (cfg->int_gpio.port != NULL) {
        gpio_pin_set_dt(&cfg->int_gpio, 1); /* active low: pull INT down */
        k_busy_wait(10);
        gpio_pin_set_dt(&cfg->int_gpio, 0); /* release, the pull-up takes it high */
    } else {
        /* Any address on the bus wakes it; asleep it does not ACK, so this "fails" by design. */
        uint8_t reg = REG_MANUFACTURER_ID_LSB;
        (void)i2c_write_dt(&cfg->i2c, &reg, 1);
    }
    k_busy_wait(T_START_SLEEP_US);
}

static int tmag_sleep(const struct device *dev) {
    const struct tmag_ring_config *cfg = dev->config;

    return i2c_reg_write_byte_dt(&cfg->i2c, REG_DEVICE_CONFIG_2, MODE_SLEEP);
}

static int tmag_configure(const struct device *dev) {
    const struct tmag_ring_config *cfg = dev->config;
    uint8_t id[2];
    int ret;

    tmag_wake(dev);
    /* back to standard reads first: the sensor keeps its settings across an MCU reset */
    ret = i2c_reg_write_byte_dt(&cfg->i2c, REG_DEVICE_CONFIG_1, I2C_RD_STANDARD);
    if (ret == 0) {
        ret = i2c_burst_read_dt(&cfg->i2c, REG_MANUFACTURER_ID_LSB, id, sizeof(id));
    }
    if (ret != 0) {
        return ret;
    }
    if (sys_get_le16(id) != MANUFACTURER_ID) {
        LOG_ERR("unexpected manufacturer id 0x%04x", sys_get_le16(id));
        return -ENODEV;
    }
    uint8_t ch = cfg->tangential_axis == 0 ? MAG_CH_ZX : MAG_CH_YZ;
    ret = i2c_reg_write_byte_dt(&cfg->i2c, REG_SENSOR_CONFIG_1, ch << 4);
    /* +/-40 mT on all axes, no on-chip angle: the MCU normalises and unwraps the angle itself */
    ret = ret ?: i2c_reg_write_byte_dt(&cfg->i2c, REG_SENSOR_CONFIG_2, 0x00);
    ret = ret ?: i2c_reg_write_byte_dt(&cfg->i2c, REG_DEVICE_CONFIG_1,
                                       (cfg->conv_avg << 2) | I2C_RD_1BYTE_16BIT);
    ret = ret ?: tmag_sleep(dev);
    if (ret == 0) {
        LOG_INF("TMAG5273 ready at 0x%02x: %u pole pairs, %u steps/rev", cfg->i2c.addr,
                cfg->pole_pairs, cfg->steps_per_rev);
    }
    return ret;
}

/* One conversion: tangential axis and Z, in LSB. */
static int tmag_sample(const struct device *dev, int16_t *tan, int16_t *z) {
    const struct tmag_ring_config *cfg = dev->config;
    uint8_t cmd[2] = {TRIGGER_BIT | REG_DEVICE_CONFIG_2, MODE_STANDBY};
    uint8_t buf[5];
    int ret;

    tmag_wake(dev);
    ret = i2c_write_dt(&cfg->i2c, cmd, sizeof(cmd));
    if (ret != 0) {
        return ret;
    }
    k_usleep(conv_us[cfg->conv_avg] + 15);
    ret = i2c_read_dt(&cfg->i2c, buf, sizeof(buf));
    (void)tmag_sleep(dev);
    if (ret != 0) {
        return ret;
    }
    if ((buf[4] & CONV_STATUS_RESULT) == 0) {
        return -EAGAIN;
    }
    *tan = (int16_t)sys_get_be16(&buf[0]);
    *z = (int16_t)sys_get_be16(&buf[2]);
    return 0;
}

static uint32_t next_interval(const struct device *dev) {
    const struct tmag_ring_config *cfg = dev->config;
    struct tmag_ring_data *data = dev->data;
    int64_t still = k_uptime_get() - data->last_motion;

    if (still < cfg->idle_after_ms) {
        return cfg->active_ms;
    }
    return still < cfg->rest_after_ms ? cfg->idle_ms : cfg->rest_ms;
}

static void tmag_ring_work(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct tmag_ring_data *data = CONTAINER_OF(dwork, struct tmag_ring_data, work);
    const struct device *dev = data->dev;
    const struct tmag_ring_config *cfg = dev->config;
    int16_t tan, z;
    int ret;

    if (!data->configured) {
        ret = tmag_configure(dev);
        if (ret != 0) {
            if (data->errors++ % 10 == 0) {
                LOG_WRN("TMAG5273 not answering (%d), retrying", ret);
            }
            k_work_schedule_for_queue(&tmag_ring_q, &data->work, K_SECONDS(1));
            return;
        }
        data->configured = true;
        data->have_prev = false;
        data->last_motion = k_uptime_get();
    }

    ret = tmag_sample(dev, &tan, &z);
    if (ret != 0) {
        if (data->errors++ % 100 == 0) {
            LOG_WRN("sample failed (%d)", ret);
        }
        if (ret != -EAGAIN) {
            data->configured = false; /* re-probe: wire loose or sensor power-cycled */
        }
        k_work_schedule_for_queue(&tmag_ring_q, &data->work, K_MSEC(cfg->rest_ms));
        return;
    }

    float t = (float)tan * cfg->tangential_gain_percent / 100.0f;
    float zf = (float)z;
    bool motion = false;

    if (fabsf(t) + fabsf(zf) < MIN_FIELD_LSB) {
        data->have_prev = false; /* ring lifted off or magnets missing */
    } else {
        float deg = atan2f(zf, t) * (180.0f / 3.14159265f);

        if (!data->have_prev) {
            data->prev_deg = deg;
            data->acc_deg = 0.0f;
            data->have_prev = true;
        } else {
            float d = deg - data->prev_deg;

            while (d > 180.0f) {
                d -= 360.0f;
            }
            while (d <= -180.0f) {
                d += 360.0f;
            }
            data->prev_deg = deg;
            motion = fabsf(d) > MOTION_DEG;

            if (fabsf(d) > MAX_JUMP_DEG) {
                /* moved too far since the last sample to know which way: drop it, never guess */
                data->acc_deg = 0.0f;
                LOG_DBG("resync after a %.0f deg jump", (double)d);
            } else {
                float step = 360.0f * cfg->pole_pairs / cfg->steps_per_rev;
                float thr = step * (0.5f + cfg->hysteresis_percent / 100.0f);
                int32_t steps = 0;

                data->acc_deg += d;
                while (data->acc_deg >= thr) {
                    data->acc_deg -= step;
                    steps++;
                }
                while (data->acc_deg <= -thr) {
                    data->acc_deg += step;
                    steps--;
                }
                if (steps != 0) {
                    input_report_rel(dev, cfg->input_code, cfg->invert ? -steps : steps, true,
                                     K_FOREVER);
                }
            }
        }
        if ((data->samples++ % 200) == 0) {
            LOG_DBG("tan %d z %d angle %.1f acc %.1f", tan, z, (double)deg,
                    (double)data->acc_deg);
        }
    }

    if (motion) {
        data->last_motion = k_uptime_get();
    }
    k_work_schedule_for_queue(&tmag_ring_q, &data->work, K_MSEC(next_interval(dev)));
}

static int tmag_ring_init(const struct device *dev) {
    const struct tmag_ring_config *cfg = dev->config;
    struct tmag_ring_data *data = dev->data;

    if (!i2c_is_ready_dt(&cfg->i2c)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }
    if (cfg->int_gpio.port != NULL) {
        if (!gpio_is_ready_dt(&cfg->int_gpio)) {
            return -ENODEV;
        }
        gpio_pin_configure_dt(&cfg->int_gpio, GPIO_OUTPUT_INACTIVE | GPIO_OPEN_DRAIN);
    }
    if (!tmag_ring_q_started) {
        k_work_queue_init(&tmag_ring_q);
        k_work_queue_start(&tmag_ring_q, tmag_ring_stack, K_THREAD_STACK_SIZEOF(tmag_ring_stack),
                           CONFIG_TMAG5273_RING_THREAD_PRIORITY, NULL);
        k_thread_name_set(&tmag_ring_q.thread, "tmag5273_ring");
        tmag_ring_q_started = true;
    }
    data->dev = dev;
    k_work_init_delayable(&data->work, tmag_ring_work);
    /* configure from the work queue: never blocks boot, and retries if the sensor is not wired */
    k_work_schedule_for_queue(&tmag_ring_q, &data->work, K_MSEC(300));
    return 0;
}

#define TMAG_RING_INIT(n)                                                                          \
    static struct tmag_ring_data tmag_ring_data_##n;                                               \
    static const struct tmag_ring_config tmag_ring_config_##n = {                                  \
        .i2c = I2C_DT_SPEC_INST_GET(n),                                                            \
        .int_gpio = GPIO_DT_SPEC_INST_GET_OR(n, int_gpios, {0}),                                   \
        .pole_pairs = DT_INST_PROP(n, pole_pairs),                                                 \
        .steps_per_rev = DT_INST_PROP(n, steps_per_rev),                                           \
        .hysteresis_percent = DT_INST_PROP(n, hysteresis_percent),                                 \
        .tangential_axis = DT_INST_PROP(n, tangential_axis),                                       \
        .tangential_gain_percent = DT_INST_PROP(n, tangential_gain_percent),                       \
        .conv_avg = DT_INST_PROP(n, conv_avg),                                                     \
        .active_ms = DT_INST_PROP(n, active_interval_ms),                                          \
        .idle_ms = DT_INST_PROP(n, idle_interval_ms),                                              \
        .rest_ms = DT_INST_PROP(n, rest_interval_ms),                                              \
        .idle_after_ms = DT_INST_PROP(n, idle_after_ms),                                           \
        .rest_after_ms = DT_INST_PROP(n, rest_after_ms),                                           \
        .invert = DT_INST_PROP(n, invert),                                                         \
        .input_code = DT_INST_PROP(n, input_code),                                                 \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, tmag_ring_init, NULL, &tmag_ring_data_##n, &tmag_ring_config_##n,     \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TMAG_RING_INIT)
