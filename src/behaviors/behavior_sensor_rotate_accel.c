/*
 * Copyright (c) 2022 The ZMK Contributors
 * Copyright (c) 2026 LiNEA40 contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_sensor_rotate_accel

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/behavior_queue.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/keymap.h>
#include <zmk/sensors.h>
#include <zmk/virtual_key_position.h>

struct behavior_sensor_rotate_accel_config {
    struct zmk_behavior_binding cw_binding;
    struct zmk_behavior_binding ccw_binding;
    uint16_t tap_ms;
    uint16_t slow_interval_ms;
    uint16_t fast_interval_ms;
    uint8_t slow_scale_percent;
    uint8_t fast_scale_percent;
};

struct behavior_sensor_rotate_accel_data {
    struct sensor_value remainder[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];
    int triggers[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];
    int64_t last_trigger_ms[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];
    uint8_t scale_percent[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];
};

static uint8_t scale_for_interval(const struct behavior_sensor_rotate_accel_config *config,
                                  int64_t interval_ms) {
    if (interval_ms <= 0 || interval_ms >= config->slow_interval_ms) {
        return config->slow_scale_percent;
    }

    if (interval_ms <= config->fast_interval_ms ||
        config->slow_interval_ms <= config->fast_interval_ms) {
        return config->fast_scale_percent;
    }

    int32_t interval_range = config->slow_interval_ms - config->fast_interval_ms;
    int32_t scale_range = config->fast_scale_percent - config->slow_scale_percent;
    int32_t elapsed_from_slow = config->slow_interval_ms - interval_ms;

    return config->slow_scale_percent + (scale_range * elapsed_from_slow / interval_range);
}

static uint32_t scale_motion(uint32_t motion, uint8_t percent) {
    int32_t horizontal = (int16_t)(motion >> 16);
    int32_t vertical = (int16_t)(motion & 0xFFFF);

    uint16_t scaled_horizontal = (uint16_t)(horizontal * percent / 100);
    uint16_t scaled_vertical = (uint16_t)(vertical * percent / 100);

    return ((uint32_t)scaled_horizontal << 16) | scaled_vertical;
}

static int sensor_binding_accept_data(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event,
                                      const struct zmk_sensor_config *sensor_config,
                                      size_t channel_data_size,
                                      const struct zmk_sensor_channel_data *channel_data) {
    ARG_UNUSED(channel_data_size);

    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_sensor_rotate_accel_config *config = dev->config;
    struct behavior_sensor_rotate_accel_data *data = dev->data;
    const struct sensor_value value = channel_data[0].value;
    const int sensor_index = ZMK_SENSOR_POSITION_FROM_VIRTUAL_KEY_POSITION(event.position);
    int triggers;

    if (value.val1 == 0) {
        triggers = value.val2;
    } else {
        struct sensor_value remainder = data->remainder[sensor_index][event.layer];

        remainder.val1 += value.val1;
        remainder.val2 += value.val2;

        if (remainder.val2 >= 1000000 || remainder.val2 <= -1000000) {
            remainder.val1 += remainder.val2 / 1000000;
            remainder.val2 %= 1000000;
        }

        const int trigger_degrees = 360 / sensor_config->triggers_per_rotation;
        triggers = remainder.val1 / trigger_degrees;
        remainder.val1 %= trigger_degrees;
        data->remainder[sensor_index][event.layer] = remainder;
    }

    data->triggers[sensor_index][event.layer] = triggers;
    if (triggers != 0) {
        const int64_t now = k_uptime_get();
        const int64_t last_trigger = data->last_trigger_ms[sensor_index][event.layer];

        data->scale_percent[sensor_index][event.layer] =
            (triggers > 1 || triggers < -1)
                ? config->fast_scale_percent
                : scale_for_interval(config, now - last_trigger);
        data->last_trigger_ms[sensor_index][event.layer] = now;
    }

    return 0;
}

static int sensor_binding_process(struct zmk_behavior_binding *binding,
                                  struct zmk_behavior_binding_event event,
                                  enum behavior_sensor_binding_process_mode mode) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_sensor_rotate_accel_config *config = dev->config;
    struct behavior_sensor_rotate_accel_data *data = dev->data;
    const int sensor_index = ZMK_SENSOR_POSITION_FROM_VIRTUAL_KEY_POSITION(event.position);

    if (mode != BEHAVIOR_SENSOR_BINDING_PROCESS_MODE_TRIGGER) {
        data->triggers[sensor_index][event.layer] = 0;
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

    int triggers = data->triggers[sensor_index][event.layer];
    if (triggers == 0) {
        return ZMK_BEHAVIOR_TRANSPARENT;
    }

    struct zmk_behavior_binding triggered_binding;
    uint32_t motion;
    if (triggers > 0) {
        triggered_binding = config->cw_binding;
        motion = binding->param1;
    } else {
        triggers = -triggers;
        triggered_binding = config->ccw_binding;
        motion = binding->param2;
    }

    triggered_binding.param1 =
        scale_motion(motion, data->scale_percent[sensor_index][event.layer]);

#if IS_ENABLED(CONFIG_ZMK_SPLIT)
    event.source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL;
#endif

    for (int i = 0; i < triggers; i++) {
        zmk_behavior_queue_add(&event, triggered_binding, true, config->tap_ms);
        zmk_behavior_queue_add(&event, triggered_binding, false, 0);
    }

    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_sensor_rotate_accel_driver_api = {
    .locality = BEHAVIOR_LOCALITY_CENTRAL,
    .sensor_binding_accept_data = sensor_binding_accept_data,
    .sensor_binding_process = sensor_binding_process,
};

#define SENSOR_ROTATE_ACCEL_INST(n)                                                               \
    static struct behavior_sensor_rotate_accel_config behavior_sensor_rotate_accel_config_##n = { \
        .cw_binding = {.behavior_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(n, bindings, 0))},  \
        .ccw_binding = {.behavior_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(n, bindings, 1))}, \
        .tap_ms = DT_INST_PROP(n, tap_ms),                                                        \
        .slow_interval_ms = DT_INST_PROP(n, slow_interval_ms),                                    \
        .fast_interval_ms = DT_INST_PROP(n, fast_interval_ms),                                    \
        .slow_scale_percent = DT_INST_PROP(n, slow_scale_percent),                                \
        .fast_scale_percent = DT_INST_PROP(n, fast_scale_percent),                                \
    };                                                                                             \
    static struct behavior_sensor_rotate_accel_data behavior_sensor_rotate_accel_data_##n = {};   \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, &behavior_sensor_rotate_accel_data_##n,                \
                            &behavior_sensor_rotate_accel_config_##n, POST_KERNEL,                \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                  \
                            &behavior_sensor_rotate_accel_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SENSOR_ROTATE_ACCEL_INST)
