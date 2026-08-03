/*
 * Copyright (c) 2022 The ZMK Contributors
 * Copyright (c) 2026 LiNEA40 contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_sensor_rotate_accel

#include <zephyr/device.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/keymap.h>
#include <zmk/sensors.h>

struct behavior_sensor_rotate_config {
    struct zmk_behavior_binding cw_binding;
    struct zmk_behavior_binding ccw_binding;
    int tap_ms;
    bool override_params;
};

struct behavior_sensor_rotate_data {
    struct sensor_value remainder[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];
    int triggers[ZMK_KEYMAP_SENSORS_LEN][ZMK_KEYMAP_LAYERS_LEN];
};

int zmk_behavior_sensor_rotate_common_accept_data(
    struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event,
    const struct zmk_sensor_config *sensor_config, size_t channel_data_size,
    const struct zmk_sensor_channel_data *channel_data);
int zmk_behavior_sensor_rotate_common_process(
    struct zmk_behavior_binding *binding, struct zmk_behavior_binding_event event,
    enum behavior_sensor_binding_process_mode mode);

static const struct behavior_driver_api behavior_sensor_rotate_accel_driver_api = {
    .sensor_binding_accept_data = zmk_behavior_sensor_rotate_common_accept_data,
    .sensor_binding_process = zmk_behavior_sensor_rotate_common_process,
};

#define SENSOR_ROTATE_ACCEL_INST(n)                                                               \
    static struct behavior_sensor_rotate_config behavior_sensor_rotate_accel_config_##n = {       \
        .cw_binding = {.behavior_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(n, bindings, 0))},  \
        .ccw_binding = {.behavior_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(n, bindings, 1))}, \
        .tap_ms = DT_INST_PROP(n, tap_ms),                                                        \
        .override_params = true,                                                                  \
    };                                                                                            \
    static struct behavior_sensor_rotate_data behavior_sensor_rotate_accel_data_##n = {};        \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, &behavior_sensor_rotate_accel_data_##n,               \
                            &behavior_sensor_rotate_accel_config_##n, POST_KERNEL,               \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                 \
                            &behavior_sensor_rotate_accel_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SENSOR_ROTATE_ACCEL_INST)
