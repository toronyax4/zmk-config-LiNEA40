/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_directional_gestures

#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <drivers/input_processor.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/behavior.h>
#include <zmk/keymap.h>
#include <zmk/virtual_key_position.h>

enum gesture_direction {
    GESTURE_UP = 0,
    GESTURE_DOWN,
    GESTURE_RIGHT,
    GESTURE_LEFT,
    GESTURE_COUNT,
};

struct directional_gestures_config {
    uint8_t index;
    uint16_t type;
    uint16_t x_code;
    uint16_t y_code;
    uint16_t cooldown_ms;
    const struct zmk_behavior_binding *bindings;
};

struct directional_gestures_data {
    const struct device *dev;
    struct k_work_delayable work;
    int16_t x;
    int16_t y;
    int64_t last_triggered_at;
    uint8_t input_device_index;
    enum gesture_direction pending_direction;
};

static uint16_t abs16(int16_t value) { return value < 0 ? -value : value; }

static void invoke_gesture_binding(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct directional_gestures_data *data =
        CONTAINER_OF(dwork, struct directional_gestures_data, work);
    const struct device *dev = data->dev;
    const struct directional_gestures_config *cfg = dev->config;

    struct zmk_behavior_binding_event event = {
        .position = ZMK_VIRTUAL_KEY_POSITION_BEHAVIOR_INPUT_PROCESSOR(data->input_device_index,
                                                                      cfg->index),
        .timestamp = k_uptime_get(),
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
    };

    int ret = zmk_behavior_invoke_binding(&cfg->bindings[data->pending_direction], event, 1);
    if (ret < 0) {
        LOG_WRN("Directional gesture press failed: %d", ret);
        return;
    }

    event.timestamp = k_uptime_get();
    ret = zmk_behavior_invoke_binding(&cfg->bindings[data->pending_direction], event, 0);
    if (ret < 0) {
        LOG_WRN("Directional gesture release failed: %d", ret);
    }
}

static int directional_gestures_handle_event(const struct device *dev, struct input_event *event,
                                             uint32_t threshold, uint32_t param2,
                                             struct zmk_input_processor_state *state) {
    const struct directional_gestures_config *cfg = dev->config;
    struct directional_gestures_data *data = dev->data;
    ARG_UNUSED(param2);

    if (event->type != cfg->type ||
        (event->code != cfg->x_code && event->code != cfg->y_code)) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (threshold == 0 || event->value == 0) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    if (state) {
        data->input_device_index = state->input_device_index;
    }

    int64_t now = k_uptime_get();
    if (data->last_triggered_at > 0 && now - data->last_triggered_at < cfg->cooldown_ms) {
        return ZMK_INPUT_PROC_STOP;
    }

    if (event->code == cfg->x_code) {
        data->x += event->value;
    } else {
        data->y += event->value;
    }

    data->x = CLAMP(data->x, -((int16_t)threshold * 2), (int16_t)threshold * 2);
    data->y = CLAMP(data->y, -((int16_t)threshold * 2), (int16_t)threshold * 2);

    uint16_t abs_x = abs16(data->x);
    uint16_t abs_y = abs16(data->y);
    if (abs_x < threshold && abs_y < threshold) {
        return ZMK_INPUT_PROC_STOP;
    }

    if (abs_x > abs_y) {
        data->pending_direction = data->x > 0 ? GESTURE_RIGHT : GESTURE_LEFT;
    } else {
        data->pending_direction = data->y > 0 ? GESTURE_DOWN : GESTURE_UP;
    }

    data->x = 0;
    data->y = 0;
    data->last_triggered_at = now;

    k_work_reschedule(&data->work, K_MSEC(1));

    return ZMK_INPUT_PROC_STOP;
}

static int directional_gestures_init(const struct device *dev) {
    struct directional_gestures_data *data = dev->data;

    data->dev = dev;
    k_work_init_delayable(&data->work, invoke_gesture_binding);

    return 0;
}

static struct zmk_input_processor_driver_api directional_gestures_driver_api = {
    .handle_event = directional_gestures_handle_event,
};

#define DIRECTIONAL_GESTURES_INST(n)                                                              \
    static const struct zmk_behavior_binding directional_gestures_bindings_##n[] = {              \
        LISTIFY(DT_INST_PROP_LEN(n, bindings), ZMK_KEYMAP_EXTRACT_BINDING, (, ),                  \
                DT_DRV_INST(n))};                                                                 \
    BUILD_ASSERT(ARRAY_SIZE(directional_gestures_bindings_##n) == GESTURE_COUNT,                  \
                 "directional gestures requires four bindings: up, down, right, left");           \
    static const struct directional_gestures_config directional_gestures_config_##n = {            \
        .index = n,                                                                               \
        .type = DT_INST_PROP_OR(n, type, INPUT_EV_REL),                                           \
        .x_code = DT_INST_PROP(n, x_code),                                                        \
        .y_code = DT_INST_PROP(n, y_code),                                                        \
        .cooldown_ms = DT_INST_PROP(n, cooldown_ms),                                              \
        .bindings = directional_gestures_bindings_##n,                                            \
    };                                                                                            \
    static struct directional_gestures_data directional_gestures_data_##n = {};                    \
    DEVICE_DT_INST_DEFINE(n, &directional_gestures_init, NULL, &directional_gestures_data_##n,     \
                          &directional_gestures_config_##n, POST_KERNEL,                          \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &directional_gestures_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DIRECTIONAL_GESTURES_INST)
