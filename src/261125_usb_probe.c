#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zmk/keymap.h>
#include "261125_probe.h"

static const struct device *const usb = DEVICE_DT_GET(DT_NODELABEL(linea40_probe_uart));
static atomic_t minus_down;

/* Observe the local right-hand switch before combos/hold-taps consume its event.
 * Always forward the entire original event and return the original result. */
int __real_raise_zmk_position_state_changed(struct zmk_position_state_changed event);
int __wrap_raise_zmk_position_state_changed(struct zmk_position_state_changed event) {
    if (event.source == ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL && event.position == 19) {
        atomic_set(&minus_down, event.state);
    }
    return __real_raise_zmk_position_state_changed(event);
}

static void send_snapshot(struct k_work *work) {
    ARG_UNUSED(work);
    uint8_t frame[9];
    uint8_t layer = zmk_keymap_layer_index_to_id(zmk_keymap_highest_layer_active());
    if (!probe_reply(frame, atomic_get(&minus_down) != 0, layer)) return;
    for (size_t i = 0; i < sizeof(frame); ++i) uart_poll_out(usb, frame[i]);
}
static K_WORK_DEFINE(snapshot, send_snapshot);

/* No timer, key history or background stream. Reply only to a complete USB query. */
static void on_receive(const struct device *dev, void *user) {
    ARG_UNUSED(user);
    static uint8_t matched;
    uint8_t chunk[16];
    if (!uart_irq_update(dev) || !uart_irq_rx_ready(dev)) return;
    int n;
    while ((n = uart_fifo_read(dev, chunk, sizeof(chunk))) > 0) {
        for (int i = 0; i < n; ++i) {
            if (probe_request(&matched, chunk[i])) k_work_submit(&snapshot);
        }
    }
}

static int probe_init(void) {
    if (!device_is_ready(usb)) return -ENODEV;
    int rc = uart_irq_callback_user_data_set(usb, on_receive, NULL);
    if (rc != 0) return rc;
    uart_irq_rx_enable(usb);
    return 0;
}
SYS_INIT(probe_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
