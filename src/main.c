#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/logging/log.h>

#include <hubble/hubble.h>

#include "b64.h"

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

static uint8_t master_key[CONFIG_HUBBLE_KEY_SIZE];
static const char master_key_str[] = HUBBLE_KEY;

#define ADV_UPDATE_PERIOD_S  300
#define ADV_INTERVAL_S       2
#define ADV_INTERVAL_CNT_MIN (ADV_INTERVAL_S * 1600)
#define ADV_INTERVAL_CNT_MAX (ADV_INTERVAL_S * 2000)

#define LED0_NODE DT_ALIAS(led0)
#define SW0_NODE  DT_ALIAS(sw0)

#define HUBBLE_USER_BUFFER_LEN 31
static uint8_t _hubble_user_buffer[HUBBLE_USER_BUFFER_LEN];

static const struct gpio_dt_spec led    = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(SW0_NODE, gpios);

static volatile bool led_state = false;
static struct gpio_callback button_cb_data;

static uint16_t app_adv_uuids[1] = { HUBBLE_BLE_UUID };
static struct bt_data app_ad[2] = {
    BT_DATA(BT_DATA_UUID16_ALL, &app_adv_uuids, sizeof(app_adv_uuids)),
    {},
};

K_SEM_DEFINE(timer_sem, 0, 1);

static void timer_cb(struct k_timer *timer) { k_sem_give(&timer_sem); }
K_TIMER_DEFINE(message_timer, timer_cb, NULL);

static void button_pressed(const struct device *dev,
                            struct gpio_callback *cb, uint32_t pins)
{
    led_state = !led_state;
    gpio_pin_set_dt(&led, led_state);
    k_sem_give(&timer_sem);
    LOG_INF("Button pressed — LED %s", led_state ? "ON" : "OFF");
}

static int decode_master_key(void)
{
    size_t keylen = b64_decoded_size(master_key_str);
    if (keylen != sizeof(master_key)) {
        LOG_ERR("Given key incorrect size (%d bytes)", keylen);
        return -1;
    }
    int ret = b64_decode(master_key_str, master_key, sizeof(master_key));
    if (ret != 0) LOG_ERR("Failed to decode given key");
    return ret;
}

int main(void)
{
    int err;

    if (!gpio_is_ready_dt(&led) || !gpio_is_ready_dt(&button)) {
        return -1;
    }

    gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&button_cb_data, button_pressed, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);

    LOG_INF("Hubble LED-toggle beacon started");

    err = bt_enable(NULL);
    if (err != 0) { LOG_ERR("Bluetooth init failed (err %d)", err); return err; }

    err = decode_master_key();
    if (err != 0) return err;

    err = hubble_init(TIME, master_key);
    if (err != 0) { LOG_ERR("Failed to initialize Hubble"); return err; }

    k_timer_start(&message_timer,
                  K_SECONDS(ADV_UPDATE_PERIOD_S),
                  K_SECONDS(ADV_UPDATE_PERIOD_S));

    while (1) {
        size_t out_len = HUBBLE_USER_BUFFER_LEN;

        uint8_t custom_payload[2] = { 0x4C, led_state ? 0x01 : 0x00 };

        err = hubble_ble_advertise_get(custom_payload, sizeof(custom_payload),
                                       _hubble_user_buffer, &out_len);
        if (err != 0) { LOG_ERR("Failed to get adv data"); goto end; }

        app_ad[1].data_len = out_len;
        app_ad[1].type     = BT_DATA_SVC_DATA16;
        app_ad[1].data     = _hubble_user_buffer;

        err = bt_le_adv_start(
            BT_LE_ADV_PARAM(BT_LE_ADV_OPT_USE_NRPA,
                            ADV_INTERVAL_CNT_MIN, ADV_INTERVAL_CNT_MAX, NULL),
            app_ad, ARRAY_SIZE(app_ad), NULL, 0);
        if (err != 0) { LOG_ERR("Adv start failed (err %d)", err); goto end; }

        k_sem_take(&timer_sem, K_FOREVER);
        bt_le_adv_stop();
    }

end:
    bt_disable();
    return err;
}
