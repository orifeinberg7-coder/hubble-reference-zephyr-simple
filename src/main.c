#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include <hubble/hubble.h>

#include "b64.h"

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

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
static volatile uint16_t button_count = 0;
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
    button_count++;
    gpio_pin_set_dt(&led, led_state);
    k_sem_give(&timer_sem);
    LOG_INF("Button #%u — LED %s", button_count, led_state ? "ON" : "OFF");
}

static int8_t read_die_temp(const struct device *temp_dev)
{
    struct sensor_value val;

    if (sensor_sample_fetch(temp_dev) != 0) {
        LOG_WRN("Temp fetch failed");
        return -128;
    }
    sensor_channel_get(temp_dev, SENSOR_CHAN_DIE_TEMP, &val);
    LOG_INF("Die temp: %d.%06d °C", val.val1, val.val2);
    return (int8_t)val.val1;
}

/*
 * Payload format (6 bytes):
 *   [0]   0x4C   — type marker
 *   [1]   LED    — 0x00=OFF, 0x01=ON
 *   [2]   temp   — signed die temperature in °C
 *   [3-4] count  — button presses since boot (big-endian)
 *   [5]   uptime — minutes since boot (wraps at 255)
 */
#define PAYLOAD_LEN 6

static void build_payload(uint8_t *buf, const struct device *temp_dev)
{
    int8_t temp = read_die_temp(temp_dev);
    uint32_t uptime_min = k_uptime_get() / 60000;

    buf[0] = 0x4C;
    buf[1] = led_state ? 0x01 : 0x00;
    buf[2] = (uint8_t)temp;
    sys_put_be16(button_count, &buf[3]);
    buf[5] = (uint8_t)(uptime_min & 0xFF);

    LOG_INF("Payload: LED=%u temp=%d°C btn=%u up=%um",
            buf[1], temp, button_count, uptime_min & 0xFF);
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

    const struct device *temp_dev = DEVICE_DT_GET(DT_NODELABEL(temp));
    if (!device_is_ready(temp_dev)) {
        LOG_ERR("Die temperature sensor not ready");
        return -1;
    }

    LOG_INF("Hubble multi-sensor beacon started");

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
        uint8_t custom_payload[PAYLOAD_LEN];

        build_payload(custom_payload, temp_dev);

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
