#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(varia, LOG_LEVEL_INF);


/* Forward declaration */
static void scan_cb(const bt_addr_le_t *addr, int8_t rssi,
                    uint8_t type, struct net_buf_simple *ad);

/* ── LEDs ─────────────────────────────────────────────────────────────────── */
static const struct gpio_dt_spec led_close = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led_medium = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
static const struct gpio_dt_spec led_long = GPIO_DT_SPEC_GET(DT_ALIAS(led3), gpios);
static const struct gpio_dt_spec buzzer = GPIO_DT_SPEC_GET(DT_ALIAS(buzzer1), gpios);

/* ── Varia UUIDs ─────────────────────────────────────────────────────────── */
/* 6a4e3200-667b-11e3-949a-0800200c9a66 */
static struct bt_uuid_128 varia_svc_uuid = BT_UUID_INIT_128(
    0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08, 0x9a, 0x94,
    0xe3, 0x11, 0x7b, 0x66, 0x00, 0x32, 0x4e, 0x6a);

/* 6a4e3203-667b-11e3-949a-0800200c9a66 */
static struct bt_uuid_128 varia_chr_uuid = BT_UUID_INIT_128(
    0x66, 0x9a, 0x0c, 0x20, 0x00, 0x08, 0x9a, 0x94,
    0xe3, 0x11, 0x7b, 0x66, 0x03, 0x32, 0x4e, 0x6a);

/* ── State ───────────────────────────────────────────────────────────────── */
static int             closest_dist = 255;

/* A bucket stays "on" for this long after the last notification that set it.
 * Longer than the ~100 ms inter-notification gap so two rapid-fire
 * notifications from the same radar cycle both contribute. */
#define BUCKET_HOLD_MS 400
static int64_t close_last_seen  = INT64_MIN / 2;
static int64_t medium_last_seen = INT64_MIN / 2;
static int64_t long_last_seen   = INT64_MIN / 2;

/* ── LED update ──────────────────────────────────────────────────────────── */
static void update_led(void)
{
    int64_t now = k_uptime_get();
    bool close_on  = (now - close_last_seen)  < BUCKET_HOLD_MS;
    bool medium_on = (now - medium_last_seen) < BUCKET_HOLD_MS;
    bool long_on   = (now - long_last_seen)   < BUCKET_HOLD_MS;

    gpio_pin_set_dt(&led_close,  close_on  ? 1 : 0);
    gpio_pin_set_dt(&led_medium, medium_on ? 1 : 0);
    gpio_pin_set_dt(&led_long,   long_on   ? 1 : 0);
    gpio_pin_set_dt(&buzzer,     close_on  ? 1 : 0);
}

/* ── Radar notification ──────────────────────────────────────────────────── */
static uint8_t notify_cb(struct bt_conn *conn,
                          struct bt_gatt_subscribe_params *params,
                          const void *data, uint16_t length)
{
    if (!data) {
        /* CCC write rejected (device pushes notifications without CCC) — keep handler */
        return BT_GATT_ITER_CONTINUE;
    }

    const uint8_t *d = data;
    int threats = (length - 1) / 3;
    closest_dist = 255;

    if (threats > 0) {
        int64_t now = k_uptime_get();
        for (int i = 0; i < threats; i++) {
            uint8_t dist = d[2 + i * 3];
            if (dist <= 40)       close_last_seen  = now;
            else if (dist <= 100) medium_last_seen = now;
            else                  long_last_seen   = now;
            if (dist < closest_dist) closest_dist = dist;
        }
        LOG_INF("Threats: %d  closest: %dm", threats, closest_dist);
    } else {
        /* Genuine all-clear: expire all buckets immediately */
        close_last_seen  = INT64_MIN / 2;
        medium_last_seen = INT64_MIN / 2;
        long_last_seen   = INT64_MIN / 2;
        LOG_INF("All clear");
    }

    return BT_GATT_ITER_CONTINUE;
}

/* ── GATT discovery ──────────────────────────────────────────────────────── */
static struct bt_gatt_subscribe_params subscribe_params;
static struct bt_gatt_discover_params  discover_chr_params;
static struct bt_gatt_discover_params  discover_svc_params;

static uint8_t discover_chr_cb(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 struct bt_gatt_discover_params *params)
{
    if (!attr) {
        LOG_WRN("Characteristic not found");
        return BT_GATT_ITER_STOP;
    }

    struct bt_gatt_chrc *chrc = attr->user_data;
    uint16_t value_handle = chrc->value_handle;

    LOG_INF("Characteristic handle: %d props: 0x%02x",
            value_handle, chrc->properties);

    subscribe_params.notify       = notify_cb;
    subscribe_params.value        = BT_GATT_CCC_NOTIFY;
    subscribe_params.value_handle = value_handle;
    subscribe_params.ccc_handle   = value_handle + 1;

    struct bt_conn_info info;
    bt_conn_get_info(conn, &info);

    int err = bt_gatt_subscribe(conn, &subscribe_params);
    if (err && err != -EALREADY) {
        LOG_WRN("Subscribe failed (%d) — trying resubscribe", err);
        err = bt_gatt_resubscribe(info.id, info.le.dst, &subscribe_params);
        if (err && err != -EALREADY) {
            LOG_ERR("Resubscribe failed: %d", err);
        } else {
            LOG_INF("Listening for radar notifications (resubscribe)...");
        }
    } else {
        LOG_INF("Listening for radar notifications...");
    }

    return BT_GATT_ITER_STOP;
}

static uint8_t discover_svc_cb(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr,
                                 struct bt_gatt_discover_params *params)
{
    if (!attr) {
        LOG_ERR("Varia service not found");
        bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
        return BT_GATT_ITER_STOP;
    }

    LOG_INF("Varia service found — discovering characteristic...");

    discover_chr_params.uuid         = &varia_chr_uuid.uuid;
    discover_chr_params.func         = discover_chr_cb;
    discover_chr_params.start_handle = attr->handle + 1;
    discover_chr_params.end_handle   = 0xffff;
    discover_chr_params.type         = BT_GATT_DISCOVER_CHARACTERISTIC;

    bt_gatt_discover(conn, &discover_chr_params);
    return BT_GATT_ITER_STOP;
}

static void start_discovery(struct bt_conn *conn)
{
    discover_svc_params.uuid         = &varia_svc_uuid.uuid;
    discover_svc_params.func         = discover_svc_cb;
    discover_svc_params.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE;
    discover_svc_params.end_handle   = BT_ATT_LAST_ATTRIBUTE_HANDLE;
    discover_svc_params.type         = BT_GATT_DISCOVER_PRIMARY;

    int err = bt_gatt_discover(conn, &discover_svc_params);
    if (err) {
        LOG_ERR("Discovery start failed: %d", err);
    }
}

/* ── Auth callbacks ───────────────────────────────────────────────────────── */
static void auth_cancel(struct bt_conn *conn)
{
    LOG_INF("Pairing cancelled");
}

static struct bt_conn_auth_cb auth_cb = {
    .cancel = auth_cancel,
};

static void pairing_complete_cb(struct bt_conn *conn, bool bonded)
{
    LOG_INF("Pairing complete, bonded: %d", bonded);
}

static void pairing_failed_cb(struct bt_conn *conn,
                                enum bt_security_err reason)
{
    LOG_WRN("Pairing failed: %d — continuing at L1", reason);
}

static struct bt_conn_auth_info_cb auth_info_cb = {
    .pairing_complete = pairing_complete_cb,
    .pairing_failed   = pairing_failed_cb,
};

/* ── Security changed ────────────────────────────────────────────────────── */
static void security_changed_cb(struct bt_conn *conn, bt_security_t level,
                                  enum bt_security_err err)
{
    /* Discovery is already started from connected_cb; log only */
    LOG_INF("Security changed: level=%d err=%d", level, err);
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connect failed: %d", err);
        return;
    }

    LOG_INF("Connected");
    start_discovery(conn);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("Disconnected, reason: 0x%02x", reason);
    closest_dist = 255;
    close_last_seen  = INT64_MIN / 2;
    medium_last_seen = INT64_MIN / 2;
    long_last_seen   = INT64_MIN / 2;
    gpio_pin_set_dt(&led_close, 0);
    gpio_pin_set_dt(&led_medium, 0);
    gpio_pin_set_dt(&led_long, 0);
    gpio_pin_set_dt(&buzzer, 0);

    /* Add a 2 s delay before rescanning to avoid a rapid reconnect storm */
    k_sleep(K_MSEC(2000));
    bt_le_scan_start(BT_LE_SCAN_ACTIVE, scan_cb);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected        = connected_cb,
    .disconnected     = disconnected_cb,
    .security_changed = security_changed_cb,
};

/* ── Scan callback ───────────────────────────────────────────────────────── */
static void scan_cb(const bt_addr_le_t *addr, int8_t rssi,
                     uint8_t type, struct net_buf_simple *ad)
{
    char name[32] = {0};
    bool svc_match = false;

    struct net_buf_simple_state state;
    net_buf_simple_save(ad, &state);

    while (ad->len > 1) {
        uint8_t len   = net_buf_simple_pull_u8(ad);
        uint8_t atype = net_buf_simple_pull_u8(ad);
        if (len == 0) break;

        if (atype == BT_DATA_NAME_COMPLETE ||
            atype == BT_DATA_NAME_SHORTENED) {
            uint8_t nlen = MIN(len - 1, sizeof(name) - 1);
            memcpy(name, ad->data, nlen);
            name[nlen] = '\0';
        }

        /* Match on Varia service UUID (normal-mode advertisement, no name) */
        if ((atype == BT_DATA_UUID128_ALL || atype == BT_DATA_UUID128_SOME) &&
            len - 1 == 16 &&
            memcmp(ad->data, varia_svc_uuid.val, 16) == 0) {
            svc_match = true;
        }

        if (len > 1) net_buf_simple_pull(ad, len - 1);
    }
    net_buf_simple_restore(ad, &state);

    bool name_match = strncmp(name, "eRTL", 4) == 0;
    if (!name_match && !svc_match) return;

    LOG_INF("Found Varia: %s  RSSI: %d  (name=%d svc=%d)",
            name, rssi, name_match, svc_match);
    bt_le_scan_stop();

    struct bt_conn *conn = NULL;
    struct bt_conn_le_create_param create_param = BT_CONN_LE_CREATE_PARAM_INIT(
        BT_CONN_LE_OPT_NONE,
        BT_GAP_SCAN_FAST_INTERVAL,
        BT_GAP_SCAN_FAST_INTERVAL);

    struct bt_le_conn_param conn_param = BT_LE_CONN_PARAM_INIT(
        BT_GAP_INIT_CONN_INT_MIN,
        BT_GAP_INIT_CONN_INT_MAX,
        0, 400);

    int err = bt_conn_le_create(addr, &create_param, &conn_param, &conn);
    if (err) {
        LOG_ERR("Create connection failed: %d", err);
        bt_le_scan_start(BT_LE_SCAN_ACTIVE, scan_cb);
    } else {
        bt_conn_unref(conn);
    }
}

/* ── Main ────────────────────────────────────────────────────────────────── */
int main(void)
{
    LOG_INF("Varia radar bridge starting...");

    gpio_pin_configure_dt(&led_close, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_medium, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_long, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&buzzer, GPIO_OUTPUT_INACTIVE);

    for (int i = 0; i < 5; i++) {
        gpio_pin_set_dt(&led_long, 1); k_sleep(K_MSEC(200));
        gpio_pin_set_dt(&led_long, 0); k_sleep(K_MSEC(100));
        gpio_pin_set_dt(&led_medium, 1); k_sleep(K_MSEC(200));
        gpio_pin_set_dt(&led_medium, 0); k_sleep(K_MSEC(100));
        gpio_pin_set_dt(&led_close, 1); k_sleep(K_MSEC(200));
        gpio_pin_set_dt(&led_close, 0); k_sleep(K_MSEC(100));
    }

    bt_enable(NULL);
    settings_load();

    bt_conn_auth_cb_register(&auth_cb);
    bt_conn_auth_info_cb_register(&auth_info_cb);

    bt_le_scan_start(BT_LE_SCAN_ACTIVE, scan_cb);
    LOG_INF("Scanning for eRTL615...");

    while (1) {
        update_led();
        k_sleep(K_MSEC(50));
    }

    return 0;
}