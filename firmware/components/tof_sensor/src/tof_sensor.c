#include "tof_sensor.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "vl53l8cx_api.h"

static const char *TAG = "tof_sensor";

/* ── Pin / address tables ────────────────────────────────────────────── */

static const gpio_num_t lpn_gpios[TOF_MAX_SENSORS] = {
    CONFIG_TOF_LPN_GPIO_0,
    CONFIG_TOF_LPN_GPIO_1,
    CONFIG_TOF_LPN_GPIO_2,
    CONFIG_TOF_LPN_GPIO_3,
};

/* DESIGN: first sensor keeps the default address 0x52 (7-bit 0x29).
   Subsequent sensors get 0x54, 0x56, 0x58 (7-bit 0x2A, 0x2B, 0x2C).
   Matches the wiring guide in docs/hardware_wiring.md. */
static const uint16_t sensor_addrs[TOF_MAX_SENSORS] = {
    0x52, 0x54, 0x56, 0x58,
};

/* ── Module state ────────────────────────────────────────────────────── */

static VL53L8CX_Configuration sensors[TOF_MAX_SENSORS];
static i2c_master_bus_handle_t i2c_bus;
static tof_scan_t              cached_scans[TOF_MAX_SENSORS];
static portMUX_TYPE            scan_lock = portMUX_INITIALIZER_UNLOCKED;
static bool                    has_data[TOF_MAX_SENSORS];
static TaskHandle_t            poll_task_handle;
static volatile bool           poll_running;
static tof_data_cb_t           user_cb;
static void                   *user_cb_arg;
static bool                    initialised;

/* ── Helpers ─────────────────────────────────────────────────────────── */

static void lpn_set(uint8_t idx, uint32_t level)
{
    gpio_set_level(lpn_gpios[idx], level);
}

static void lpn_init_all(void)
{
    const gpio_config_t cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = 0,
    };

    for (int i = 0; i < CONFIG_TOF_SENSOR_COUNT; i++) {
        gpio_config_t c = cfg;
        c.pin_bit_mask = 1ULL << lpn_gpios[i];
        ESP_ERROR_CHECK(gpio_config(&c));
        lpn_set(i, 0);  /* all sensors disabled */
    }
}

/* Stored so we can copy into each sensor's platform struct (the rjrp44
   platform.c uses bus_config.i2c_port to index its internal buffer). */
static i2c_master_bus_config_t i2c_bus_cfg;

static esp_err_t i2c_bus_init(void)
{
    i2c_bus_cfg = (i2c_master_bus_config_t){
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = CONFIG_TOF_I2C_SCL_GPIO,
        .sda_io_num = CONFIG_TOF_I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus);
}

/**
 * Add a device on the bus at @p addr_8bit and store the handle in the
 * sensor's platform struct.
 */
static esp_err_t add_i2c_device(VL53L8CX_Configuration *dev, uint16_t addr_8bit)
{
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr_8bit >> 1,
        .scl_speed_hz = CONFIG_TOF_I2C_FREQ_HZ,
    };
    return i2c_master_bus_add_device(i2c_bus, &dev_cfg, &dev->platform.handle);
}

/**
 * Power-sequence one sensor: raise LPn, wait for boot, init ULD,
 * reassign I2C address, configure resolution + frequency.
 */
static esp_err_t init_one_sensor(uint8_t idx)
{
    VL53L8CX_Configuration *dev = &sensors[idx];
    uint8_t is_alive = 0;
    uint8_t status;

    /* Populate platform fields used by the ULD platform layer */
    dev->platform.bus_config = i2c_bus_cfg;

    /* Wake sensor — LPn HIGH */
    lpn_set(idx, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Attach at the default address first */
    esp_err_t err = add_i2c_device(dev, VL53L8CX_DEFAULT_I2C_ADDRESS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "sensor %d: i2c add failed: %s", idx, esp_err_to_name(err));
        return err;
    }
    dev->platform.address = VL53L8CX_DEFAULT_I2C_ADDRESS;

    status = vl53l8cx_is_alive(dev, &is_alive);
    if (status || !is_alive) {
        ESP_LOGE(TAG, "sensor %d: not detected at 0x%02X", idx,
                 VL53L8CX_DEFAULT_I2C_ADDRESS);
        return ESP_ERR_NOT_FOUND;
    }

    /* Load firmware (~84 KB over I2C) */
    status = vl53l8cx_init(dev);
    if (status) {
        ESP_LOGE(TAG, "sensor %d: init failed (status %d)", idx, status);
        return ESP_FAIL;
    }

    /* Reassign to unique address (skip for sensor 0 which keeps default) */
    if (sensor_addrs[idx] != VL53L8CX_DEFAULT_I2C_ADDRESS) {
        status = vl53l8cx_set_i2c_address(dev, sensor_addrs[idx]);
        if (status) {
            ESP_LOGE(TAG, "sensor %d: set addr 0x%02X failed", idx,
                     sensor_addrs[idx]);
            return ESP_FAIL;
        }

        /* Remove old device handle and re-add at the new address */
        i2c_master_bus_rm_device(dev->platform.handle);
        err = add_i2c_device(dev, sensor_addrs[idx]);
        if (err != ESP_OK) {
            return err;
        }
        dev->platform.address = sensor_addrs[idx];
    }

    /* Configure 8×8 resolution and target ranging frequency */
    status = vl53l8cx_set_resolution(dev, VL53L8CX_RESOLUTION_8X8);
    if (status) {
        ESP_LOGE(TAG, "sensor %d: set resolution failed", idx);
        return ESP_FAIL;
    }

    status = vl53l8cx_set_ranging_frequency_hz(dev, CONFIG_TOF_RANGING_FREQ_HZ);
    if (status) {
        ESP_LOGE(TAG, "sensor %d: set frequency failed", idx);
        return ESP_FAIL;
    }

    /* DESIGN: closest-first target order — best for obstacle avoidance. */
    status = vl53l8cx_set_target_order(dev, VL53L8CX_TARGET_ORDER_CLOSEST);
    if (status) {
        ESP_LOGE(TAG, "sensor %d: set target order failed", idx);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "sensor %d: ready at 0x%02X", idx, sensor_addrs[idx]);
    return ESP_OK;
}

/* ── Polling task ────────────────────────────────────────────────────── */

static void fill_scan(tof_scan_t *scan, uint8_t sensor_id,
                       const VL53L8CX_ResultsData *results)
{
    scan->sensor_id = sensor_id;
    scan->timestamp_us = esp_timer_get_time();
    scan->temperature_degc = results->silicon_temp_degc;

    for (int z = 0; z < TOF_ZONES; z++) {
        int16_t raw = results->distance_mm[z];
        scan->ranges_mm[z] = (raw > 0) ? (uint16_t)raw : 0;
        scan->status[z] = results->target_status[z];
#ifndef VL53L8CX_DISABLE_AMBIENT_PER_SPAD
        scan->ambient[z] = (uint16_t)(results->ambient_per_spad[z] >> 11);
#else
        scan->ambient[z] = 0;
#endif
    }
}

static void poll_task(void *arg)
{
    VL53L8CX_ResultsData results;
    uint8_t is_ready;

    ESP_LOGI(TAG, "poll task started (%d sensors)", CONFIG_TOF_SENSOR_COUNT);

    while (poll_running) {
        for (int i = 0; i < CONFIG_TOF_SENSOR_COUNT; i++) {
            uint8_t status = vl53l8cx_check_data_ready(&sensors[i], &is_ready);
            if (status || !is_ready) {
                continue;
            }

            status = vl53l8cx_get_ranging_data(&sensors[i], &results);
            if (status) {
                ESP_LOGW(TAG, "sensor %d: read failed (status %d)", i, status);
                continue;
            }

            tof_scan_t scan;
            fill_scan(&scan, (uint8_t)i, &results);

            /* Cache for get_scan() callers */
            taskENTER_CRITICAL(&scan_lock);
            cached_scans[i] = scan;
            has_data[i] = true;
            taskEXIT_CRITICAL(&scan_lock);

            /* Notify callback */
            if (user_cb) {
                user_cb(&scan, user_cb_arg);
            }
        }

        /* DESIGN: yield between poll rounds.  At 15 Hz per sensor the data
           arrives every ~66 ms; polling at 5 ms keeps latency low without
           hogging the CPU. */
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    ESP_LOGI(TAG, "poll task exiting");
    vTaskDelete(NULL);
}

/* ── Public API ──────────────────────────────────────────────────────── */

esp_err_t tof_sensor_init(void)
{
    if (initialised) {
        ESP_LOGW(TAG, "already initialised");
        return ESP_ERR_INVALID_STATE;
    }

    memset(cached_scans, 0, sizeof(cached_scans));
    memset(has_data, 0, sizeof(has_data));

    esp_err_t err = i2c_bus_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c bus init failed: %s", esp_err_to_name(err));
        return err;
    }

    lpn_init_all();
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Sequentially wake each sensor and assign a unique address.
       Order matters — only one sensor at the default address at a time. */
    for (int i = 0; i < CONFIG_TOF_SENSOR_COUNT; i++) {
        err = init_one_sensor((uint8_t)i);
        if (err != ESP_OK) {
            return err;
        }
    }

    initialised = true;
    ESP_LOGI(TAG, "all %d sensors initialised", CONFIG_TOF_SENSOR_COUNT);
    return ESP_OK;
}

esp_err_t tof_sensor_start(void)
{
    if (!initialised) {
        return ESP_ERR_INVALID_STATE;
    }
    if (poll_running) {
        return ESP_ERR_INVALID_STATE;
    }

    for (int i = 0; i < CONFIG_TOF_SENSOR_COUNT; i++) {
        uint8_t status = vl53l8cx_start_ranging(&sensors[i]);
        if (status) {
            ESP_LOGE(TAG, "sensor %d: start ranging failed (status %d)", i,
                     status);
            return ESP_FAIL;
        }
    }

    poll_running = true;
    BaseType_t ret = xTaskCreate(poll_task, "tof_poll",
                                  CONFIG_TOF_POLL_TASK_STACK, NULL,
                                  CONFIG_TOF_POLL_TASK_PRIORITY,
                                  &poll_task_handle);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "failed to create poll task");
        poll_running = false;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "ranging started");
    return ESP_OK;
}

esp_err_t tof_sensor_stop(void)
{
    if (!poll_running) {
        return ESP_ERR_INVALID_STATE;
    }

    poll_running = false;
    /* Give the task time to notice and exit */
    vTaskDelay(pdMS_TO_TICKS(50));

    for (int i = 0; i < CONFIG_TOF_SENSOR_COUNT; i++) {
        vl53l8cx_stop_ranging(&sensors[i]);
    }

    ESP_LOGI(TAG, "ranging stopped");
    return ESP_OK;
}

esp_err_t tof_sensor_register_callback(tof_data_cb_t cb, void *user_data)
{
    user_cb = cb;
    user_cb_arg = user_data;
    return ESP_OK;
}

esp_err_t tof_sensor_get_scan(uint8_t sensor_id, tof_scan_t *out)
{
    if (sensor_id >= CONFIG_TOF_SENSOR_COUNT || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!initialised) {
        return ESP_ERR_INVALID_STATE;
    }

    taskENTER_CRITICAL(&scan_lock);
    bool ready = has_data[sensor_id];
    if (ready) {
        *out = cached_scans[sensor_id];
    }
    taskEXIT_CRITICAL(&scan_lock);

    return ready ? ESP_OK : ESP_ERR_INVALID_STATE;
}
