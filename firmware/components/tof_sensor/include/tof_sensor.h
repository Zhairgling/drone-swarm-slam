#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Number of zones in 8x8 resolution mode. */
#define TOF_ZONES 64

/** Maximum number of sensors supported. */
#define TOF_MAX_SENSORS 4

/**
 * Single scan from one VL53L8CX sensor (8x8 grid).
 *
 * Matches the fields in drone_swarm_msgs/ToFScan.
 */
typedef struct {
    uint8_t  sensor_id;              /**< Sensor index 0–3 */
    uint16_t ranges_mm[TOF_ZONES];   /**< Range per zone (mm), 0 = invalid */
    uint8_t  status[TOF_ZONES];      /**< Per-zone status (0 = valid) */
    uint16_t ambient[TOF_ZONES];     /**< Ambient light (kcps/SPAD) */
    int8_t   temperature_degc;       /**< Die temperature at measurement */
    int64_t  timestamp_us;           /**< esp_timer_get_time() at readout */
} tof_scan_t;

/**
 * Callback invoked from the polling task when new data arrives.
 *
 * Called once per sensor per frame.  Runs in the polling task context —
 * keep processing minimal or copy data and defer.
 */
typedef void (*tof_data_cb_t)(const tof_scan_t *scan, void *user_data);

/**
 * Initialise the I2C bus, configure LPn GPIOs, power-sequence all sensors,
 * assign unique I2C addresses, and load firmware into each VL53L8CX.
 *
 * Must be called once before tof_sensor_start().
 *
 * @return ESP_OK on success, or an error code.
 */
esp_err_t tof_sensor_init(void);

/**
 * Start continuous ranging on all sensors and launch the background
 * polling task.
 *
 * @return ESP_OK on success, or an error code.
 */
esp_err_t tof_sensor_start(void);

/**
 * Stop ranging and delete the polling task.
 *
 * @return ESP_OK on success, or an error code.
 */
esp_err_t tof_sensor_stop(void);

/**
 * Register a callback for new scan data.
 *
 * Only one callback is supported; a second call replaces the first.
 * Pass NULL to unregister.
 *
 * @param cb        Callback function (may be NULL).
 * @param user_data Opaque pointer forwarded to @p cb.
 * @return ESP_OK.
 */
esp_err_t tof_sensor_register_callback(tof_data_cb_t cb, void *user_data);

/**
 * Copy the most recent scan for @p sensor_id into @p out.
 *
 * Thread-safe (uses a spinlock internally).  Returns immediately even
 * if no new data has arrived since the last call.
 *
 * @param sensor_id Sensor index (0 .. CONFIG_TOF_SENSOR_COUNT-1).
 * @param[out] out  Destination buffer.
 * @return ESP_OK              Data copied.
 * @return ESP_ERR_INVALID_ARG sensor_id out of range or out is NULL.
 * @return ESP_ERR_INVALID_STATE Driver not initialised or no data yet.
 */
esp_err_t tof_sensor_get_scan(uint8_t sensor_id, tof_scan_t *out);

#ifdef __cplusplus
}
#endif
