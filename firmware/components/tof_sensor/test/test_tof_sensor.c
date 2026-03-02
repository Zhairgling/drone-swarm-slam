#include "unity.h"
#include "tof_sensor.h"

#include <string.h>

/*
 * These tests verify the tof_sensor API contract without real hardware.
 * They run on the host (ESP-IDF Unity test framework) and check argument
 * validation, state guards, and data structure invariants.
 *
 * Tests that require I2C communication are guarded by the actual init
 * sequence failing gracefully (no sensors connected in CI).
 */

/* ── tof_scan_t structure ────────────────────────────────────────────── */

TEST_CASE("tof_scan_t has expected field sizes", "[tof_sensor]")
{
    tof_scan_t scan;

    /* 64 zones for 8x8 grid */
    TEST_ASSERT_EQUAL(64, TOF_ZONES);
    TEST_ASSERT_EQUAL(64 * sizeof(uint16_t), sizeof(scan.ranges_mm));
    TEST_ASSERT_EQUAL(64 * sizeof(uint8_t), sizeof(scan.status));
    TEST_ASSERT_EQUAL(64 * sizeof(uint16_t), sizeof(scan.ambient));
}

/* ── get_scan parameter validation ───────────────────────────────────── */

TEST_CASE("tof_sensor_get_scan rejects NULL output", "[tof_sensor]")
{
    esp_err_t err = tof_sensor_get_scan(0, NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

TEST_CASE("tof_sensor_get_scan rejects out-of-range sensor_id", "[tof_sensor]")
{
    tof_scan_t scan;
    esp_err_t err = tof_sensor_get_scan(TOF_MAX_SENSORS, &scan);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);

    err = tof_sensor_get_scan(255, &scan);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

/* ── State guards ────────────────────────────────────────────────────── */

TEST_CASE("tof_sensor_start fails before init", "[tof_sensor]")
{
    esp_err_t err = tof_sensor_start();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, err);
}

TEST_CASE("tof_sensor_stop fails before start", "[tof_sensor]")
{
    esp_err_t err = tof_sensor_stop();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, err);
}

/* ── Callback registration ───────────────────────────────────────────── */

static void dummy_cb(const tof_scan_t *scan, void *user_data)
{
    (void)scan;
    (void)user_data;
}

TEST_CASE("tof_sensor_register_callback accepts function and NULL", "[tof_sensor]")
{
    int ctx = 42;
    TEST_ASSERT_EQUAL(ESP_OK, tof_sensor_register_callback(dummy_cb, &ctx));
    TEST_ASSERT_EQUAL(ESP_OK, tof_sensor_register_callback(NULL, NULL));
}

/* ── tof_scan_t zero-init ────────────────────────────────────────────── */

TEST_CASE("tof_scan_t can be zero-initialised", "[tof_sensor]")
{
    tof_scan_t scan;
    memset(&scan, 0, sizeof(scan));

    TEST_ASSERT_EQUAL(0, scan.sensor_id);
    TEST_ASSERT_EQUAL(0, scan.temperature_degc);
    TEST_ASSERT_EQUAL(0, scan.timestamp_us);

    for (int i = 0; i < TOF_ZONES; i++) {
        TEST_ASSERT_EQUAL(0, scan.ranges_mm[i]);
        TEST_ASSERT_EQUAL(0, scan.status[i]);
        TEST_ASSERT_EQUAL(0, scan.ambient[i]);
    }
}
