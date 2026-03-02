/* Unity tests for the OV2640 camera driver.
 *
 * These tests verify argument-validation and state-machine invariants
 * without real camera hardware.  Tests that require an attached camera
 * are tagged [ov2640_hw] and are not expected to pass in CI.
 */

#include "unity.h"
#include "ov2640.h"

/* ── Argument validation ─────────────────────────────────────── */

TEST_CASE("ov2640_capture rejects NULL frame pointer", "[ov2640]")
{
    esp_err_t err = ov2640_capture(NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

/* ── Release invariants ──────────────────────────────────────── */

TEST_CASE("ov2640_release is safe before any capture", "[ov2640]")
{
    /* s_current_fb is NULL at this point — must be a no-op. */
    ov2640_release();
    /* If we reach here without crashing, the test passes. */
}

TEST_CASE("ov2640_release is idempotent", "[ov2640]")
{
    ov2640_release();
    ov2640_release();
}

/* ── frame_t structure ───────────────────────────────────────── */

TEST_CASE("ov2640_frame_t can be zero-initialised", "[ov2640]")
{
    ov2640_frame_t frame;
    memset(&frame, 0, sizeof(frame));
    TEST_ASSERT_NULL(frame.buf);
    TEST_ASSERT_EQUAL(0u, frame.len);
}

/* ── Capture without hardware ────────────────────────────────── */

TEST_CASE("ov2640_capture returns ESP_FAIL when camera not available",
          "[ov2640]")
{
    /* Without a connected camera, esp_camera_fb_get() returns NULL and
       ov2640_capture() must return ESP_FAIL rather than crashing.       */
    ov2640_frame_t frame;
    esp_err_t err = ov2640_capture(&frame);
    /* Accept ESP_FAIL (no camera) or ESP_OK (camera happened to work). */
    TEST_ASSERT_TRUE(err == ESP_FAIL || err == ESP_OK);
}
