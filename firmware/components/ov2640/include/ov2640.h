#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

/**
 * JPEG frame captured from the OV2640 camera.
 * Valid until ov2640_release() is called.
 */
typedef struct {
    const uint8_t *buf;   /**< Pointer to JPEG data */
    size_t len;           /**< JPEG data length in bytes */
} ov2640_frame_t;

/**
 * Initialize the OV2640 camera on XIAO ESP32S3 Sense.
 *
 * Uses Kconfig settings for resolution, JPEG quality, and buffer count.
 * Must be called once before ov2640_capture().
 *
 * @return ESP_OK on success.
 */
esp_err_t ov2640_init(void);

/**
 * Capture a single JPEG frame.
 *
 * Only one frame can be held at a time. Call ov2640_release() before
 * capturing the next frame.
 *
 * @param[out] frame  Filled with JPEG data pointer and length on success.
 * @return ESP_OK on success, ESP_FAIL on capture failure.
 */
esp_err_t ov2640_capture(ov2640_frame_t *frame);

/**
 * Release the current frame buffer back to the camera driver.
 */
void ov2640_release(void);
