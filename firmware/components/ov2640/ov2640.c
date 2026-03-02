#include "ov2640.h"

#include "esp_camera.h"
#include "esp_log.h"

static const char *TAG = "ov2640";

/* XIAO ESP32S3 Sense camera expansion board pin mapping.
   These are fixed by the Sense expansion board PCB — not configurable. */
#define CAM_PIN_PWDN    (-1)
#define CAM_PIN_RESET   (-1)
#define CAM_PIN_XCLK    10
#define CAM_PIN_SIOD    40
#define CAM_PIN_SIOC    39
#define CAM_PIN_D7      48
#define CAM_PIN_D6      11
#define CAM_PIN_D5      12
#define CAM_PIN_D4      14
#define CAM_PIN_D3      16
#define CAM_PIN_D2      18
#define CAM_PIN_D1      17
#define CAM_PIN_D0      15
#define CAM_PIN_VSYNC   38
#define CAM_PIN_HREF    47
#define CAM_PIN_PCLK    13

/* Map Kconfig choice to esp_camera framesize_t */
#if defined(CONFIG_OV2640_FRAME_SIZE_QQVGA)
#define OV2640_FRAME_SIZE  FRAMESIZE_QQVGA
#elif defined(CONFIG_OV2640_FRAME_SIZE_QVGA)
#define OV2640_FRAME_SIZE  FRAMESIZE_QVGA
#elif defined(CONFIG_OV2640_FRAME_SIZE_CIF)
#define OV2640_FRAME_SIZE  FRAMESIZE_CIF
#elif defined(CONFIG_OV2640_FRAME_SIZE_VGA)
#define OV2640_FRAME_SIZE  FRAMESIZE_VGA
#elif defined(CONFIG_OV2640_FRAME_SIZE_SVGA)
#define OV2640_FRAME_SIZE  FRAMESIZE_SVGA
#else
#define OV2640_FRAME_SIZE  FRAMESIZE_QVGA
#endif

/* Current frame held by the caller (one-at-a-time) */
static camera_fb_t *s_current_fb;

esp_err_t ov2640_init(void)
{
    camera_config_t config = {
        .pin_pwdn = CAM_PIN_PWDN,
        .pin_reset = CAM_PIN_RESET,
        .pin_xclk = CAM_PIN_XCLK,
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,
        .pin_d7 = CAM_PIN_D7,
        .pin_d6 = CAM_PIN_D6,
        .pin_d5 = CAM_PIN_D5,
        .pin_d4 = CAM_PIN_D4,
        .pin_d3 = CAM_PIN_D3,
        .pin_d2 = CAM_PIN_D2,
        .pin_d1 = CAM_PIN_D1,
        .pin_d0 = CAM_PIN_D0,
        .pin_vsync = CAM_PIN_VSYNC,
        .pin_href = CAM_PIN_HREF,
        .pin_pclk = CAM_PIN_PCLK,

        .xclk_freq_hz = 20000000,
        .ledc_timer = LEDC_TIMER_0,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = OV2640_FRAME_SIZE,
        .jpeg_quality = CONFIG_OV2640_JPEG_QUALITY,
        .fb_count = CONFIG_OV2640_FB_COUNT,
        .fb_location = CAMERA_FB_IN_PSRAM,
        /* DESIGN: GRAB_LATEST with multiple buffers ensures the publisher
           always gets the freshest frame, dropping stale ones */
        .grab_mode = (CONFIG_OV2640_FB_COUNT > 1) ? CAMERA_GRAB_LATEST
                                                   : CAMERA_GRAB_WHEN_EMPTY,
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "init failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "initialized (quality=%d, frame_size=%d, fb_count=%d)",
             CONFIG_OV2640_JPEG_QUALITY, (int)OV2640_FRAME_SIZE,
             CONFIG_OV2640_FB_COUNT);
    return ESP_OK;
}

esp_err_t ov2640_capture(ov2640_frame_t *frame)
{
    if (s_current_fb != NULL) {
        ESP_LOGW(TAG, "previous frame not released, releasing now");
        esp_camera_fb_return(s_current_fb);
        s_current_fb = NULL;
    }

    s_current_fb = esp_camera_fb_get();
    if (s_current_fb == NULL) {
        ESP_LOGW(TAG, "capture failed");
        return ESP_FAIL;
    }

    if (s_current_fb->format != PIXFORMAT_JPEG) {
        ESP_LOGE(TAG, "unexpected format %d, expected JPEG",
                 s_current_fb->format);
        esp_camera_fb_return(s_current_fb);
        s_current_fb = NULL;
        return ESP_FAIL;
    }

    frame->buf = s_current_fb->buf;
    frame->len = s_current_fb->len;
    return ESP_OK;
}

void ov2640_release(void)
{
    if (s_current_fb != NULL) {
        esp_camera_fb_return(s_current_fb);
        s_current_fb = NULL;
    }
}
