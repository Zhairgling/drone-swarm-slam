#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "tof_sensor.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/compressed_image.h>
#include <uros_network_interfaces.h>

#include "mavlink_bridge.h"
#include "ov2640.h"

static const char *TAG = "drone_onboard";

/* DESIGN: abort on micro-ROS init errors (same philosophy as ESP_ERROR_CHECK) */
#define RCCHECK(fn)                                                            \
    do {                                                                       \
        rcl_ret_t rc = (fn);                                                   \
        if (rc != RCL_RET_OK) {                                                \
            ESP_LOGE(TAG, "micro-ROS error %d at %s:%d", (int)rc, __FILE__,    \
                     __LINE__);                                                \
            vTaskDelete(NULL);                                                 \
        }                                                                      \
    } while (0)

/* DESIGN: log-and-continue for non-critical micro-ROS calls in spin loop */
#define RCSOFTCHECK(fn)                                                        \
    do {                                                                       \
        rcl_ret_t rc = (fn);                                                   \
        if (rc != RCL_RET_OK) {                                                \
            ESP_LOGW(TAG, "micro-ROS warning %d at %s:%d", (int)rc, __FILE__,  \
                     __LINE__);                                                \
        }                                                                      \
    } while (0)

/* DESIGN: 64KB JPEG buffer covers QVGA/CIF typical output.
   Allocated in PSRAM to preserve internal SRAM for stack/heap. */
#define CAMERA_JPEG_BUF_SIZE (64 * 1024)

static rcl_publisher_t camera_pub;
static sensor_msgs__msg__CompressedImage camera_msg;

static void camera_timer_cb(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) {
        return;
    }

    ov2640_frame_t frame;
    if (ov2640_capture(&frame) != ESP_OK) {
        return;
    }

    if (frame.len <= camera_msg.data.capacity) {
        memcpy(camera_msg.data.data, frame.buf, frame.len);
        camera_msg.data.size = frame.len;

        int64_t now_us = esp_timer_get_time();
        camera_msg.header.stamp.sec = (int32_t)(now_us / 1000000);
        camera_msg.header.stamp.nanosec =
            (uint32_t)((now_us % 1000000) * 1000);

        RCSOFTCHECK(rcl_publish(&camera_pub, &camera_msg, NULL));
    } else {
        ESP_LOGW(TAG, "JPEG too large (%u > %u)", (unsigned)frame.len,
                 (unsigned)camera_msg.data.capacity);
    }

    ov2640_release();
}

static void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
    rmw_init_options_t *rmw_options =
        rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(
        CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
#endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options,
                                           &allocator));

    /* Node name: drone_N_onboard, namespace: drone_N */
    char node_name[32];
    snprintf(node_name, sizeof(node_name), "drone_%d_onboard", CONFIG_DRONE_ID);

    char ns[32];
    snprintf(ns, sizeof(ns), "drone_%d", CONFIG_DRONE_ID);

    RCCHECK(rclc_node_init_default(&node, node_name, ns, &support));
    ESP_LOGI(TAG, "node '%s/%s' created", ns, node_name);

    /* --- Camera publisher (best-effort QoS for sensor data) --- */
    RCCHECK(rclc_publisher_init_best_effort(
        &camera_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
        "camera/compressed"));

    /* Pre-allocate CompressedImage message fields */
    char frame_id[32];
    snprintf(frame_id, sizeof(frame_id), "drone_%d_camera", CONFIG_DRONE_ID);
    rosidl_runtime_c__String__assign(&camera_msg.header.frame_id, frame_id);
    rosidl_runtime_c__String__assign(&camera_msg.format, "jpeg");

    camera_msg.data.data =
        (uint8_t *)heap_caps_malloc(CAMERA_JPEG_BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (camera_msg.data.data == NULL) {
        ESP_LOGE(TAG, "failed to allocate JPEG buffer in PSRAM");
        vTaskDelete(NULL);
    }
    camera_msg.data.capacity = CAMERA_JPEG_BUF_SIZE;
    camera_msg.data.size = 0;

    /* --- Camera timer (10 Hz = 100 ms per CLAUDE.md spec) --- */
    rcl_timer_t camera_timer;
    RCCHECK(rclc_timer_init_default(&camera_timer, &support,
                                    RCL_MS_TO_NS(100), camera_timer_cb));

    /* Executor handles: camera timer (1) + MAVLink subscriber.
       Will grow as ToF publisher is added. */
    const unsigned int num_handles = 1 + MAVLINK_BRIDGE_NUM_HANDLES;
    RCCHECK(
        rclc_executor_init(&executor, &support.context, num_handles, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &camera_timer));

    /* MAVLink serial bridge (UART ↔ micro-ROS) */
    RCCHECK(mavlink_bridge_create(&node, &executor));
    RCCHECK(mavlink_bridge_start());

    ESP_LOGI(TAG, "spinning...");
    while (true) {
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    RCCHECK(rclc_executor_fini(&executor));
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rclc_support_fini(&support));
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "drone-swarm firmware v0.1.0 (drone_id=%d)", CONFIG_DRONE_ID);

    /* NVS required by WiFi driver */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* DESIGN: WiFi STA handled by micro-ROS component via its Kconfig
       (SSID/password set in menuconfig, not hardcoded). Will be replaced
       by custom WiFi mesh component when multi-drone networking is added. */
    ESP_ERROR_CHECK(uros_network_interface_initialize());

    /* MAVLink UART must be ready before the micro-ROS task creates the
       publisher/subscriber that uses it. */
    ESP_ERROR_CHECK(mavlink_bridge_init());

    /* Initialize OV2640 camera before starting micro-ROS task */
    ESP_ERROR_CHECK(ov2640_init());

    /* Initialise ToF sensors (I2C bus + LPn sequencing + firmware load) */
    ESP_ERROR_CHECK(tof_sensor_init());
    ESP_ERROR_CHECK(tof_sensor_start());

    /* DESIGN: 24KB stack for micro-ROS task; increased from 16KB to
       accommodate camera publisher and JPEG buffer operations. */
    xTaskCreate(micro_ros_task, "uros", 24576, NULL, 5, NULL);
}
