#include <math.h>
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
#include <sensor_msgs/msg/point_cloud2.h>
#include <sensor_msgs/msg/point_field.h>
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

/* ── Constants ──────────────────────────────────────────────────────── */

#define TOF_TIMER_MS       66   /* ~15 Hz */
#define CAMERA_TIMER_MS    100  /* ~10 Hz */

/* Executor handles: 2 timers (ToF + camera) + MAVLink subscriber */
#define EXECUTOR_HANDLES   (2 + MAVLINK_BRIDGE_NUM_HANDLES)

/* PointCloud2 layout: sensors * 64 zones, each point = (x,y,z) float32 */
#define TOF_GRID_SIZE      8
#define TOF_FOV_DEG        45.0f
#define TOF_POINT_COUNT    (CONFIG_TOF_SENSOR_COUNT * TOF_ZONES)
#define TOF_POINT_STEP     12  /* 3 * sizeof(float) */

/* DESIGN: 64KB JPEG buffer covers QVGA/CIF typical output.
   Allocated in PSRAM to preserve internal SRAM for stack/heap. */
#define CAMERA_JPEG_BUF_SIZE (64 * 1024)

/* VL53L8CX angular resolution */
#define TOF_PIXEL_ANG      ((TOF_FOV_DEG * (float)M_PI / 180.0f) / TOF_GRID_SIZE)

/* ── Static message storage ────────────────────────────────────────── */

static rcl_publisher_t tof_pub;
static rcl_publisher_t camera_pub;

static sensor_msgs__msg__PointCloud2 tof_msg;
static sensor_msgs__msg__CompressedImage camera_msg;

/* PointCloud2 backing buffers */
static uint8_t tof_data_buf[TOF_POINT_COUNT * TOF_POINT_STEP];
static sensor_msgs__msg__PointField tof_fields[3];
static char tof_frame_id[32];
static char tof_field_names[3][2] = {"x", "y", "z"};

/* DESIGN: Sensor yaw angles (radians) relative to the drone body frame.
   Front = +X, Right = -Y, Back = -X, Left = +Y. See docs/hardware_wiring.md
   "Sensor Placement" diagram. */
static const float sensor_yaw_rad[TOF_MAX_SENSORS] = {
    0.0f,                /*  Front (#0): +X              */
    -(float)M_PI / 2.0f, /*  Right (#1): +X rotated -90  */
    (float)M_PI,         /*  Back  (#2): +X rotated 180  */
    (float)M_PI / 2.0f,  /*  Left  (#3): +X rotated +90  */
};

/* ── Helpers ────────────────────────────────────────────────────────── */

static void stamp_now(builtin_interfaces__msg__Time *stamp)
{
    int64_t us = esp_timer_get_time();
    stamp->sec = (int32_t)(us / 1000000);
    stamp->nanosec = (uint32_t)((us % 1000000) * 1000);
}

/* ── Message initialisation ─────────────────────────────────────────── */

static void init_tof_msg(void)
{
    memset(&tof_msg, 0, sizeof(tof_msg));

    snprintf(tof_frame_id, sizeof(tof_frame_id), "drone_%d_tof",
             CONFIG_DRONE_ID);
    tof_msg.header.frame_id.data = tof_frame_id;
    tof_msg.header.frame_id.size = strlen(tof_frame_id);
    tof_msg.header.frame_id.capacity = sizeof(tof_frame_id);

    tof_msg.height = 1;
    tof_msg.width = TOF_POINT_COUNT;
    tof_msg.is_bigendian = false;
    tof_msg.point_step = TOF_POINT_STEP;
    tof_msg.row_step = TOF_POINT_COUNT * TOF_POINT_STEP;
    tof_msg.is_dense = false;

    /* PointField descriptors: x, y, z (FLOAT32) */
    for (int i = 0; i < 3; i++) {
        tof_fields[i].name.data = tof_field_names[i];
        tof_fields[i].name.size = 1;
        tof_fields[i].name.capacity = 2;
        tof_fields[i].offset = (uint32_t)(i * sizeof(float));
        tof_fields[i].datatype = sensor_msgs__msg__PointField__FLOAT32;
        tof_fields[i].count = 1;
    }
    tof_msg.fields.data = tof_fields;
    tof_msg.fields.size = 3;
    tof_msg.fields.capacity = 3;

    tof_msg.data.data = tof_data_buf;
    tof_msg.data.size = 0;
    tof_msg.data.capacity = sizeof(tof_data_buf);
}

/* ── ToF → PointCloud2 conversion ───────────────────────────────────── */

/* Write a float into the PointCloud2 data buffer at the given byte offset.
   Uses memcpy to avoid strict-aliasing violations (uint8_t* → float*). */
static void write_float(uint8_t *buf, size_t offset, float value)
{
    memcpy(&buf[offset], &value, sizeof(float));
}

/* DESIGN: Convert N × 8x8 depth grids into a single unstructured PointCloud2
   in the drone body frame. Each pixel's angle is derived from its grid
   position and the VL53L8CX 45-degree FoV. Points with ranges_mm == 0
   are kept as (0,0,0) and is_dense is set to false so downstream filters
   can discard them. */
static void tof_to_pointcloud(const tof_scan_t scans[], int num_sensors)
{
    int idx = 0;

    for (int s = 0; s < num_sensors; s++) {
        float yaw = sensor_yaw_rad[s];
        float cos_yaw = cosf(yaw);
        float sin_yaw = sinf(yaw);

        for (int row = 0; row < TOF_GRID_SIZE; row++) {
            float vert_ang = ((float)row - 3.5f) * TOF_PIXEL_ANG;
            for (int col = 0; col < TOF_GRID_SIZE; col++) {
                float horiz_ang = ((float)col - 3.5f) * TOF_PIXEL_ANG;
                uint16_t dist_mm =
                    scans[s].ranges_mm[row * TOF_GRID_SIZE + col];
                size_t off = (size_t)idx * TOF_POINT_STEP;

                if (dist_mm == 0) {
                    write_float(tof_data_buf, off + 0, 0.0f);
                    write_float(tof_data_buf, off + 4, 0.0f);
                    write_float(tof_data_buf, off + 8, 0.0f);
                } else {
                    float dist_m = (float)dist_mm / 1000.0f;

                    /* Point in sensor frame (+X = straight ahead) */
                    float sx = dist_m * cosf(vert_ang) * cosf(horiz_ang);
                    float sy = dist_m * cosf(vert_ang) * sinf(horiz_ang);
                    float sz = dist_m * sinf(vert_ang);

                    /* Rotate to body frame by sensor yaw */
                    write_float(tof_data_buf, off + 0,
                                sx * cos_yaw - sy * sin_yaw);
                    write_float(tof_data_buf, off + 4,
                                sx * sin_yaw + sy * cos_yaw);
                    write_float(tof_data_buf, off + 8, sz);
                }
                idx++;
            }
        }
    }
    tof_msg.data.size = (size_t)(idx * TOF_POINT_STEP);
}

/* ── Timer callbacks ────────────────────────────────────────────────── */

// cppcheck-suppress constParameterCallback
static void tof_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) {
        return;
    }

    tof_scan_t scans[TOF_MAX_SENSORS];
    for (int i = 0; i < CONFIG_TOF_SENSOR_COUNT; i++) {
        esp_err_t err = tof_sensor_get_scan((uint8_t)i, &scans[i]);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "ToF sensor %d read failed: %s", i,
                     esp_err_to_name(err));
        }
    }

    stamp_now(&tof_msg.header.stamp);
    tof_to_pointcloud(scans, CONFIG_TOF_SENSOR_COUNT);
    RCSOFTCHECK(rcl_publish(&tof_pub, &tof_msg, NULL));
}

// cppcheck-suppress constParameterCallback
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

/* ── micro-ROS task ─────────────────────────────────────────────────── */

static void micro_ros_task(void *arg)
{
    (void)arg;
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

    /* ── Init static ToF message buffers ── */
    init_tof_msg();

    /* ── Publishers (best-effort QoS for sensor streams) ── */
    RCCHECK(rclc_publisher_init_best_effort(
        &tof_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, PointCloud2),
        "tof/pointcloud"));
    ESP_LOGI(TAG, "publisher: %s/tof/pointcloud (best_effort)", ns);

    RCCHECK(rclc_publisher_init_best_effort(
        &camera_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, CompressedImage),
        "camera/compressed"));
    ESP_LOGI(TAG, "publisher: %s/camera/compressed (best_effort)", ns);

    /* Pre-allocate CompressedImage message fields in PSRAM */
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

    /* ── Timers: 66 ms (~15 Hz) ToF, 100 ms (~10 Hz) camera ── */
    rcl_timer_t tof_timer;
    RCCHECK(rclc_timer_init_default(&tof_timer, &support,
                                    RCL_MS_TO_NS(TOF_TIMER_MS),
                                    tof_timer_callback));
    rcl_timer_t camera_timer;
    RCCHECK(rclc_timer_init_default(&camera_timer, &support,
                                    RCL_MS_TO_NS(CAMERA_TIMER_MS),
                                    camera_timer_cb));

    /* ── Executor ── */
    RCCHECK(rclc_executor_init(&executor, &support.context, EXECUTOR_HANDLES,
                               &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &tof_timer));
    RCCHECK(rclc_executor_add_timer(&executor, &camera_timer));

    /* MAVLink serial bridge (UART ↔ micro-ROS) */
    RCCHECK(mavlink_bridge_create(&node, &executor));
    RCCHECK(mavlink_bridge_start());

    ESP_LOGI(TAG, "spinning (tof @%d ms, camera @%d ms, mavlink)...",
             TOF_TIMER_MS, CAMERA_TIMER_MS);

    while (true) {
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* Unreachable in normal operation; included for completeness */
    RCCHECK(rclc_executor_fini(&executor));
    RCCHECK(rcl_publisher_fini(&tof_pub, &node));
    RCCHECK(rcl_publisher_fini(&camera_pub, &node));
    RCCHECK(rcl_node_fini(&node));
    RCCHECK(rclc_support_fini(&support));
    vTaskDelete(NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "drone-swarm firmware v0.3.0 (drone_id=%d)", CONFIG_DRONE_ID);

    /* NVS required by WiFi driver */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* DESIGN: WiFi STA handled by micro-ROS component via its Kconfig
       (SSID/password set in menuconfig, not hardcoded). */
    ESP_ERROR_CHECK(uros_network_interface_initialize());

    /* MAVLink UART must be ready before the micro-ROS task creates the
       publisher/subscriber that uses it. */
    ESP_ERROR_CHECK(mavlink_bridge_init());

    /* Initialize OV2640 camera before starting micro-ROS task */
    ESP_ERROR_CHECK(ov2640_init());

    /* Initialise ToF sensors (I2C bus + LPn sequencing + firmware load) */
    ESP_ERROR_CHECK(tof_sensor_init());
    ESP_ERROR_CHECK(tof_sensor_start());

    /* DESIGN: 24KB stack for micro-ROS task. Increased from 16KB (scaffold)
       to accommodate ToF/camera publishers, timer callbacks, MAVLink bridge,
       and the tof_to_pointcloud conversion. */
    xTaskCreate(micro_ros_task, "uros", 24576, NULL, 5, NULL);
}
