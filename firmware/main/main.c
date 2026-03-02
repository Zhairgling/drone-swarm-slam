#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microxrcedds_c/config.h>
#include <uros_network_interfaces.h>

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

    /* DESIGN: executor handles = 0 for scaffold; will grow as publishers/
       subscriptions are added in subsequent PRs (ToF, camera, MAVLink) */
    const unsigned int num_handles = 1;
    RCCHECK(
        rclc_executor_init(&executor, &support.context, num_handles, &allocator));

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

    /* DESIGN: 16KB stack for micro-ROS task; sufficient for node + executor.
       Will need increase when ToF/camera publishers are added. */
    xTaskCreate(micro_ros_task, "uros", 16384, NULL, 5, NULL);
}
