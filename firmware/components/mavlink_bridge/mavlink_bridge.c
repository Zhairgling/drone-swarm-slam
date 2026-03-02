#include "mavlink_bridge.h"
#include "mavlink_parser.h"

#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/u_int8_multi_array.h>

static const char *TAG = "mavlink_bridge";

/* UART buffer sizes. RX needs headroom for bursts; TX can be smaller
   since we write complete frames at once. */
#define UART_RX_BUF_SIZE 1024
#define UART_TX_BUF_SIZE 512

/* Stack for the UART reader task. 4 KB is sufficient for UART reads,
   parser state, and a single rcl_publish call. */
#define UART_TASK_STACK_SIZE 4096
#define UART_TASK_PRIORITY   4

/* Chunk size for uart_read_bytes per iteration. */
#define UART_READ_CHUNK 128

/* ── static state ─────────────────────────────────────────────── */

static rcl_publisher_t  pub_from_fc;
static rcl_subscription_t sub_to_fc;

/* Publisher message — statically allocated. */
static std_msgs__msg__UInt8MultiArray pub_msg;
static uint8_t pub_msg_buf[MAVLINK_MAX_FRAME_LEN];

/* Subscriber message — statically allocated. */
static std_msgs__msg__UInt8MultiArray sub_msg;
static uint8_t sub_msg_buf[MAVLINK_MAX_FRAME_LEN];

static mavlink_parser_t parser;

/* ── subscriber callback ─────────────────────────────────────── */

static void mavlink_to_fc_cb(const void *msg_in)
{
    const std_msgs__msg__UInt8MultiArray *msg =
        (const std_msgs__msg__UInt8MultiArray *)msg_in;

    if (msg->data.size == 0 || msg->data.size > MAVLINK_MAX_FRAME_LEN) {
        ESP_LOGW(TAG, "to_fc: invalid frame size %zu", msg->data.size);
        return;
    }

    int written = uart_write_bytes(CONFIG_MAVLINK_UART_NUM,
                                   msg->data.data, msg->data.size);
    if (written < 0) {
        ESP_LOGE(TAG, "uart_write_bytes failed");
    }
}

/* ── UART reader task ────────────────────────────────────────── */

static void mavlink_uart_task(void *arg)
{
    uint8_t rx_buf[UART_READ_CHUNK];

    ESP_LOGI(TAG, "UART reader task started (UART%d, %d baud)",
             CONFIG_MAVLINK_UART_NUM, CONFIG_MAVLINK_UART_BAUD);

    while (true) {
        int len = uart_read_bytes(CONFIG_MAVLINK_UART_NUM, rx_buf,
                                  sizeof(rx_buf), pdMS_TO_TICKS(20));
        if (len <= 0) {
            continue;
        }

        for (int i = 0; i < len; i++) {
            mavlink_parser_result_t res =
                mavlink_parser_parse_byte(&parser, rx_buf[i]);

            if (res == MAVLINK_PARSER_COMPLETE) {
                const uint8_t *frame;
                size_t frame_len;
                mavlink_parser_get_frame(&parser, &frame, &frame_len);

                memcpy(pub_msg.data.data, frame, frame_len);
                pub_msg.data.size = frame_len;

                rcl_ret_t rc = rcl_publish(&pub_from_fc, &pub_msg, NULL);
                if (rc != RCL_RET_OK) {
                    ESP_LOGW(TAG, "publish from_fc failed: %d", (int)rc);
                }

                mavlink_parser_init(&parser);
            }
        }
    }

    vTaskDelete(NULL);
}

/* ── message initialisation ──────────────────────────────────── */

static void init_messages(void)
{
    /* DESIGN: static allocation avoids heap fragmentation on the ESP32.
       Layout fields are unused — we transport raw byte sequences only. */
    pub_msg.data.data     = pub_msg_buf;
    pub_msg.data.size     = 0;
    pub_msg.data.capacity = sizeof(pub_msg_buf);
    pub_msg.layout.dim.data     = NULL;
    pub_msg.layout.dim.size     = 0;
    pub_msg.layout.dim.capacity = 0;
    pub_msg.layout.data_offset  = 0;

    sub_msg.data.data     = sub_msg_buf;
    sub_msg.data.size     = 0;
    sub_msg.data.capacity = sizeof(sub_msg_buf);
    sub_msg.layout.dim.data     = NULL;
    sub_msg.layout.dim.size     = 0;
    sub_msg.layout.dim.capacity = 0;
    sub_msg.layout.data_offset  = 0;
}

/* ── public API ──────────────────────────────────────────────── */

esp_err_t mavlink_bridge_init(void)
{
    const uart_config_t uart_cfg = {
        .baud_rate  = CONFIG_MAVLINK_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(CONFIG_MAVLINK_UART_NUM,
                                        UART_RX_BUF_SIZE, UART_TX_BUF_SIZE,
                                        0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(CONFIG_MAVLINK_UART_NUM, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(CONFIG_MAVLINK_UART_NUM,
                                 CONFIG_MAVLINK_UART_TX_PIN,
                                 CONFIG_MAVLINK_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    mavlink_parser_init(&parser);
    init_messages();

    ESP_LOGI(TAG, "UART%d initialised (TX=%d, RX=%d, baud=%d)",
             CONFIG_MAVLINK_UART_NUM, CONFIG_MAVLINK_UART_TX_PIN,
             CONFIG_MAVLINK_UART_RX_PIN, CONFIG_MAVLINK_UART_BAUD);

    return ESP_OK;
}

esp_err_t mavlink_bridge_create(rcl_node_t *node, rclc_executor_t *executor)
{
    const rosidl_message_type_support_t *type =
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8MultiArray);

    /* DESIGN: reliable QoS for MAVLink topics — frames must not be dropped
       between the ground station and the flight controller. */
    const rmw_qos_profile_t qos = rmw_qos_profile_default;

    /* Publisher: mavlink/from_fc (FC → ground) */
    rcl_ret_t rc = rclc_publisher_init(
        &pub_from_fc, node, type, "mavlink/from_fc", &qos);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "failed to create from_fc publisher: %d", (int)rc);
        return ESP_FAIL;
    }

    /* Subscriber: mavlink/to_fc (ground → FC) */
    rc = rclc_subscription_init(
        &sub_to_fc, node, type, "mavlink/to_fc", &qos);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "failed to create to_fc subscriber: %d", (int)rc);
        return ESP_FAIL;
    }

    rc = rclc_executor_add_subscription(
        executor, &sub_to_fc, &sub_msg, &mavlink_to_fc_cb, ON_NEW_DATA);
    if (rc != RCL_RET_OK) {
        ESP_LOGE(TAG, "failed to add to_fc subscription to executor: %d",
                 (int)rc);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "publisher/subscriber created");
    return ESP_OK;
}

esp_err_t mavlink_bridge_start(void)
{
    BaseType_t ret = xTaskCreate(mavlink_uart_task, "mavlink",
                                 UART_TASK_STACK_SIZE, NULL,
                                 UART_TASK_PRIORITY, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "failed to create UART reader task");
        return ESP_FAIL;
    }
    return ESP_OK;
}
