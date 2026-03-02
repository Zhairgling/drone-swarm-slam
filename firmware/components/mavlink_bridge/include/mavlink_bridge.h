#pragma once

#include <esp_err.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>

/* Number of executor handles this component adds (1 subscription). */
#define MAVLINK_BRIDGE_NUM_HANDLES 1

/* Initialise UART for MAVLink communication with the flight controller.
   Uses CONFIG_MAVLINK_UART_* Kconfig values for pin/baud configuration. */
esp_err_t mavlink_bridge_init(void);

/* Create the micro-ROS publisher (mavlink/from_fc) and subscriber
   (mavlink/to_fc) on the given node, and register the subscriber with
   the executor.  Call after mavlink_bridge_init(). */
esp_err_t mavlink_bridge_create(rcl_node_t *node, rclc_executor_t *executor);

/* Start the UART reader FreeRTOS task.  Incoming MAVLink frames are
   published to the from_fc topic.  Call after mavlink_bridge_create(). */
esp_err_t mavlink_bridge_start(void);
