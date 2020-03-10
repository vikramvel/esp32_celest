/*
 * @file main.h
 *  @brief Main Common Header file for wifi driver functions.
 *
 *  This contains the prototypes for the app_main
 *  driver and eventually any macros, constants,
 *  or global variables you will need.
 *
 *  @author Vikram vel (svikramvel@gmail.com)
 *
 */

#ifndef __MAIN_H__
#define __MAIN_H__

/* ==================================================================== */
/* ========================== include files =========================== */
/* ==================================================================== */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_wpa2.h"
#include "esp_event_loop.h"
#include "tcpip_adapter.h"
#include "esp_log.h"
#include <lwip/sockets.h>
#include "soc/uart_struct.h"
#include "driver/uart.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "esp_types.h"
#include <nvs.h>
#include "esp_timer.h"
#include "ring_buffer.h"
#include "esp_flash_partitions.h"
#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "lwip/inet.h"
#include "lwip/ip4_addr.h"
#include "lwip/dns.h"
#include "ping/ping.h"
#include "esp_ping.h"
#include "include/version.h"
#include "include/counter.h"
#include "mdns.h"
#include "include/dns_server.h"

/* ==================================================================== */
/* ============================ constants ============================= */
/* ==================================================================== */

/* Set to 1 for debug build */
//  #define Debug_Enable 1


/* 
 * Long version name tag
 * <module_name>-vMAJOR.MINOR.PATCH(-METADATA.STAGE+BUILD)
 */
#define LONG_VERSION	  	"ESP32F_FW_VER_1.0.1.0-Alpha_V.1"

#define ESP_IDF_VERSION		"v3.2"

#define HTTP_SERVER_START_BIT_0 ( 1 << 0 )


/* ==================================================================== */
/* ======================== global variables ========================== */
/* ==================================================================== */

/* Structures and Enum */

/* Variable Declaration */ 

/* Biblical reference */


/* ==================================================================== */
/* ==================== function prototypes =========================== */
/* ==================================================================== */

/* Function Declaration */


#endif
