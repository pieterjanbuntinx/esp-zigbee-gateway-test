/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee Gateway Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include "esp_zigbee_gateway.h"
#include "esp_coexist_internal.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_rcp_update.h"
#include "esp_spiffs.h"
#include "esp_vfs_eventfd.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include <fcntl.h>
#include <string.h>
#include <driver/gpio.h>

#include "driver/usb_serial_jtag.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_usb_serial_jtag.h"

#if (!defined ZB_MACSPLIT_HOST && defined ZB_MACSPLIT_DEVICE)
#error Only Zigbee gateway host device should be defined
#endif

static const char* TAG = "ESP_ZB_GATEWAY";

/* Note: Please select the correct console output port based on the development
 * board in menuconfig */
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
esp_err_t esp_zb_gateway_console_init(void) {
    esp_err_t ret = ESP_OK;
    /* Disable buffering on stdin */
    setvbuf(stdin, NULL, _IONBF, 0);

    /* Minicom, screen, idf_monitor send CR when ENTER key is pressed */
    esp_vfs_dev_usb_serial_jtag_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
    /* Move the caret to the beginning of the next line on '\n' */
    esp_vfs_dev_usb_serial_jtag_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);

    /* Enable non-blocking mode on stdin and stdout */
    fcntl(fileno(stdout), F_SETFL, O_NONBLOCK);
    fcntl(fileno(stdin), F_SETFL, O_NONBLOCK);

    usb_serial_jtag_driver_config_t usb_serial_jtag_config = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ret                                                    = usb_serial_jtag_driver_install(&usb_serial_jtag_config);
    esp_vfs_usb_serial_jtag_use_driver();
    esp_vfs_dev_uart_register();
    return ret;
}
#endif

#if (CONFIG_ZIGBEE_GW_AUTO_UPDATE_RCP)
static void esp_zb_gateway_update_rcp(void) {
    /* Deinit uart to transfer UART to the serial loader */
    esp_zb_macsplit_uart_deinit();
    if (esp_rcp_update() != ESP_OK) {
        esp_rcp_mark_image_verified(false);
    }
    esp_restart();
}

static void esp_zb_gateway_board_try_update(const char* rcp_version_str) {
    char version_str[RCP_VERSION_MAX_SIZE];
    if (esp_rcp_load_version_in_storage(version_str, sizeof(version_str)) == ESP_OK) {
        ESP_LOGI(TAG, "Storage RCP Version: %s", version_str);
        if (strcmp(version_str, rcp_version_str)) {
            ESP_LOGI(TAG, "*** NOT MATCH VERSION! ***");
            esp_zb_gateway_update_rcp();
        } else {
            ESP_LOGI(TAG, "*** MATCH VERSION! ***");
            esp_rcp_mark_image_verified(true);
        }
    } else {
        ESP_LOGI(TAG, "RCP firmware not found in storage, will reboot to try next image");
        esp_rcp_mark_image_verified(false);
        esp_restart();
    }
}

static esp_err_t init_spiffs(void) {
    esp_vfs_spiffs_conf_t rcp_fw_conf = {.base_path = "/rcp_fw", .partition_label = "rcp_fw", .max_files = 10, .format_if_mount_failed = false};
    esp_vfs_spiffs_register(&rcp_fw_conf);
    return ESP_OK;
}
#endif

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

#define EXTERNAL_DEVICES_ENDPOINT 5

static void bind_cb(esp_zb_zdp_status_t zdo_status, void* user_ctx) {
    bind_ctx_t* ctx = (bind_ctx_t*)user_ctx;

    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Bind response from address(0x%x), with status(%d)\n", ctx->short_addr, zdo_status);

        // configure reporting
        esp_zb_zcl_config_report_cmd_t report_cmd;
        bool                           report_change   = true;
        report_cmd.zcl_basic_cmd.dst_addr_u.addr_short = ctx->short_addr;
        report_cmd.zcl_basic_cmd.dst_endpoint          = ctx->ep_id;
        report_cmd.zcl_basic_cmd.src_endpoint          = EXTERNAL_DEVICES_ENDPOINT;

        report_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        report_cmd.clusterID    = ctx->cluster_id;

        esp_zb_zcl_config_report_record_t* records = NULL;
        switch (ctx->cluster_id) {
        case ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT:
        case ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT:
        case ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT:
            records = malloc(sizeof(esp_zb_zcl_config_report_record_t));
            if (records == NULL) {
                ESP_LOGW(TAG, "Failed to alloc mem for report config! ep: %i, nwk_id: 0x%04x\n", ctx->ep_id, ctx->short_addr);
                goto end;
            }
            records[0].min_interval      = 0;
            records[0].max_interval      = 30;
            records[0].reportable_change = &report_change;
            records[0].direction         = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;
            report_cmd.record_number     = 1;

            if (ctx->cluster_id == ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT) {
                records[0].attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;
                records[0].attrType    = ESP_ZB_ZCL_ATTR_TYPE_S16;
            } else if (ctx->cluster_id == ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT) {
                records[0].attributeID = ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID;
                records[0].attrType    = ESP_ZB_ZCL_ATTR_TYPE_U16;
            } else if (ctx->cluster_id == ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT) {
                records[0].attributeID = ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID;
                records[0].attrType    = ESP_ZB_ZCL_ATTR_TYPE_S16;
            } else if (ctx->cluster_id == ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT) {
                records[0].attributeID = ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID;
                records[0].attrType    = ESP_ZB_ZCL_ATTR_TYPE_SINGLE;
            }
        }

        report_cmd.record_field = records;

        if (records != NULL) {
            ESP_LOGI(TAG, "Report config send! dev(0x%x) endpoint(0x%x) cluster_id(0x%x)\n", ctx->short_addr, ctx->ep_id, ctx->cluster_id);
            esp_zb_zcl_config_report_cmd_req(&report_cmd);
            free(records);
        }
    }

end:
    free(ctx);
}

void simple_desc_req_cb(esp_zb_zdp_status_t zdo_status, esp_zb_af_simple_desc_1_1_t* simple_desc, void* user_ctx) {
    device_ctx_t* ctx = (device_ctx_t*)user_ctx;

    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG,
                 "Simple desc response: status(%d), device_id(%d), "
                 "app_version(%d), profile_id(0x%x), endpoint_ID(%d)\n",
                 zdo_status, simple_desc->app_device_id, simple_desc->app_device_version, simple_desc->app_profile_id, simple_desc->endpoint);

        for (int i = 0; i < (simple_desc->app_input_cluster_count + simple_desc->app_output_cluster_count); i++) {
            uint16_t cluster_id = *(simple_desc->app_cluster_list + i);
            ESP_LOGI(TAG, "Cluster ID list: 0x%x, index %i\n", cluster_id, i);

            // only bind output clusters
            if (i < simple_desc->app_input_cluster_count) {
                // continue; TODO check if only input or output clusters needed
            }

            // bind clusters
            esp_zb_zdo_bind_req_param_t bind_req;
            bind_req.cluster_id = 0xFFFF;

            bind_ctx_t* bind_ctx = malloc(sizeof(bind_ctx_t));
            bind_ctx->short_addr = ctx->short_addr;
            bind_ctx->ep_id      = simple_desc->endpoint;
            bind_ctx->cluster_id = cluster_id;

            switch (cluster_id) {
            case ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT:
            case ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT:
            case ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT:
            case ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT: {
                memcpy(&(bind_req.src_address), ctx->ieee_addr, sizeof(esp_zb_ieee_addr_t));
                bind_req.src_endp      = simple_desc->endpoint;
                bind_req.cluster_id    = cluster_id;
                bind_req.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
                esp_zb_get_long_address(bind_req.dst_address_u.addr_long);
                bind_req.dst_endp     = EXTERNAL_DEVICES_ENDPOINT;
                bind_req.req_dst_addr = ctx->short_addr;

            } break;
            default:
                break;
            }

            if (bind_req.cluster_id != 0xFFFF) {
                ESP_LOGI(TAG, "Bind request send! dev(0x%x) endpoint(0x%x) cluster_id(0x%x)\n", ctx->short_addr, simple_desc->endpoint, cluster_id);
                esp_zb_zdo_device_bind_req(&bind_req, bind_cb, bind_ctx);
            } else {
                free(bind_ctx);
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    free(ctx);
}

void active_ep_cb(esp_zb_zdp_status_t zdo_status, uint8_t ep_count, uint8_t* ep_id_list, void* user_ctx) {
    device_ctx_t* ctx = (device_ctx_t*)user_ctx;

    if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "Active endpoint response: status(%d) and endpoint count(%d)\n", zdo_status, ep_count);

        for (int i = 0; i < ep_count; i++) {
            ESP_LOGI(TAG, "Endpoint ID List: %d\n", ep_id_list[i]);

            esp_zb_zdo_simple_desc_req_param_t simple_descr_req;
            simple_descr_req.addr_of_interest = ctx->short_addr;
            simple_descr_req.endpoint         = ep_id_list[i];

            device_ctx_t* new_ctx = malloc(sizeof(device_ctx_t));
            memcpy(new_ctx, ctx, sizeof(device_ctx_t));
            esp_zb_zdo_simple_desc_req(&simple_descr_req, simple_desc_req_cb, new_ctx);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    free(ctx);
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t* signal_struct) {
    uint32_t*                                     p_sg_p           = signal_struct->p_app_signal;
    esp_err_t                                     err_status       = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t                      sig_type         = *p_sg_p;
    esp_zb_zdo_signal_device_annce_params_t*      dev_annce_params = NULL;
    esp_zb_zdo_signal_macsplit_dev_boot_params_t* rcp_version      = NULL;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_MACSPLIT_DEVICE_BOOT:
        ESP_LOGI(TAG, "Zigbee rcp device booted");
        rcp_version = (esp_zb_zdo_signal_macsplit_dev_boot_params_t*)esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGI(TAG, "Running RCP Version: %s", rcp_version->version_str);
#if (CONFIG_ZIGBEE_GW_AUTO_UPDATE_RCP)
        esp_zb_gateway_board_try_update(rcp_version->version_str);
#endif
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Start network formation");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_FORMATION);
        } else {
            ESP_LOGE(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }
        break;
    case ESP_ZB_BDB_SIGNAL_FORMATION:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t ieee_address;
            esp_zb_get_long_address(ieee_address);
            ESP_LOGI(TAG,
                     "Formed network successfully (ieee_address: "
                     "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, "
                     "Channel:%d, Short Address: 0x%04hx)",
                     ieee_address[7], ieee_address[6], ieee_address[5], ieee_address[4], ieee_address[3], ieee_address[2], ieee_address[1], ieee_address[0], esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGI(TAG, "Restart network formation (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_FORMATION, 1000);
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Network steering started");
        }
        break;
    case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
        dev_annce_params = (esp_zb_zdo_signal_device_annce_params_t*)esp_zb_app_signal_get_params(p_sg_p);
        ESP_LOGI(TAG, "New device commissioned or rejoined (short: 0x%04hx)", dev_annce_params->device_short_addr);

        device_ctx_t* ctx = malloc(sizeof(device_ctx_t));
        memcpy(ctx->ieee_addr, dev_annce_params->ieee_addr, sizeof(esp_zb_ieee_addr_t));
        ctx->short_addr = dev_annce_params->device_short_addr;

        // get EP list
        esp_zb_zdo_active_ep_req_param_t active_ep_req = {};
        active_ep_req.addr_of_interest                 = ctx->short_addr;
        esp_zb_zdo_active_ep_req(&active_ep_req, active_ep_cb, ctx);

        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}

void rcp_error_handler(uint8_t connect_timeout) {
    ESP_LOGI(TAG, "RCP connection failed timeout:%d seconds", connect_timeout);
#if (CONFIG_ZIGBEE_GW_AUTO_UPDATE_RCP)
    ESP_LOGI(TAG, "Timeout! Re-flashing RCP");
    esp_zb_gateway_update_rcp();
#endif
}

static void esp_zb_task(void* pvParameters) {
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZC_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_secur_link_key_exchange_required_set(false);

    /* create endpoint list */
    esp_zb_ep_list_t* esp_zb_ep_list = esp_zb_ep_list_create();

    /* create cluster list for external device endpoint */
    esp_zb_cluster_list_t* esp_zb_external_devices_cluster_list = esp_zb_zcl_cluster_list_create();

    /* create ias zone cluster with client parameters */
    esp_zb_ias_zone_cluster_cfg_t zone_cfg = {
        .zone_state   = 0,
        .zone_type    = ESP_ZB_ZCL_IAS_ZONE_ZONETYPE_CONTACT_SWITCH,
        .zone_status  = 0,
        .ias_cie_addr = ESP_ZB_ZCL_ZONE_IAS_CIE_ADDR_DEFAULT,
        .zone_id      = 0xFF,
    };
    esp_zb_attribute_list_t* esp_zb_ias_zone_cluster = esp_zb_ias_zone_cluster_create(&zone_cfg);
    esp_zb_cluster_list_add_ias_zone_cluster(esp_zb_external_devices_cluster_list, esp_zb_ias_zone_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    esp_zb_temperature_meas_cluster_cfg_t temp_meas_cfg            = {0};
    esp_zb_attribute_list_t*              esp_zb_temp_meas_cluster = esp_zb_temperature_meas_cluster_create(&temp_meas_cfg);
    esp_zb_cluster_list_add_temperature_meas_cluster(esp_zb_external_devices_cluster_list, esp_zb_temp_meas_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    esp_zb_humidity_meas_cluster_cfg_t humid_meas_cfg            = {0};
    esp_zb_attribute_list_t*           esp_zb_humid_meas_cluster = esp_zb_humidity_meas_cluster_create(&humid_meas_cfg);
    esp_zb_cluster_list_add_humidity_meas_cluster(esp_zb_external_devices_cluster_list, esp_zb_humid_meas_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    esp_zb_fan_control_cluster_cfg_t fan_ctl_cfg            = {0};
    esp_zb_attribute_list_t*         esp_zb_fan_ctl_cluster = esp_zb_fan_control_cluster_create(&fan_ctl_cfg);
    esp_zb_cluster_list_add_fan_control_cluster(esp_zb_external_devices_cluster_list, esp_zb_fan_ctl_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    esp_zb_pressure_meas_cluster_cfg_t press_meas_cfg            = {0};
    esp_zb_attribute_list_t*           esp_zb_press_meas_cluster = esp_zb_pressure_meas_cluster_create(&press_meas_cfg);
    esp_zb_cluster_list_add_pressure_meas_cluster(esp_zb_external_devices_cluster_list, esp_zb_press_meas_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    esp_zb_carbon_dioxide_measurement_cluster_cfg_t co2_meas_cfg            = {0};
    esp_zb_attribute_list_t*                        esp_zb_co2_meas_cluster = esp_zb_carbon_dioxide_measurement_cluster_create(&co2_meas_cfg);
    esp_zb_cluster_list_add_carbon_dioxide_measurement_cluster(esp_zb_external_devices_cluster_list, esp_zb_co2_meas_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    /* create endpoint for external devices */
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_external_devices_cluster_list, EXTERNAL_DEVICES_ENDPOINT, ESP_ZB_AF_HA_PROFILE_ID, ESP_ZB_HA_TEST_DEVICE_ID);

    /* register endpoints */
    esp_zb_device_register(esp_zb_ep_list);

    ESP_ERROR_CHECK(esp_zb_start(false));
#if (CONFIG_ZB_RADIO_MACSPLIT_UART)
    esp_zb_add_rcp_failure_cb(rcp_error_handler);
#endif
    esp_zb_main_loop_iteration();
    esp_rcp_update_deinit();
    vTaskDelete(NULL);
}

void app_main(void) {
    gpio_reset_pin(HOST_RESET_PIN_TO_RCP_RESET);
    gpio_set_direction(HOST_RESET_PIN_TO_RCP_RESET, GPIO_MODE_OUTPUT);
    gpio_set_level(HOST_RESET_PIN_TO_RCP_RESET, 1);

    gpio_reset_pin(HOST_BOOT_PIN_TO_RCP_BOOT);
    gpio_set_direction(HOST_BOOT_PIN_TO_RCP_BOOT, GPIO_MODE_OUTPUT);
    gpio_set_level(HOST_BOOT_PIN_TO_RCP_BOOT, 1);

    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(HOST_RESET_PIN_TO_RCP_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(HOST_RESET_PIN_TO_RCP_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };

    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    ESP_ERROR_CHECK(esp_zb_gateway_console_init());
#endif

#if CONFIG_EXAMPLE_CONNECT_WIFI
    ESP_ERROR_CHECK(example_connect());
#if CONFIG_ESP_COEX_SW_COEXIST_ENABLE
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
    coex_enable();
    coex_schm_status_bit_set(1, 1);
#else
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
#endif
#endif
#if (CONFIG_ZIGBEE_GW_AUTO_UPDATE_RCP)
    esp_rcp_update_config_t rcp_update_config = ESP_ZB_RCP_UPDATE_CONFIG();
    ESP_ERROR_CHECK(init_spiffs());
    ESP_ERROR_CHECK(esp_rcp_update_init(&rcp_update_config));
#endif
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
