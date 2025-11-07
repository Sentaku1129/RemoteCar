#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_http_server.h"
#include "cJSON.h"

#include "bsp_key_fsm.h"
#include "bsp_config.h"
#include "bsp_joystick.h"
#include "bsp_dns.h"

#define BEEP_GPIO GPIO_NUM_4

bool g_beep_status = true;
bool g_wifi_connected = false;
int g_wifi_con_status = 0;
dev_config_t g_dev_config = {0};

QueueHandle_t beep_queue = NULL;

void beep_task(void *arg)
{
    gpio_config_t beep_cfg = {
        .pin_bit_mask = 1ULL << BEEP_GPIO,
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&beep_cfg);
    gpio_set_level(BEEP_GPIO, 0);

    beep_queue = xQueueCreate(5, sizeof(int32_t));
    beep_msg_t beep_msg = beep_none;
    while (1)
    {
        if (xQueueReceive(beep_queue, NULL, portMAX_DELAY))
        {
            if (g_beep_status)
            {
                switch (beep_msg)
                {
                case beep_none:
                    gpio_set_level(BEEP_GPIO, 0);
                    break;
                case beep_button:
                    gpio_set_level(BEEP_GPIO, 1);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    gpio_set_level(BEEP_GPIO, 0);
                    break;
                case beep_continue:
                    gpio_set_level(BEEP_GPIO, 1);
                default:
                    break;
                }
            }
        }
    }
}

static esp_err_t check_sys_config(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_SYS_CONFIG, NVS_READONLY, &nvs_handle);

    if (ret != ESP_OK)
    {
        ESP_LOGI(__func__, "error (%s) opening NVS handle!", esp_err_to_name(ret));
        goto check_sys_finally;
    }

    size_t require_size = 0;

    // ssid
    ret = nvs_get_str(nvs_handle, NVS_SYS_WIFI_SSID, g_dev_config.dev_ssid, &require_size);
    if(ret != ESP_OK)
    {
        ESP_LOGI(__func__, "read nvs fail");
        goto check_sys_finally;
    }
    if(require_size == 0)
    {
        ESP_LOGI(__func__, "no wifi ssid");
        ret = ESP_FAIL;
        goto check_sys_finally;
    }

    // pswd
    ret = nvs_get_str(nvs_handle, NVS_SYS_WIFI_SSID, g_dev_config.dev_ssid, &require_size);
    if(ret != ESP_OK)
    {
        ESP_LOGI(__func__, "read nvs fail");
        goto check_sys_finally;
    }

check_sys_finally:
    nvs_close(nvs_handle);
    return ret;
}

void save_wifi_config()
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_SYS_CONFIG, NVS_READWRITE, &nvs_handle);
    if(ret != ESP_OK)
    {
        ESP_LOGI(__func__, "open nvs error, %s", esp_err_to_name(ret));
    }

    ret = nvs_set_str(nvs_handle, NVS_SYS_WIFI_SSID, g_dev_config.dev_ssid);
    if(ret != ESP_OK)
    {
        ESP_LOGI(__func__, "open nvs error, %s", esp_err_to_name(ret));
    }
    ret = nvs_set_str(nvs_handle, NVS_SYS_WIFI_PSWD, g_dev_config.dev_pswd);
    if(ret != ESP_OK)
    {
        ESP_LOGI(__func__, "open nvs error, %s", esp_err_to_name(ret));
    }
    ret = nvs_set_str(nvs_handle, NVS_SYS_DEV_NAME, g_dev_config.dev_name);
    if(ret != ESP_OK)
    {
        ESP_LOGI(__func__, "open nvs error, %s", esp_err_to_name(ret));
    }

    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    // ESP_LOGI(__func__, "event id: %d", event_id);
    wifi_mode_t md;
    esp_wifi_get_mode(&md);
    if (md == WIFI_MODE_APSTA)
        return;

    if (event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        ESP_LOGI(__func__, "station " MACSTR " join, AID=%d", MAC2STR(event->mac), event->aid);
        // AP_STA_CONNECTED
    }
    else if (event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        ESP_LOGI(__func__, "station " MACSTR " leave, AID=%d", MAC2STR(event->mac), event->aid);
        // AP_STA_DISCONNECTED
    }
    else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        g_wifi_con_status = WIFI_EVENT_STA_DISCONNECTED;
        g_wifi_connected = false;
        ESP_LOGI(__func__, "Wi-Fi disconnected, trying to reconnect...");
        esp_wifi_connect();
        // STA_DISCONNECTED
    }
}

static void on_got_ip(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    wifi_mode_t md;
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(__func__, "Got IPv4 event: Interface \"%s\" address: " IPSTR, esp_netif_get_desc(event->esp_netif), IP2STR(&event->ip_info.ip));

    esp_wifi_get_mode(&md);

    g_wifi_con_status = IP_EVENT_STA_GOT_IP;
    g_wifi_connected = true;
    if(md == WIFI_MODE_APSTA)
    {
        save_wifi_config();
    }
}

static void web_connect_wifi(void)
{
    g_wifi_con_status = 0;

    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip, NULL);
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_OPEN,
        },
    };

    sprintf((char *)wifi_config.sta.ssid, "%s", g_dev_config.dev_ssid);
    sprintf((char *)wifi_config.sta.password, "%s", g_dev_config.dev_pswd);

    ESP_LOGI(__func__, "web sta try to connecting to %s~~~", wifi_config.sta.ssid);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_connect();
}

static void dev_connect_wifi(void)
{
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_init_config);

    esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .threshold = {
                .authmode = WIFI_AUTH_OPEN,
            },
        },
    };

    sprintf((char *)wifi_config.sta.ssid, "%s", g_dev_config.dev_ssid);
    sprintf((char *)wifi_config.sta.password, "%s", g_dev_config.dev_pswd);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
    esp_wifi_connect();

    uint8_t eth_mac[6] = {0};
    esp_wifi_get_mac(WIFI_IF_STA, eth_mac);;
    sprintf(g_dev_config.dev_mac, "%02X%02X%02X%02X%02X%02X", eth_mac[0], eth_mac[1], eth_mac[2], eth_mac[3], eth_mac[4], eth_mac[5]);
}

static void start_softap(void)
{
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t config = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&config);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = SOFT_AP_SSID,
            .ssid_len = strlen(SOFT_AP_PSWD),
            .channel = 7,
            .password = SOFT_AP_PSWD,
            .max_connection = 2,
            .authmode = WIFI_AUTH_WPA_PSK,
        },
    };

    esp_wifi_set_mode(WIFI_MODE_APSTA);
    esp_wifi_set_config(WIFI_IF_AP, &wifi_config);
    esp_wifi_start();

    uint8_t eth_mac[6];
    esp_wifi_get_mac(ESP_IF_WIFI_STA, eth_mac);
    sprintf(g_dev_config.dev_mac, "%02X%02X%02X%02X%02X%02X", eth_mac[0], eth_mac[1], eth_mac[2], eth_mac[3], eth_mac[4], eth_mac[5]);

    ESP_LOGI(__func__, "DEV STA STATION MAC : %s", g_dev_config.dev_mac);
}

static char *scan_wifi_ssid()
{
    uint16_t number = 10;
    uint16_t ap_count = 0;
    wifi_ap_record_t *ap_info = (wifi_ap_record_t *)user_malloc(10 * sizeof(wifi_ap_record_t) , MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if(ap_info == NULL)
    {
        ESP_LOGE(__func__, "ap_info allocate memory fail");
        return NULL;
    }
    memset(ap_info, 0, 10 * sizeof(wifi_ap_record_t));
    esp_wifi_scan_start(NULL, true);

    esp_wifi_scan_get_ap_num(&ap_count);
    ESP_LOGI(__func__, "Total Aps number: %u", ap_count);
    esp_wifi_scan_get_ap_records(&number, ap_info);

    cJSON *wifi_array = cJSON_CreateArray();

    for (int i = 0; (i < 10) && (i < ap_count); i++)
    {
        cJSON *item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "ssid", (const char *)ap_info[i].ssid);
        cJSON_AddNumberToObject(item, "rssi", ap_info[i].rssi);
        cJSON_AddItemToArray(wifi_array, item);
    }

    char *data = cJSON_PrintUnformatted(wifi_array);

    cJSON_Delete(wifi_array);
    return data;
}

static esp_err_t root_uri_handler(httpd_req_t *req)
{
    ESP_LOGI(__func__, "Server root");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, root_start, root_end - root_start);
    return ESP_OK;
}

static esp_err_t scan_uri_handler(httpd_req_t *req)
{
    char *body = scan_wifi_ssid();
    if(body == NULL)
    {
        ESP_LOGE(__func__, "no ap wifi");
        httpd_resp_send(req, "[]", strlen("[]"));
    }
    ESP_LOGI(__func__, "ap list: %s", body);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, body, strlen(body));
    user_free(__func__, body);
    return ESP_OK;
}

static esp_err_t config_uri_handler(httpd_req_t *req)
{
    char *body = user_malloc(128, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if(body == NULL)
    {
        ESP_LOGI(__func__, "Failed to allocate memory for request body");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"code\": \"400\"}", HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }

    char *p;
    memset(body, 0x00, 128);

    int len = httpd_req_recv(req, body, 127);
    if(len <= 0)
    {
        ESP_LOGI(__func__, "Failed to recvive request body");
        user_free(__func__, body);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"code\": \"400\"}", HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }

    ESP_LOGI(__func__, "Received request body: %s", body);

    cJSON *root = cJSON_Parse(body);
    user_free(__func__, body);
    if(root == NULL)
    {
        ESP_LOGI(__func__, "Wifi config body error");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"code\": \"400\"}", HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }
    memset(g_dev_config.dev_ssid, 0x00, sizeof(g_dev_config.dev_ssid));
    memset(g_dev_config.dev_pswd, 0x00, sizeof(g_dev_config.dev_pswd));
    memset(g_dev_config.dev_name, 0x00, sizeof(g_dev_config.dev_name));

    p = cJSON_GetObjectItem(root, "ssid") ? cJSON_GetObjectItem(root, "ssid")->valuestring : "";
    strcpy(g_dev_config.dev_ssid, p);
    p = cJSON_GetObjectItem(root, "pswd") ? cJSON_GetObjectItem(root, "pswd")->valuestring : "";
    strcpy(g_dev_config.dev_pswd, p);
    p = cJSON_GetObjectItem(root, "name") ? cJSON_GetObjectItem(root, "name")->valuestring : "";
    strcpy(g_dev_config.dev_name, p);

    ESP_LOGI(__func__, "ssid: %s, pswd: %s, name: %s", g_dev_config.dev_ssid, g_dev_config.dev_pswd, g_dev_config.dev_name);

    cJSON_Delete(root);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, "{\"code\":\"200\"}", HTTPD_RESP_USE_STRLEN);
    web_connect_wifi();

    return ESP_OK;
}

static esp_err_t status_uri_handler(httpd_req_t *req)
{
    ESP_LOGI(__func__, "status_uri");
    char buf[2] = {0};
    itoa(g_wifi_con_status, buf, 10);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, strlen(buf));

    return ESP_OK;
}

static esp_err_t generate_204_handler(httpd_req_t *req)
{
    ESP_LOGI(__func__, "generate_204");
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", dev_host_web);
    httpd_resp_send(req, "", 0);
    return ESP_OK;
}

static esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    ESP_LOGI(__func__, "http_404_error");
    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", dev_host_web);
    httpd_resp_send(req, "", 0);
    return ESP_OK;
}

static httpd_handle_t start_web_server(void)
{
    ESP_LOGI(__func__, "start web server");
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_uri_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t scan_uri = {
        .uri = "/scanap",
        .method = HTTP_GET,
        .handler = scan_uri_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t config_uri = {
        .uri = "/config",
        .method = HTTP_POST,
        .handler = config_uri_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t status_uri = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = status_uri_handler,
        .user_ctx = NULL,
    };
    httpd_uri_t generate_204_uri = {
        .uri = "/generate_204",
        .method = HTTP_GET,
        .handler = generate_204_handler,
        .user_ctx = NULL,
    };

    ESP_LOGI(__func__, "server port: %d", config.server_port);
    if(httpd_start(&server, &config) == ESP_OK)
    {
        ESP_LOGI(__func__, "Register URI handlers");
        httpd_register_uri_handler(server, &root_uri);
        httpd_register_uri_handler(server, &scan_uri);
        httpd_register_uri_handler(server, &status_uri);
        httpd_register_uri_handler(server, &config_uri);
        httpd_register_uri_handler(server, &generate_204_uri);
        httpd_register_err_handler(server, HTTPD_404_NOT_FOUND, http_404_error_handler);
    }

    return server;
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_netif_init();
    esp_event_loop_create_default();

    ret = check_sys_config();
    if (ret == ESP_OK)
    {
        dev_connect_wifi();
        while (g_wifi_connected)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        
    }
    else
    {
        start_softap();
        start_web_server();
        dns_server_config_t config = DNS_SERVER_CONFIG_SINGLE("*" /* all A queries */, "WIFI_AP_DEF" /* softAP netif ID */);
        start_dns_server(&config);
    }

    xTaskCreatePinnedToCore(button_task, "button_task", 1024 * 4, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(joystick_task, "joystick_task", 1024 * 4, NULL, 5, NULL, 0);
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(__func__, "hello world");
    }

    printf("Hello world!\n");
}
