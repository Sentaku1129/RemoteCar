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
#include "mqtt_client.h"
#include "esp_http_client.h"
#include "cJSON.h"

#include "sign_api.h"
#include "bsp_key_fsm.h"
#include "bsp_config.h"
#include "bsp_joystick.h"
#include "bsp_dns.h"

#define BEEP_GPIO GPIO_NUM_4

bool g_beep_status = true;
bool g_wifi_connected = false;
int g_wifi_con_status = 0;
dev_config_t g_dev_config = {0};
dev_mqtt_t g_dev_mqtt = {0};
esp_mqtt_client_handle_t g_mqtt_client = NULL;

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
    if (ret != ESP_OK)
    {
        ESP_LOGI(__func__, "read nvs fail");
        goto check_sys_finally;
    }
    if (require_size == 0)
    {
        ESP_LOGI(__func__, "no wifi ssid");
        ret = ESP_FAIL;
        goto check_sys_finally;
    }

    // pswd
    ret = nvs_get_str(nvs_handle, NVS_SYS_WIFI_SSID, g_dev_config.dev_ssid, &require_size);
    if (ret != ESP_OK)
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
    if (ret != ESP_OK)
    {
        ESP_LOGI(__func__, "open nvs error, %s", esp_err_to_name(ret));
    }

    ret = nvs_set_str(nvs_handle, NVS_SYS_WIFI_SSID, g_dev_config.dev_ssid);
    if (ret != ESP_OK)
    {
        ESP_LOGI(__func__, "open nvs error, %s", esp_err_to_name(ret));
    }
    ret = nvs_set_str(nvs_handle, NVS_SYS_WIFI_PSWD, g_dev_config.dev_pswd);
    if (ret != ESP_OK)
    {
        ESP_LOGI(__func__, "open nvs error, %s", esp_err_to_name(ret));
    }
    ret = nvs_set_str(nvs_handle, NVS_SYS_DEV_NAME, g_dev_config.dev_name);
    if (ret != ESP_OK)
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
    if (md == WIFI_MODE_APSTA)
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
    esp_wifi_get_mac(WIFI_IF_STA, eth_mac);
    ;
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
    wifi_ap_record_t *ap_info = (wifi_ap_record_t *)user_malloc(10 * sizeof(wifi_ap_record_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (ap_info == NULL)
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
    if (body == NULL)
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
    if (body == NULL)
    {
        ESP_LOGI(__func__, "Failed to allocate memory for request body");
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"code\": \"400\"}", HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }

    char *p;
    memset(body, 0x00, 128);

    int len = httpd_req_recv(req, body, 127);
    if (len <= 0)
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
    if (root == NULL)
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
    if (httpd_start(&server, &config) == ESP_OK)
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

void mqtt_event_process(const char *data)
{
    cJSON *root = cJSON_Parse(data);
    if (root != NULL)
    {
        cJSON *tmp_json = cJSON_GetObjectItem(root, "code");
        if (tmp_json == NULL)
        {
            ESP_LOGE(__func__, "mqtt data code is NULL");
        }
        else
        {
            int code = tmp_json->valueint;
            ESP_LOGI(__func__, "mqtt code: %d", code);
            switch (code)
            {
            case 200:
                break;
            case 300:
                break;
            default:
                break;
            }
        }
        cJSON_Delete(root);
    }
}

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0)
    {
        ESP_LOGE(__func__, "Last error %s: 0x%x", message, error_code);
    }
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(__func__, "Event dispatched from event loop base=%s, event_id=%" PRIi32, base, event_id);
    esp_mqtt_event_handle_t event = event_data;

    ESP_LOGD(__func__, "free heap size is %" PRIu32 ", minimum %" PRIu32, esp_get_free_heap_size(), esp_get_minimum_free_heap_size());

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
    {
        char *mqtt_topic_cloud = user_malloc(64, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        memset(mqtt_topic_cloud, 0x00, 64);
        sprintf(mqtt_topic_cloud, "%s/%s/user/get", g_dev_mqtt.ProductKey, g_dev_mqtt.DeviceName);
        ESP_LOGI(__func__, "topic is %s", mqtt_topic_cloud);

        esp_mqtt_client_subscribe(g_mqtt_client, mqtt_topic_cloud, 1);
    }
    break;

    case MQTT_EVENT_DISCONNECTED:
    {
        ESP_LOGI(__func__, "MQTT_EVENT_DISCONNECTED");
    }
    break;

    case MQTT_EVENT_SUBSCRIBED:
    {
        ESP_LOGI(__func__, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
    }
    break;

    case MQTT_EVENT_DATA:
    {
        ESP_LOGI(__func__, "TOPIC=%.*s", event->topic_len, event->topic);
        ESP_LOGI(__func__, "DATA=%.*s", event->data_len, event->data);
        mqtt_event_process((const char *)(event->data));
    }
    break;

    case MQTT_EVENT_ERROR:
    {
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno", event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(__func__, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
    }
    break;

    default:
    {
        ESP_LOGI(__func__, "Other event id:%d", event->event_id);
    }
    break;
    }
}

static void start_mqtt_app(void)
{
    iotx_dev_meta_info_t *meta_info = user_malloc(sizeof(iotx_dev_meta_info_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    memset(meta_info, 0x00, sizeof(iotx_dev_meta_info_t));
    memcpy(meta_info->product_key, g_dev_mqtt.ProductKey, strlen(g_dev_mqtt.ProductKey));
    memcpy(meta_info->device_name, g_dev_mqtt.DeviceName, strlen(g_dev_mqtt.DeviceName));
    memcpy(meta_info->device_secret, g_dev_mqtt.DeviceSecret, strlen(g_dev_mqtt.DeviceSecret));

    iotx_sign_mqtt_t *sign_mqtt = user_malloc(sizeof(iotx_sign_mqtt_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    memset(sign_mqtt, 0x00, sizeof(iotx_sign_mqtt_t));

    IOT_Sign_MQTT(IOTX_CLOUD_REGION_SHANGHAI, meta_info, sign_mqtt);

    esp_mqtt_client_config_t *mqtt_cfg = user_malloc(sizeof(esp_http_client_config_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    memset(mqtt_cfg, 0x00, sizeof(esp_mqtt_client_config_t));

    mqtt_cfg->broker.address.hostname = sign_mqtt->hostname;
    mqtt_cfg->broker.address.port = sign_mqtt->port;
    mqtt_cfg->broker.address.transport = MQTT_TRANSPORT_OVER_TCP;
    mqtt_cfg->credentials.username = sign_mqtt->username;
    mqtt_cfg->credentials.client_id = sign_mqtt->clientid;
    mqtt_cfg->credentials.authentication.password = sign_mqtt->password;
    mqtt_cfg->session.keepalive = 120;
    mqtt_cfg->session.disable_clean_session = true;
    mqtt_cfg->session.protocol_ver = MQTT_PROTOCOL_V_5;
    mqtt_cfg->session.message_retransmit_timeout = 3000;

    g_mqtt_client = esp_mqtt_client_init(mqtt_cfg);

    // 事件处理函数
    esp_mqtt_client_register_event(g_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(g_mqtt_client);
}

esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(__func__, "HTTP_EVENT_ERROR");
        break;

    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(__func__, "HTTP_EVENT_ON_CONNECTED");
        break;

    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(__func__, "HTTP_EVENT_HEADER_SENT");
        break;

    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(__func__, "HTTP_EVENT_ON_HEADER");
        break;

    case HTTP_EVENT_ON_DATA:
    {
        file_stream_t *user_data = (file_stream_t *)evt->user_data;
        if (!esp_http_client_is_chunked_response(evt->client))
        {
            if (user_data->length == 0)
                ESP_LOGI(__func__, "HTTP_EVENT_ON_DATA");
        }
        if (user_data->length + evt->data_len >= user_data->maxlen)
        {
            ESP_LOGE(__func__, "data buffer overlow!");
            user_data->length = -1;
            esp_http_client_close(evt->client);
        }
        else
        {
            memcpy(user_data->data + user_data->length, evt->data, evt->data_len);
            user_data->length += evt->data_len;
        }
    }
    break;

    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(__func__, "HTTP_EVENT_ON_FINISH");
        break;

    default:
        break;
    }
    return ESP_OK;
}

esp_err_t http_post(const char *url, char *body, char *resp, uint32_t len)
{
    file_stream_t http_data = {
        .data = (resp == NULL ? (uint8_t *)resp : NULL),
        .length = 0,
        .maxlen = len,
    };

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .event_handler = http_event_handler,
        .user_data = &http_data,
        .timeout_ms = 30000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    if (body)
    {
        esp_http_client_set_header(client, "Content-type", "application/json");
        esp_http_client_set_post_field(client, body, strlen(body));
    }

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK)
    {
        ESP_LOGI(__func__, "http post status = %d, coentent length = %lld",
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));

        if (esp_http_client_get_content_length(client) < 0)
        {
            esp_http_client_cleanup(client);
            return ESP_FAIL;
        }
    }
    else
    {
        ESP_LOGD(__func__, "http post request failed: %s", esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
    return err;
}

static void get_device_info()
{
    char *resp = user_malloc(256, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    const char *url = HTTPS_HOST "register_device";
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "type", 2);
    cJSON_AddStringToObject(root, "mac", g_dev_config.dev_mac);
    char *body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    esp_err_t ret = ESP_OK;
    int retry = 0;

try_get_dev_info:
    memset(resp, 0x00, 256);

    ret = http_post(url, body, resp, 256);

    if(retry > 12)
    {
        esp_restart();
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    if(ret != ESP_OK)
    {
        retry++;
        ESP_LOGE(__func__, "device info get err! retry times: %d", retry);
        vTaskDelay(pdMS_TO_TICKS(5000));
        goto try_get_dev_info;
    }
    else
    {
        root = cJSON_Parse(resp);
        if(root == NULL)
        {
            retry++;
            ESP_LOGE(__func__, "resp parse to json fail! retry times: %d", retry);
            vTaskDelay(pdMS_TO_TICKS(5000));
            goto try_get_dev_info;
        }
        else
        {
            bool success = cJSON_IsTrue(cJSON_GetObjectItem(root, "success"));
            if(success)
            {
                cJSON *data = cJSON_GetObjectItem(root, "data");
                char *product_key = (cJSON_GetObjectItem(data, "product_key") != NULL ? cJSON_GetObjectItem(data, "product_key")->valuestring : NULL);
                char *device_name = (cJSON_GetObjectItem(data, "device_name") != NULL ? cJSON_GetObjectItem(data, "device_name")->valuestring : NULL);
                char *device_secret = (cJSON_GetObjectItem(data, "device_secret") != NULL ? cJSON_GetObjectItem(data, "device_secret")->valuestring : NULL);
                if (product_key == NULL || device_name == NULL || device_secret == NULL)
                {
                    retry++;
                    cJSON_Delete(root);
                    ESP_LOGE(__func__, "mqtt triplet get fail! retry times: %d", retry);
                    vTaskDelay(pdMS_TO_TICKS(5000));
                    goto try_get_dev_info;
                }
                
                memset(g_dev_mqtt.ProductKey, 0x00, sizeof(g_dev_mqtt.ProductKey));
                memset(g_dev_mqtt.DeviceName, 0x00, sizeof(g_dev_mqtt.DeviceName));
                memset(g_dev_mqtt.DeviceSecret, 0x00, sizeof(g_dev_mqtt.DeviceSecret));

                strcpy(g_dev_mqtt.ProductKey, product_key);
                strcpy(g_dev_mqtt.DeviceName, device_name);
                strcpy(g_dev_mqtt.DeviceSecret, device_secret);
            }
            cJSON_Delete(root);
        }
    }
    ESP_LOGI(__func__, "dev_info: %s, %s, %s", g_dev_mqtt.ProductKey, g_dev_mqtt.DeviceName, g_dev_mqtt.DeviceSecret);
    user_free(__func__, resp);
    user_free(__func__, body);
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
        get_device_info();
        start_mqtt_app();
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
