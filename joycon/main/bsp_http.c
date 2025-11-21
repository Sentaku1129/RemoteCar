#include "esp_http_client.h"
#include "esp_log.h"
#include "bsp_config.h"

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