#ifndef _BSP_HTTP_H_
#define _BSP_HTTP_H_

#include "esp_err.h"

esp_err_t http_post(const char *url, char *body, char *resp, uint32_t len);

#endif
