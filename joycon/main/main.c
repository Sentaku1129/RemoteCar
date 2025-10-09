#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"

#include "bsp_key_fsm.h"

#define BEEP_GPIO   GPIO_NUM_4

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
    while (1)
    {
        xQueueReceive(beep_queue, NULL, portMAX_DELAY);
        if(xQueueReceive(beep_queue, NULL, portMAX_DELAY))
        {
            
        }
    }
    
}

void app_main(void)
{
    xTaskCreatePinnedToCore(button_task, "button_task", 1024 * 2, NULL, 10, NULL, 0);
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(__func__, "hello world");
    }
    
    printf("Hello world!\n");
}
