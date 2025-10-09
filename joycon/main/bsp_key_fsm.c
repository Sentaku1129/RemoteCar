#include <stdio.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"

#include "bsp_key_fsm.h"

typedef enum
{
    LOW_LEVEL = 0,
    HIGH_LEVEL = 1,
} key_level_t;

typedef struct
{
    bool key_enable;
    key_status_t key_status;
    key_status_t key_event;
    key_level_t key_valid_level;
    bool key_is_press;
    int64_t key_press_times;
    int64_t key_release_times;
    int64_t key_debouncint_times;
    int click_count;
    key_level_t (*READ_PIN)(key_init_t key);
} key_fsm_t;

typedef struct
{
    key_init_t key_init;
    key_fsm_t key_fsm;
    key_value_t *key_value;

} key_config_t;

key_config_t key_config[KEY_NUM] = {0};

static void get_key_level()
{
    uint8_t i = 0;
    for (i = 0; i < KEY_NUM; i++)
    {
        if (key_config[i].key_fsm.key_enable == false)
            continue;

        if (key_config[i].key_fsm.READ_PIN(key_config[i].key_init) == key_config[i].key_fsm.key_valid_level)
        {
            key_config[i].key_fsm.key_is_press = true;
            // ESP_LOGI(__func__, "i = GPIO_NUM_%d is press", key_config[i].key_init.GPIO_NUM);
        }
        else
        {
            key_config[i].key_fsm.key_is_press = false;
        }
    }
}

static key_level_t key_read_level(key_init_t key)
{
    return gpio_get_level(key.GPIO_NUM) == LOW_LEVEL ? LOW_LEVEL : HIGH_LEVEL;
}

static void key_gpio_init(key_init_t key)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << key.GPIO_NUM),
        .pull_down_en = key.GPIO_PULL == GPIO_PULLUP_ENABLE ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
        .pull_up_en = key.GPIO_PULL == GPIO_PULLUP_ENABLE ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,  // 上拉
    };
    printf("%s: io num = %d\r\n", __func__, key.GPIO_NUM);
    gpio_config(&io_conf);
}

static void create_key(key_init_t *key, key_value_t *key_value)
{
    uint8_t i;
    for (i = 0; i < KEY_NUM; i++)
    {
        key_config[i].key_init = key[i];
        key_config[i].key_init.GPIO_INDEX = i;

        key_config[i].key_fsm.key_status = KEY_IDLE;
        key_config[i].key_fsm.key_event = KEY_IDLE;
        key_config[i].key_fsm.key_enable = true;
        key_config[i].key_fsm.click_count = 0;
        key_config[i].key_fsm.key_valid_level = key_config[i].key_init.GPIO_PULL == GPIO_PULLUP_ENABLE ? LOW_LEVEL : HIGH_LEVEL;
        key_config[i].key_fsm.key_is_press = false;
        key_config[i].key_fsm.READ_PIN = key_read_level;

        key_value[i].key_status = KEY_IDLE;
        key_value[i].key_value = 0;
        key_value[i].key_fsm_finished = false;
        key_config[i].key_value = &key_value[i];
        key_gpio_init(key[i]);
    }
}

void key_init(key_init_t *key_io, key_value_t *key_value) // IO初始化
{
    create_key(key_io, key_value); // 调用按键初始化函数
}

void key_read_status()
{
    uint8_t i;
    get_key_level();
    for (i = 0; i < KEY_NUM; i++)
    {
        // ESP_LOGI(__func__, "GPIO_NUM_%d, status = %d", key_config[i].key_init.GPIO_NUM, key_config[i].key_fsm.key_status);
        switch (key_config[i].key_fsm.key_status)
        {
        case KEY_IDLE:
            if (key_config[i].key_fsm.key_is_press == true)
            {
                key_config[i].key_fsm.key_status = KEY_DEBOUNCING;
                key_config[i].key_fsm.key_event = KEY_DEBOUNCING;
                key_config[i].key_fsm.key_debouncint_times = 1;
                key_config[i].key_value->key_fsm_finished = false;
                key_config[i].key_value->key_status = KEY_DEBOUNCING;
                key_config[i].key_value->key_value = 0;
            }
            else
            {
                key_config[i].key_fsm.key_status = KEY_IDLE;
                key_config[i].key_fsm.key_event = KEY_IDLE;
            }
            break;
        case KEY_DEBOUNCING:
            if (key_config[i].key_fsm.key_is_press == true)
            {
                if (key_config[i].key_fsm.key_event == KEY_IDLE)
                {
                    key_config[i].key_fsm.key_event = KEY_DEBOUNCING;
                    key_config[i].key_fsm.key_press_times += key_config[i].key_fsm.key_debouncint_times;
                }
                else
                {
                    if (key_config[i].key_fsm.key_debouncint_times >= (DEBOUNCE_TIME_MS / KEY_POLLING_CYCLE))
                    {
                        key_config[i].key_fsm.key_status = KEY_PRESSED;
                        key_config[i].key_fsm.key_event = KEY_PRESSED;
                        key_config[i].key_fsm.key_debouncint_times += 1;
                        key_config[i].key_fsm.key_press_times = key_config[i].key_fsm.key_debouncint_times;

                        key_config[i].key_value->key_status = KEY_PRESSED;
                    }
                    else
                    {
                        key_config[i].key_fsm.key_debouncint_times += 1;
                    }
                }
            }
            else
            {
                if (key_config[i].key_fsm.key_event == KEY_IDLE)
                {
                    if (key_config[i].key_fsm.key_release_times >= (DEBOUNCE_TIME_MS / KEY_POLLING_CYCLE))
                    {
                        key_config[i].key_fsm.key_status = KEY_IDLE;
                        key_config[i].key_fsm.key_event = KEY_IDLE;
                        key_config[i].key_value->key_fsm_finished = true;
                    }
                    else
                    {
                        key_config[i].key_fsm.key_release_times += 1;
                        key_config[i].key_fsm.key_debouncint_times += 1;
                    }
                }
                else if (key_config[i].key_fsm.key_event == KEY_DEBOUNCING)
                {
                    key_config[i].key_fsm.key_event = KEY_IDLE;
                    key_config[i].key_fsm.key_release_times = 1;
                    key_config[i].key_fsm.key_debouncint_times = 1;
                }
            }
            break;
        case KEY_PRESSED:
            if (key_config[i].key_fsm.key_is_press == true)
            {
                if (key_config[i].key_fsm.key_event == KEY_IDLE)
                {
                    if(key_config[i].key_fsm.key_debouncint_times < (DEBOUNCE_TIME_MS / KEY_POLLING_CYCLE))
                    {
                        key_config[i].key_fsm.key_debouncint_times = key_config[i].key_fsm.key_debouncint_times + key_config[i].key_fsm.key_press_times;
                        key_config[i].key_fsm.key_press_times += key_config[i].key_fsm.key_debouncint_times;
                    }
                    else
                    {
                        key_config[i].key_fsm.key_status = KEY_MULTI_CLICK;
                        key_config[i].key_fsm.key_event = KEY_MULTI_CLICK;
                        key_config[i].key_fsm.click_count = 2;
                        key_config[i].key_fsm.key_debouncint_times = 1;
                        key_config[i].key_fsm.key_press_times = 1;

                        key_config[i].key_value->key_status = KEY_MULTI_CLICK;
                        key_config[i].key_value->key_value = 2;
                    }
                    // ESP_LOGI(__func__, "Multi Click");
                }
                else if (key_config[i].key_fsm.key_event == KEY_PRESSED)
                {
                    if (key_config[i].key_fsm.key_press_times >= (LONG_PRESS_TIME_MS / KEY_POLLING_CYCLE))
                    {
                        key_config[i].key_fsm.key_status = KEY_LONG_PRESSED;
                        key_config[i].key_fsm.key_event = KEY_LONG_PRESSED;

                        key_config[i].key_value->key_status = KEY_LONG_PRESSED;
                        key_config[i].key_value->key_value = key_config[i].key_fsm.key_press_times + 1;
                        // ESP_LOGI(__func__, "Long Click Start");
                    }
                    key_config[i].key_fsm.key_press_times += 1;
                    key_config[i].key_fsm.key_debouncint_times += 1;
                }
            }
            else
            {
                if (key_config[i].key_fsm.key_event == KEY_IDLE)
                {
                    if (key_config[i].key_fsm.key_debouncint_times >= (MULTI_CLICK_INTERVAL / 10))
                    {
                        key_config[i].key_fsm.key_status = KEY_IDLE;
                        key_config[i].key_fsm.key_event = KEY_IDLE;

                        key_config[i].key_value->key_status = KEY_PRESSED;
                        key_config[i].key_value->key_value = 0;
                        key_config[i].key_value->key_fsm_finished = true;
                        // ESP_LOGI(__func__, "ONE Click Finished");
                    }
                    else
                    {
                        key_config[i].key_fsm.key_debouncint_times += 1;
                        key_config[i].key_fsm.key_release_times += 1;
                    }
                }
                else if (key_config[i].key_fsm.key_event == KEY_PRESSED)
                {
                    key_config[i].key_fsm.key_event = KEY_IDLE;
                    key_config[i].key_fsm.key_debouncint_times = 1;
                    key_config[i].key_fsm.key_release_times = 1;
                }
            }
            break;
        case KEY_LONG_PRESSED:
            if (key_config[i].key_fsm.key_is_press == true)
            {
                if(key_config[i].key_fsm.key_event == KEY_IDLE)
                {
                    key_config[i].key_fsm.key_status = KEY_LONG_PRESSED;
                    key_config[i].key_fsm.key_press_times += key_config[i].key_fsm.key_debouncint_times;
                }
                key_config[i].key_fsm.key_press_times += 1;
                key_config[i].key_value->key_value = key_config[i].key_fsm.key_press_times;
            }
            else
            {
                if (key_config[i].key_fsm.key_event == KEY_LONG_PRESSED)
                {
                    key_config[i].key_fsm.key_event = KEY_IDLE;
                    key_config[i].key_fsm.key_debouncint_times = 1;
                }
                else if (key_config[i].key_fsm.key_event == KEY_IDLE)
                {
                    if (key_config[i].key_fsm.key_debouncint_times >= (DEBOUNCE_TIME_MS / KEY_POLLING_CYCLE))
                    {
                        key_config[i].key_fsm.key_status = KEY_IDLE;
                        key_config[i].key_value->key_value = key_config[i].key_fsm.key_press_times + key_config[i].key_fsm.key_debouncint_times;
                        key_config[i].key_value->key_fsm_finished = true;
                        // ESP_LOGI(__func__, "Long Click time: %lld ms", key_config[i].key_fsm.key_press_times * KEY_POLLING_CYCLE);
                    }
                    else
                    {
                        key_config[i].key_fsm.key_debouncint_times += 1;
                    }
                }
            }
            break;
        case KEY_MULTI_CLICK:
            if (key_config[i].key_fsm.key_is_press == true)
            {
                if (key_config[i].key_fsm.key_event == KEY_IDLE)
                {
                    key_config[i].key_fsm.key_event = KEY_MULTI_CLICK;
                    if(key_config[i].key_fsm.key_debouncint_times < (DEBOUNCE_TIME_MS / KEY_POLLING_CYCLE))
                    {
                        key_config[i].key_fsm.key_debouncint_times = key_config[i].key_fsm.key_press_times + key_config[i].key_fsm.key_release_times;
                        key_config[i].key_fsm.key_press_times += key_config[i].key_fsm.key_release_times;
                    }
                    else
                    {
                        key_config[i].key_fsm.click_count += 1;
                        key_config[i].key_fsm.key_debouncint_times = 1;
                        key_config[i].key_fsm.key_press_times = 1;

                        key_config[i].key_value->key_value = key_config[i].key_fsm.click_count;
                    }
                }
                else if(key_config[i].key_fsm.key_event == KEY_MULTI_CLICK)
                {
                    key_config[i].key_fsm.key_debouncint_times +=1;
                    key_config[i].key_fsm.key_press_times += 1;
                }
            }
            else
            {
                if(key_config[i].key_fsm.key_event == KEY_MULTI_CLICK)
                {
                    key_config[i].key_fsm.key_event = KEY_IDLE;
                    if(key_config[i].key_fsm.key_debouncint_times < (DEBOUNCE_TIME_MS / KEY_POLLING_CYCLE))
                    {
                        key_config[i].key_fsm.click_count -= 1;
                        if(key_config[i].key_fsm.click_count == 1)
                        {
                            key_config[i].key_fsm.key_status = KEY_PRESSED;
                            key_config[i].key_value->key_status = KEY_PRESSED;
                        }
                            
                        key_config[i].key_fsm.key_debouncint_times = key_config[i].key_fsm.key_press_times + key_config[i].key_fsm.key_release_times;
                        key_config[i].key_fsm.key_release_times += key_config[i].key_fsm.key_press_times;
                    }
                    else
                    {
                        key_config[i].key_fsm.key_debouncint_times = 1;
                        key_config[i].key_fsm.key_release_times = 1;
                    }
                }
                else if(key_config[i].key_fsm.key_event == KEY_IDLE)
                {
                    if(key_config[i].key_fsm.key_debouncint_times >= (MULTI_CLICK_INTERVAL / KEY_POLLING_CYCLE))
                    {
                        key_config[i].key_fsm.key_status = KEY_IDLE;
                        key_config[i].key_fsm.key_event = KEY_IDLE;

                        key_config[i].key_value->key_status = KEY_MULTI_CLICK;
                        key_config[i].key_value->key_value = key_config[i].key_fsm.click_count;
                        key_config[i].key_value->key_fsm_finished = true;
                        // ESP_LOGI(__func__, "%d times Click", key_config[i].key_fsm.click_count);
                    }
                    key_config[i].key_fsm.key_debouncint_times +=1;
                    key_config[i].key_fsm.key_release_times += 1;
                }
            }
            break;
        default:
            break;
        }
    }
}

void button_task(void *arg)
{
    key_value_t key_value[KEY_NUM] = {0};
    key_init_t key_io[KEY_NUM] =
        {
            {GPIO_PULLUP_ENABLE, BUTTON_0, 0},
            {GPIO_PULLUP_ENABLE, BUTTON_1, 0},
            {GPIO_PULLUP_ENABLE, BUTTON_2, 0},
            {GPIO_PULLUP_ENABLE, BUTTON_3, 0},
        };
    key_init(key_io, key_value);
    while (1)
    {
        key_read_status();
        for(int i = 0; i < KEY_NUM; i++)
        {
            if(key_value[i].key_status != KEY_IDLE)
            {
                ESP_LOGI(__func__, "GPIO_NUM_%d mode is %d; value = %lld; check %s finished",
                                    key_io[i].GPIO_NUM, 
                                    key_value[i].key_status, 
                                    key_value[i].key_value, 
                                    key_value[i].key_fsm_finished ? "is" : "isnt");
                if(key_value[i].key_fsm_finished)
                {
                    key_value[i].key_status = KEY_IDLE;
                    key_value[i].key_value = 0;
                    key_value[i].key_fsm_finished = false;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
