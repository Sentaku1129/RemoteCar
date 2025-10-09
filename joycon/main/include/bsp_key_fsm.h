#ifndef _BSP_KEY_FSM_H_
#define _BSP_KEY_FSM_H_

#include "driver/gpio.h"

// 状态机参数
#define DEBOUNCE_TIME_MS 50
#define LONG_PRESS_TIME_MS 1000
#define MULTI_CLICK_INTERVAL 400
#define KEY_POLLING_CYCLE 10

#define KEY_NUM 4

// 按键
#define BUTTON_0    GPIO_NUM_25
#define BUTTON_1    GPIO_NUM_26
#define BUTTON_2    GPIO_NUM_27
#define BUTTON_3    GPIO_NUM_14

typedef struct
{
    uint32_t GPIO_PULL;
    gpio_num_t GPIO_NUM;
    uint8_t GPIO_INDEX;
} key_init_t;

typedef enum
{
    KEY_IDLE,
    KEY_DEBOUNCING,
    KEY_PRESSED,
    KEY_LONG_PRESSED,
    KEY_MULTI_CLICK
} key_status_t;

typedef struct {
    key_status_t key_status;
    int64_t key_value;
    bool key_fsm_finished;
} key_value_t;

void key_init(key_init_t *key_io, key_value_t *key_value);
void key_read_status();
void button_task(void *arg);

#endif
