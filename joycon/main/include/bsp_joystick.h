#ifndef _BSP_JOYSTICK_H_
#define _BSP_JOYSTICK_H_

#include "driver/adc_types_legacy.h"

// ADC通道定义
// 左摇杆: SENSOR_VP (GPIO36) -> ADC1_CH0, SENSOR_VN (GPIO39) -> ADC1_CH3
// 右摇杆: IO35 (GPIO35) -> ADC1_CH7, IO33 (GPIO33) -> ADC1_CH5
#define LEFT_X_CHANNEL      ADC1_CHANNEL_0  // GPIO36 (SENSOR_VP)
#define LEFT_Y_CHANNEL      ADC1_CHANNEL_3  // GPIO39 (SENSOR_VN)
#define RIGHT_X_CHANNEL     ADC1_CHANNEL_7  // GPIO35
#define RIGHT_Y_CHANNEL     ADC1_CHANNEL_5  // GPIO33

// ADC配置
#define ADC_WIDTH           ADC_WIDTH_BIT_12
#define ADC_ATTEN           ADC_ATTEN_DB_12
#define ADC_MAX_VALUE       4095
#define ADC_CENTER_VALUE    2048

// vitual button reference value
#define vitual_button_norm_reference_value 0.3f

typedef struct
{
    int x;
    int y;
} joystick_value_t;

typedef struct
{
    float x;
    float y;
}joystick_normalized_t;

typedef enum
{
    joystick_vitual_idle = 0,
    joystick_vitual_up,
    joystick_vitual_down,
    joystick_vitual_left,
    joystick_vitual_right
}joystick_vatual_button_t;

void joystick_task(void *arg);

#endif