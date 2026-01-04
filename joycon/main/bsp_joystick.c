#include "esp_log.h"
#include "stdio.h"
#include "math.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"

#include "bsp_joystick.h"
static adc_oneshot_unit_handle_t unit_handle;
static joystick_vatual_button_t joystcik_left_vitual_button = joystick_vitual_idle;
static joystick_vatual_button_t joystcik_right_vitual_button = joystick_vitual_idle;
static adc_cali_handle_t left_x_cali_handle = NULL;
static adc_cali_handle_t left_y_cali_handle = NULL;
static adc_cali_handle_t right_x_cali_handle = NULL;
static adc_cali_handle_t right_y_cali_handle = NULL;
joystick_normalized_t joycon_value_L = {0};
joystick_normalized_t joycon_value_R = {0};

int g_battery_level; // 0-100
static adc_cali_handle_t battery_cali_handle = NULL;

static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

    if (!calibrated)
    {
        ESP_LOGI(__func__, "calibration scheme version is %s", "Line Fitting");

        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_WIDTH,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
            calibrated = true;
    }

    *out_handle = handle;
    if (ret == ESP_OK)
    {
        ESP_LOGI(__func__, "Calibration Success");
    }
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
    {
        ESP_LOGW(__func__, "eFuse not burnt, skip software calibration");
    }
    else
    {
        ESP_LOGE(__func__, "Invalid arg or no memory");
    }

    return calibrated;
}

esp_err_t joystick_init(void)
{
    esp_err_t ret = ESP_OK;

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config, &unit_handle);

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_WIDTH,
    };
    adc_oneshot_config_channel(unit_handle, LEFT_X_CHANNEL, &config);
    adc_oneshot_config_channel(unit_handle, LEFT_Y_CHANNEL, &config);
    adc_oneshot_config_channel(unit_handle, RIGHT_X_CHANNEL, &config);
    adc_oneshot_config_channel(unit_handle, RIGHT_Y_CHANNEL, &config);

    bool do_left_x_cali = adc_calibration_init(ADC_UNIT_1, LEFT_X_CHANNEL, ADC_ATTEN, &left_x_cali_handle);
    if (!do_left_x_cali)
    {
        ESP_LOGE(__func__, "Left X Calibration Fail");
    }
    bool do_left_y_cali = adc_calibration_init(ADC_UNIT_1, LEFT_Y_CHANNEL, ADC_ATTEN, &left_y_cali_handle);
    if (!do_left_y_cali)
    {
        ESP_LOGE(__func__, "Left Y Calibration Fail");
    }
    bool do_right_x_cali = adc_calibration_init(ADC_UNIT_1, RIGHT_X_CHANNEL, ADC_ATTEN, &right_x_cali_handle);
    if (!do_right_x_cali)
    {
        ESP_LOGE(__func__, "Right X Calibration Fail");
    }
    bool do_right_y_cali = adc_calibration_init(ADC_UNIT_1, RIGHT_Y_CHANNEL, ADC_ATTEN, &right_y_cali_handle);
    if (!do_right_y_cali)
    {
        ESP_LOGE(__func__, "Right Y Calibration Fail");
    }
    return ret;
}

esp_err_t battery_init()
{
    esp_err_t ret = ESP_OK;

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_WIDTH,
    };

    adc_oneshot_config_channel(unit_handle, BATTERY_CHANNEL, &config);

    bool do_battery_cali = adc_calibration_init(ADC_UNIT_1, BATTERY_CHANNEL, ADC_ATTEN, &battery_cali_handle);
    if (!do_battery_cali)
    {
        ESP_LOGI(__func__, "Battery Calibration Fail");
    }
    return ret;
}

esp_err_t joystick_read_left(joystick_value_t *value)
{
    if (value == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    adc_oneshot_read(unit_handle, LEFT_X_CHANNEL, &(value->x));
    adc_oneshot_read(unit_handle, LEFT_Y_CHANNEL, &(value->y));

    if (value->x < 0 || value->y < 0)
    {
        ESP_LOGE(__func__, "Failed to read left joystick");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t joystick_read_right(joystick_value_t *value)
{
    if (value == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    adc_oneshot_read(unit_handle, RIGHT_X_CHANNEL, &(value->x));
    adc_oneshot_read(unit_handle, RIGHT_Y_CHANNEL, &(value->y));

    if (value->x < 0 || value->y < 0)
    {
        ESP_LOGE(__func__, "Failed to read left joystick");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t battery_read()
{
    int raw = 0, voltage = 0;
    esp_err_t ret = adc_oneshot_read(unit_handle, BATTERY_CHANNEL, &raw);

    if (ret != ESP_OK)
    {
        ESP_LOGE("BATTERY", "ADC read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = adc_cali_raw_to_voltage(battery_cali_handle, raw, &voltage);
    if(ret != ESP_OK)
    {
        ESP_LOGI(__func__, "ADC cali failed: %s", esp_err_to_name(ret));
        return ret;
    }

    voltage *=2;

    voltage = voltage < 3000 ? 3000 : voltage;
    voltage = voltage > 4200 ? 4200 : voltage;

    // 计算电池电量百分比
    g_battery_level = (voltage - 3000) * 100 / 1200;
    // ESP_LOGI(__func__, "raw = %d, voltage = %dmv, battery level = %d%%", raw, voltage, g_battery_level);

    return ESP_OK;
}

esp_err_t joystick_read_left_normalized(joystick_normalized_t *value)
{
    if (value == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    joystick_value_t raw_value = {0};
    esp_err_t ret = joystick_read_left(&raw_value);
    if (ret != ESP_OK)
    {
        return ret;
    }

    value->x = (raw_value.x - ADC_CENTER_VALUE) / (float)ADC_CENTER_VALUE;
    value->y = (raw_value.y - ADC_CENTER_VALUE) / (float)ADC_CENTER_VALUE;

    if (value->x > 1.0f)
        value->x = 1.0f;
    if (value->x < -1.0f)
        value->x = -1.0f;

    if (value->y > 1.0f)
        value->y = 1.0f;
    if (value->y < -1.0f)
        value->y = -1.0f;

    return ESP_OK;
}

esp_err_t joystick_read_right_normalized(joystick_normalized_t *value)
{
    if (value == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    joystick_value_t raw_value = {0};
    esp_err_t ret = joystick_read_right(&raw_value);
    if (ret != ESP_OK)
    {
        return ret;
    }

    value->x = (raw_value.x - ADC_CENTER_VALUE) / (float)ADC_CENTER_VALUE;
    value->y = (raw_value.y - ADC_CENTER_VALUE) / (float)ADC_CENTER_VALUE;

    if (value->x > 1.0f)
        value->x = 1.0f;
    if (value->x < -1.0f)
        value->x = -1.0f;

    if (value->y > 1.0f)
        value->y = 1.0f;
    if (value->y < -1.0f)
        value->y = -1.0f;

    return ESP_OK;
}

void joystick_task(void *arg)
{
    joystick_init();
    battery_init();
    joystick_normalized_t left_norm = {0}, right_norm = {0};
    while (1)
    {
        if (joystick_read_left_normalized(&left_norm) == ESP_OK && joystick_read_right_normalized(&right_norm) == ESP_OK)
        {
            joycon_value_L = left_norm;
            joycon_value_R = right_norm;
            if (fabs(left_norm.x) > vitual_button_norm_reference_value || fabs(left_norm.y) > vitual_button_norm_reference_value)
            {
                float value = fabs(left_norm.x) >= fabs(left_norm.y) ? left_norm.x : left_norm.y;
                if (value > 0)
                {
                    joystcik_left_vitual_button = fabs(left_norm.x) >= fabs(left_norm.y) ? joystick_vitual_right : joystick_vitual_up;
                }
                else
                {
                    joystcik_left_vitual_button = fabs(left_norm.x) >= fabs(left_norm.y) ? joystick_vitual_left : joystick_vitual_down;
                }
            }
            else
            {
                joystcik_left_vitual_button = joystick_vitual_idle;
            }

            if (fabs(right_norm.x) > vitual_button_norm_reference_value || fabs(right_norm.y) > vitual_button_norm_reference_value)
            {
                float value = fabs(right_norm.x) >= fabs(right_norm.y) ? right_norm.x : right_norm.y;
                if (value > 0)
                {
                    joystcik_right_vitual_button = fabs(right_norm.x) >= fabs(right_norm.y) ? joystick_vitual_right : joystick_vitual_up;
                }
                else
                {
                    joystcik_right_vitual_button = fabs(right_norm.x) >= fabs(right_norm.y) ? joystick_vitual_left : joystick_vitual_down;
                }
            }
            else
            {
                joystcik_right_vitual_button = joystick_vitual_idle;
            }

            battery_read();
        }
        else
        {
            ESP_LOGE(__func__, " READ Normalized fail");
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

joystick_vatual_button_t read_left_joystick_postion()
{
    return joystcik_left_vitual_button;
}

joystick_vatual_button_t read_right_joystick_postion()
{
    return joystcik_right_vitual_button;
}

joy_adjust_value_t read_joy_adjust_offset()
{
    const int adjust_tick = 100;
    joy_adjust_value_t joy_adjust_value = {0};
    for(int i = 0; i < adjust_tick; i ++)
    {
        joy_adjust_value.left.x += joycon_value_L.x;
        joy_adjust_value.left.y += joycon_value_L.y;

        joy_adjust_value.right.x += joycon_value_R.x;
        joy_adjust_value.right.y += joycon_value_R.y;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    joy_adjust_value.left.x /= adjust_tick;
    joy_adjust_value.left.y /= adjust_tick;

    joy_adjust_value.right.x /= adjust_tick;
    joy_adjust_value.right.y /= adjust_tick;
    return joy_adjust_value;
}
