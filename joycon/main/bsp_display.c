#include "string.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"

#include "bsp_display.h"
#include "bsp_http.h"
#include "bsp_config.h"
#include "bsp_key_fsm.h"
#include "bsp_joystick.h"

static const char *TAG = "LCD_SPI";

// SPI 配置
#define LCD_HOST SPI2_HOST
#define LCD_SPI_CLOCK 8000000 // 8MHz SPI 时钟（可根据需要调整 1-20MHz）

static spi_device_handle_t spi_handle = NULL;

// ==================== LCD 复位 ====================
void lcd_reset(void)
{
    gpio_set_level(io_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(io_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "LCD 复位完成");
}

// ==================== 初始化硬件 SPI ====================
esp_err_t lcd_spi_init(void)
{
    esp_err_t ret;

    // 配置 DC 和 RST 引脚为输出
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << io_LCD_AO) | (1ULL << io_LCD_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    // 初始化引脚状态
    gpio_set_level(io_LCD_RST, 1);
    gpio_set_level(io_LCD_AO, 0);

    // 配置 SPI 总线
    spi_bus_config_t buscfg = {
        .mosi_io_num = io_LCD_SDA,  // MOSI 引脚
        .miso_io_num = -1,          // LCD 不需要 MISO
        .sclk_io_num = io_LCD_SCLK, // SCLK 引脚
        .quadwp_io_num = -1,        // 不使用 WP
        .quadhd_io_num = -1,        // 不使用 HD
        .max_transfer_sz = 4096,    // 最大传输大小
    };

    // 初始化 SPI 总线
    ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI 总线初始化失败: %s", esp_err_to_name(ret));
        return ret;
    }

    // 配置 SPI 设备（LCD）
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = LCD_SPI_CLOCK, // SPI 时钟频率
        .mode = 3,                       // SPI mode 3 (CPOL=1, CPHA=1) ST7567 常用
        .spics_io_num = io_LCD_CS,       // CS 引脚
        .queue_size = 7,                 // 事务队列大小
        .flags = SPI_DEVICE_NO_DUMMY,    // 无虚拟字节
        .pre_cb = NULL,                  // 传输前回调
    };

    // 添加 SPI 设备到总线
    ret = spi_bus_add_device(LCD_HOST, &devcfg, &spi_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI 设备添加失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "硬件 SPI 初始化完成");
    ESP_LOGI(TAG, "SPI 配置: SCLK=%d, MOSI=%d, CS=%d, DC=%d, RST=%d",
             io_LCD_SCLK, io_LCD_SDA, io_LCD_CS, io_LCD_AO, io_LCD_RST);

    return ESP_OK;
}

// ==================== 写命令 ====================
void lcd_write_command(uint8_t cmd)
{
    spi_transaction_t trans = {
        .length = 8,       // 传输 8 位
        .tx_buffer = &cmd, // 发送缓冲区
        .user = (void *)0, // 用户数据：0 表示命令
    };

    gpio_set_level(io_LCD_AO, 0); // DC = 0 表示命令
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &trans);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "写命令失败: 0x%02X", cmd);
    }
}

// ==================== 写单个数据 ====================
void lcd_write_data(uint8_t data)
{
    spi_transaction_t trans = {
        .length = 8,        // 传输 8 位
        .tx_buffer = &data, // 发送缓冲区
        .user = (void *)1,  // 用户数据：1 表示数据
    };

    gpio_set_level(io_LCD_AO, 1); // DC = 1 表示数据
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &trans);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "写数据失败: 0x%02X", data);
    }
}

// ==================== 批量写数据（高效）====================
void lcd_write_data_bulk(const uint8_t *data, size_t len)
{
    if (len == 0 || data == NULL)
        return;

    spi_transaction_t trans = {
        .length = len * 8, // 传输位数
        .tx_buffer = data, // 发送缓冲区
        .user = (void *)1, // 用户数据：1 表示数据
    };

    gpio_set_level(io_LCD_AO, 1); // DC = 1 表示数据
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &trans);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "批量写数据失败，长度: %d", len);
    }
}

// ==================== ST7567 初始化序列 ====================
void lcd_st7567_init(void)
{
    // 初始化硬件 SPI
    ESP_ERROR_CHECK(lcd_spi_init());

    // 复位 LCD
    lcd_reset();

    // ST7567 初始化命令序列
    lcd_write_command(0xE2); // 软复位
    vTaskDelay(pdMS_TO_TICKS(5));

    lcd_write_command(0x2C); // Power Control: Boost ON
    vTaskDelay(pdMS_TO_TICKS(2));
    lcd_write_command(0x2E); // Power Control: V regulator ON
    vTaskDelay(pdMS_TO_TICKS(2));
    lcd_write_command(0x2F); // Power Control: V follower ON
    vTaskDelay(pdMS_TO_TICKS(2));

    lcd_write_command(0x23); // 粗调对比度，可设置范围 0x20~0x27
    lcd_write_command(0x81); // 微调对比度命令
    lcd_write_command(0x28); // 对比度值，可设置范围 0x00~0x3F

    lcd_write_command(0xA2); // 1/9 偏压比（bias）
    lcd_write_command(0xC8); // 行扫描顺序：从上到下
    lcd_write_command(0xA0); // 列扫描顺序：从左到右

    lcd_write_command(0x40); // 起始行：第一行开始
    lcd_write_command(0xAF); // 开启显示

    ESP_LOGI(TAG, "ST7567 LCD 初始化完成");
}

// ==================== 清屏 ====================
void lcd_clear(void)
{
    uint8_t clear_data[128] = {0}; // 一行数据缓冲区

    for (uint8_t page = 0; page < 8; page++) // 8 页 (64 行 / 8)
    {
        lcd_write_command(0xB0 + page); // 设置页地址
        lcd_write_command(0x10);        // 设置列地址高 4 位
        lcd_write_command(0x00);        // 设置列地址低 4 位

        lcd_write_data_bulk(clear_data, 128); // 批量写入清空数据
    }

    ESP_LOGI(TAG, "LCD 清屏完成");
}

// ==================== 设置光标位置 ====================
void lcd_set_position(uint8_t page, uint8_t column)
{
    lcd_write_command(0xB0 + page);            // 设置页地址 (0-7)
    lcd_write_command(0x10 + (column >> 4));   // 列地址高 4 位
    lcd_write_command(0x00 + (column & 0x0F)); // 列地址低 4 位
}

// ==================== 显示图片/缓冲区 ====================
void lcd_display_image(const uint8_t *image)
{
    for (uint8_t page = 0; page < 8; page++)
    {
        lcd_set_position(page, 0);
        lcd_write_data_bulk(&image[page * 128], 128);
    }
}

// ==================== 设置对比度 ====================
void lcd_set_contrast(uint8_t contrast)
{
    lcd_write_command(0x81);     // 对比度命令
    lcd_write_command(contrast); // 对比度值 (0x00-0x3F)
}

// ==================== 显示开关 ====================
void lcd_display_on(void)
{
    lcd_write_command(0xAF); // 显示开
}

void lcd_display_off(void)
{
    lcd_write_command(0xAE); // 显示关
}

// ==================== 反色显示 ====================
void lcd_inverse_display(bool enable)
{
    if (enable)
    {
        lcd_write_command(0xA7); // 反色显示
    }
    else
    {
        lcd_write_command(0xA6); // 正常显示
    }
}

// ==================== 全屏点亮/熄灭 ====================
void lcd_all_pixel_on(bool enable)
{
    if (enable)
    {
        lcd_write_command(0xA5); // 全屏点亮
    }
    else
    {
        lcd_write_command(0xA4); // 正常显示
    }
}

typedef struct disp_menu_s
{
    const char *name;               // 菜单名
    const char *desc;               // 当前菜单描述
    struct disp_menu_s *supmenus;   // 上级菜单列表
    size_t supmenu_count;           // 上级菜单数量
    struct disp_menu_s *submenus;   // 子菜单列表
    size_t submenu_count;           // 子菜单数量
    esp_err_t (*action)(void *arg); // 菜单执行函数
    void *arg;                      // 函数执行参数
} disp_menu_t;

typedef struct
{
    int idx;
    char idxs[8];
    char name[32];
    char mac[13];
} dev_list_t;

#define MAX_DEVICE_LIST 10

dev_list_t dev_list[MAX_DEVICE_LIST];

extern disp_menu_t main_menu[];
extern disp_menu_t settings_menu[];
extern disp_menu_t check_menu[];
disp_menu_t dev_list_menu[MAX_DEVICE_LIST];

TaskHandle_t idle_disp_task_handle = NULL;

void idle_disp_task(void *arg)
{
    while (1)
    {
        /* code */
    }
}

esp_err_t bind_device(void *arg)
{
    if (g_wifi_connected)
    {
        char *url = HTTPS_HOST "/bind_device";
        char *resp = user_malloc(256, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        memset(resp, 0x00, 256);

        xTaskCreatePinnedToCore(idle_disp_task, "idle_disp_task", 1024 * 2, "BOND DEVICE", 2, &idle_disp_task_handle, 1);

        cJSON *root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "mac", g_dev_config.dev_mac);
        cJSON_AddStringToObject(root, "car", (char *)arg);
        char *body = cJSON_PrintUnformatted(root);
        cJSON_Delete(root);
        esp_err_t ret = http_post(url, body, resp, 255);
        user_free(__func__, body);
        if (ret == ESP_OK)
        {
            root = cJSON_Parse(resp);
            user_free(__func__, resp);
            if (cJSON_GetObjectItem(root, "success") && cJSON_IsTrue(cJSON_GetObjectItem(root, "success")))
            {
                sprintf(g_bind_car_dev.car, "%s", cJSON_GetObjectItem(root, "car")->valuestring);
                g_bind_car_dev.is_bind = true;

                cJSON_Delete(root);
                vTaskDelete(idle_disp_task_handle);
                return ESP_OK;
            }
            cJSON_Delete(root);
            vTaskDelete(idle_disp_task_handle);
            return ESP_FAIL;
        }
        else
        {
            user_free(__func__, resp);
            vTaskDelete(idle_disp_task_handle);
            return ESP_FAIL;
        }
    }
    return ESP_FAIL;
}

esp_err_t unbind_device(void *arg)
{
    if (g_wifi_connected)
    {
        char *url = HTTPS_HOST "/unbind_device";
        char *resp = user_malloc(256, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        memset(resp, 0x00, 256);

        xTaskCreatePinnedToCore(idle_disp_task, "idle_disp_task", 1024 * 2, "BOND DEVICE", 2, &idle_disp_task_handle, 1);

        cJSON *root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "mac", g_dev_config.dev_mac);
        char *body = cJSON_PrintUnformatted(root);
        cJSON_Delete(root);
        esp_err_t ret = http_post(url, body, resp, 255);
        user_free(__func__, body);
        if (ret == ESP_OK)
        {
            root = cJSON_Parse(resp);
            user_free(__func__, resp);
            if (cJSON_GetObjectItem(root, "success") && cJSON_IsTrue(cJSON_GetObjectItem(root, "success")))
            {
                memset(g_bind_car_dev.car, 0x00, sizeof(g_bind_car_dev.car));
                g_bind_car_dev.is_bind = false;

                cJSON_Delete(root);
                vTaskDelete(idle_disp_task_handle);
                return ESP_OK;
            }
            cJSON_Delete(root);
            vTaskDelete(idle_disp_task_handle);
            return ESP_FAIL;
        }
        else
        {
            user_free(__func__, resp);
            vTaskDelete(idle_disp_task_handle);
            return ESP_FAIL;
        }
    }
    return ESP_FAIL;
}

esp_err_t check_devices(void *arg)
{
    if (g_wifi_connected)
    {
        char *url = HTTPS_HOST "/check_devices";
        char *resp = user_malloc(256, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        memset(resp, 0x00, 256);
        xTaskCreatePinnedToCore(idle_disp_task, "idle_disp_task", 1024 * 2, "CHECK DEVICES", 2, &idle_disp_task_handle, 1);
        esp_err_t ret = http_post(url, NULL, resp, 255);
        if (ret == ESP_OK)
        {
            cJSON *root = cJSON_Parse(resp);
            user_free(__func__, resp);
            if (cJSON_GetObjectItem(root, "success") && cJSON_IsTrue(cJSON_GetObjectItem(root, "success")))
            {
                for (int i = 0; i < cJSON_GetObjectItem(root, "device_len")->valueint; i++)
                {
                    cJSON *item = cJSON_GetArrayItem(cJSON_GetObjectItem(root, "data"), i);
                    if (item)
                    {
                        cJSON *idx = cJSON_GetObjectItem(item, "idx");
                        cJSON *name = cJSON_GetObjectItem(item, "name");
                        cJSON *mac = cJSON_GetObjectItem(item, "mac");

                        dev_list[i].idx = idx->valueint;
                        itoa(dev_list[i].idx, dev_list[i].idxs, 10);
                        sprintf(dev_list[i].name, name->valuestring);
                        sprintf(dev_list[i].mac, mac->valuestring);

                        dev_list_menu[i].name = dev_list[i].mac;
                        dev_list_menu[i].desc = dev_list[i].name;

                        dev_list_menu[i].supmenus = check_menu;
                        dev_list_menu[i].supmenu_count = 3;
                        dev_list_menu[i].submenus = NULL;
                        dev_list_menu[i].submenu_count = 0;

                        dev_list_menu[i].action = bind_device;
                        dev_list_menu[i].arg = dev_list[i].mac;
                    }
                }

                check_menu[0].submenus = dev_list_menu;
                check_menu[0].submenu_count = cJSON_GetObjectItem(root, "device_len")->valueint;
                cJSON_Delete(root);
                vTaskDelete(idle_disp_task_handle);
                return ESP_OK;
            }
            cJSON_Delete(root);
            vTaskDelete(idle_disp_task_handle);
            return ESP_FAIL;
        }
        else
        {
            user_free(__func__, resp);
            vTaskDelete(idle_disp_task_handle);
            return ESP_FAIL;
        }
    }
    return ESP_FAIL;
}

disp_menu_t check_menu[] = {
    {"bond_disp", "bond display", settings_menu, 3, NULL, 0, NULL, NULL},
};

disp_menu_t settings_menu[] = {
    {"check_disp", "check display", main_menu, 2, check_menu, 1, check_devices, NULL},
    {"unbond_disp", "unbond display", main_menu, 2, NULL, 0, unbind_device, NULL},
    {"reset", "reset device", main_menu, 2, NULL, 0, NULL, NULL},
};

disp_menu_t main_menu[] = {
    {"default_disp", "default display", NULL, 0, NULL, 0, NULL, NULL},
    {"setting_disp", "setting display", NULL, 0, settings_menu, 3, NULL, NULL},
};

void draw_dashboard_screen()
{
    // TODO: 调用 LCD 绘图 API
    // 显示 g_battery_level
    // 显示 g_joy_x, g_joy_y
    // printf("DASHBOARD: Bat:%d Joy:%d,%d\n", g_battery_level, g_joy_x, g_joy_y);
}

void draw_menu_screen(disp_menu_t *menu, int len, int sel)
{
    for (int i = 0; i < menu->submenu_count; i++)
    {
    }
}

typedef struct
{
    int type; // 0: 摇杆, 1: 按键
    int id;   // 按键ID
    key_value_t key_val;
    joystick_vatual_button_t joy_val;
} input_event_t;

QueueHandle_t input_queue;

void read_input_value_task()
{
    while (1)
    {
        input_event_t evt = {0};
        joystick_vatual_button_t vitual_button = joystick_vitual_idle;

        vitual_button = read_left_joystick_postion();
        if (vitual_button == joystick_vitual_left || vitual_button == joystick_vitual_right)
        {
            evt.type = 0;
            evt.id = 0;
            evt.joy_val = vitual_button;
            xQueueSend(input_queue, &evt, pdMS_TO_TICKS(50));
        }

        vitual_button = read_right_joystick_postion();
        if (vitual_button == joystick_vitual_up || vitual_button == joystick_vitual_down)
        {
            evt.type = 0;
            evt.id = 0;
            evt.joy_val = vitual_button;
            xQueueSend(input_queue, &evt, pdMS_TO_TICKS(50));
        }

        for (int i = 0; i < KEY_NUM; i++)
        {
            if (key_value[i].key_status != KEY_IDLE)
            {
                evt.type = 1;
                evt.id = i;
                evt.key_val = key_value[i];
                xQueueSend(input_queue, &evt, pdMS_TO_TICKS(50));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void display_task(void *arg)
{
    // 0: 仪表盘模式 1: 设置菜单模式
    int disp_mode = 0;
    disp_menu_t *curr_menu = main_menu;
    int curr_menu_len = 1;
    int cursor = 0;

    input_event_t evt;
    while (1)
    {
        if (xQueueReceive(input_queue, &evt, pdMS_TO_TICKS(20)) == pdTRUE)
        {
            if (disp_mode == 0)
            {
                switch (evt.type)
                {
                case 0:
                    break;
                case 1:
                {
                    switch (evt.id)
                    {
                    case 0:
                        break;
                    case 1:
                        break;
                    case 2:
                        break;
                    case 3:
                    {
                        if (evt.key_val.key_status == KEY_LONG_PRESSED)
                            if (evt.key_val.key_fsm_finished == true)
                                disp_mode = 1;
                    }
                    break;

                    default:
                        break;
                    }
                }
                break;

                default:
                    break;
                }
            }
            else if (disp_mode == 1)
            {
                switch (evt.type)
                {
                case 0:
                {
                    switch (evt.id)
                    {
                    case joystick_vitual_idle:
                        break;
                    case joystick_vitual_up:
                    {
                        cursor = (cursor - 1 + curr_menu_len) % curr_menu_len;
                    }
                    break;
                    case joystick_vitual_down:
                    {
                        cursor = (cursor + 1 + curr_menu_len) % curr_menu_len;
                    }
                    break;
                    case joystick_vitual_left:
                        break;
                    case joystick_vitual_right:
                        break;

                    default:
                        break;
                    }
                }
                break;
                case 1:
                {
                    switch (evt.id)
                    {
                    case 0:
                        break;
                    case 1: // 确认(Action)
                    {
                        if (curr_menu[cursor].action != NULL)
                        {
                            if (curr_menu[cursor].action(curr_menu[cursor].arg) != ESP_OK)
                            {
                                ESP_LOGI(__func__, "menu: %s action false", curr_menu[cursor].desc);
                                curr_menu[cursor].action(curr_menu[cursor].arg);
                            }
                            else
                            {
                                ESP_LOGI(__func__, "menu: %s action true", curr_menu[cursor].desc);
                            }
                        }

                        if (curr_menu[cursor].submenus != NULL)
                        {
                            cursor = 0;
                            curr_menu_len = curr_menu[cursor].submenu_count;
                            curr_menu = curr_menu[cursor].submenus;
                        }

                        if (curr_menu[cursor].action == NULL && curr_menu[cursor].submenus == NULL)
                        {
                            disp_mode = 0;
                            cursor = 0;
                            curr_menu_len = 2;
                            curr_menu = main_menu;
                        }
                    }
                    break;
                    case 2:
                        break;
                    case 3: // 取消(Back)
                    {
                        if (curr_menu->supmenus != NULL)
                        {
                            cursor = 0;
                            curr_menu_len = curr_menu[cursor].supmenu_count;
                            curr_menu = curr_menu[cursor].supmenus;
                        }
                        else // joycon defalut display
                        {
                            disp_mode = 0;
                            cursor = 0;
                            curr_menu_len = 2;
                            curr_menu = main_menu;
                        }
                    }
                    break;

                    default:
                        break;
                    }
                }
                break;

                default:
                    break;
                }
            }
        }

        if (disp_mode == 0)
        {
            draw_dashboard_screen();
        }
        else
        {
            draw_menu_screen(curr_menu, curr_menu_len, cursor);
        }
    }
}
