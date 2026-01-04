#include <string.h>
#include <math.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cJSON.h"

#include "bsp_display.h"
#include "bsp_http.h"
#include "bsp_config.h"
#include "bsp_key_fsm.h"
#include "bsp_joystick.h"

#include "lcd_font.h"

bool remote_led_status = false;

static spi_device_handle_t spi_handle = NULL;

uint8_t lcd_frame_buffer[LCD_WIDTH * LCD_PAGES] = {0};

dev_list_t dev_list[MAX_DEVICE_LIST];
disp_menu_t dev_list_menu[MAX_DEVICE_LIST];
disp_menu_t *dev_list_ptrs = NULL;
int dev_list_count = 0;
TaskHandle_t idle_disp_task_handle = NULL;
SemaphoreHandle_t lcd_spi_mutex = NULL;

// ==================== LCD 复位 ====================
void lcd_reset(void)
{
    gpio_set_level(io_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(io_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(__func__, "LCD 复位完成");
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
        ESP_LOGE(__func__, "SPI 总线初始化失败: %s", esp_err_to_name(ret));
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
        ESP_LOGE(__func__, "SPI 设备添加失败: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(__func__, "硬件 SPI 初始化完成");
    ESP_LOGI(__func__, "SPI 配置: SCLK=%d, MOSI=%d, CS=%d, DC=%d, RST=%d",
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
    xSemaphoreTake(lcd_spi_mutex, portMAX_DELAY);
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &trans);
    xSemaphoreGive(lcd_spi_mutex);
    if (ret != ESP_OK)
    {
        ESP_LOGE(__func__, "写命令失败: 0x%02X", cmd);
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
    
    xSemaphoreTake(lcd_spi_mutex, portMAX_DELAY);
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &trans);
    xSemaphoreGive(lcd_spi_mutex);
    if (ret != ESP_OK)
    {
        ESP_LOGE(__func__, "lcd send data error: 0x%02X", data);
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
    
    xSemaphoreTake(lcd_spi_mutex, portMAX_DELAY);
    esp_err_t ret = spi_device_polling_transmit(spi_handle, &trans);
    xSemaphoreGive(lcd_spi_mutex);
    if (ret != ESP_OK)
    {
        ESP_LOGE(__func__, "lcd send long data error, length: %d", len);
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

    ESP_LOGI(__func__, "ST7567 LCD 初始化完成");
}

// 清除显存
void lcd_clear_buffer(void)
{
    memset(lcd_frame_buffer, 0x00, sizeof(lcd_frame_buffer));
}

// 缓存刷新写入lcd
void lcd_refresh_frame(void)
{
    uint8_t page;
    for (page = 0; page < LCD_PAGES; page++)
    {
        lcd_write_command(0xB0 + page);
        lcd_write_command(0x10); // 设置列地址高 4 位
        lcd_write_command(0x00); // 设置列地址低 4 位

        lcd_write_data_bulk(&lcd_frame_buffer[page * LCD_WIDTH], LCD_WIDTH);
    }
}

// ==================== 清屏 ====================
void lcd_clear(void)
{
    lcd_clear_buffer();
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

/**
 * @brief 画点
 * @param color 1=亮, 0=灭
 */
void lcd_draw_pixel(uint8_t x, uint8_t y, uint8_t color)
{
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT)
        return;
    uint8_t page = y / 8;
    uint8_t bit = y % 8;

    if (color)
        lcd_frame_buffer[page * LCD_WIDTH + x] |= (1 << bit);
    else
        lcd_frame_buffer[page * LCD_WIDTH + x] &= ~(1 << bit);
}

/**
 * @brief 画横线
 */
void lcd_draw_h_line(uint8_t x_start, uint8_t x_end, uint8_t y, uint8_t color)
{
    if (x_start > x_end)
    {
        uint8_t temp = x_start;
        x_start = x_end;
        x_end = temp;
    }

    // 遍历绘制
    for (uint8_t x = x_start; x <= x_end; x++)
    {
        lcd_draw_pixel(x, y, color);
    }
}

/**
 * @brief 画竖线
 */
void lcd_draw_v_line(uint8_t x, uint8_t y_start, uint8_t y_end, uint8_t color)
{
    if (y_end < y_start)
    {
        uint8_t temp = y_start;
        y_start = y_end;
        y_end = temp;
    }
    for (uint8_t y = y_start; y <= y_end; y++)
    {
        lcd_draw_pixel(x, y, color);
    }
}

/**
 * @brief 画空心矩形
 * @param x      左上角 X 坐标
 * @param y      左上角 Y 坐标
 * @param w      宽度
 * @param h      高度
 * @param color  1=亮, 0=灭
 */
void lcd_draw_rect_empty(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color)
{
    // 参数检查，防止尺寸为0导致错误
    if (w == 0 || h == 0)
        return;

    // 1. 上横线
    lcd_draw_h_line(x, x + w - 1, y, color);

    // 2. 下横线
    lcd_draw_h_line(x, x + w - 1, y + h - 1, color);

    // 3. 左竖线
    lcd_draw_v_line(x, y, y + h - 1, color);

    // 4. 右竖线
    lcd_draw_v_line(x + w - 1, y, y + h - 1, color);
}

/**
 * @brief 画实心矩形
 */
void lcd_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color)
{
    for (uint8_t i = 0; i < w; i++)
        lcd_draw_v_line(x + i, y, y + h - 1, color);
}

/**
 * @brief 绘制字符 (核心函数)
 * @param invert false=正常, true=反色(背景亮字灭)
 */
static void lcd_draw_char_core(uint8_t x, uint8_t y, uint8_t width, uint8_t height, char c, bool invert)
{
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT)
        return;

    const uint8_t *char_ptr = &LCD_F6x12[c - 0x20][0];
    uint8_t y_page_base = y / 8;
    uint8_t y_shift = y % 8;
    uint8_t font_pages = (height + 7) / 8;

    for (uint8_t i = 0; i < width; i++)
    {
        if (x + i >= LCD_WIDTH)
            break;

        for (uint8_t p = 0; p < font_pages; p++)
        {
            uint8_t target_page = y_page_base + p;
            if (target_page >= LCD_PAGES)
                break;

            uint8_t src_data = char_ptr[p * width + i];
            if (invert)
                src_data = ~src_data; // 反色核心逻辑

            uint16_t idx = target_page * LCD_WIDTH + (x + i);

            // 写入当前页
            if (invert)
                lcd_frame_buffer[idx] &= ~(src_data << y_shift); // 反色用 AND NOT (需先画实心背景)
            else
                lcd_frame_buffer[idx] |= (src_data << y_shift); // 正常用 OR

            // 处理跨页
            if (y_shift > 0 && (target_page + 1) < LCD_PAGES)
            {
                if (invert)
                    lcd_frame_buffer[idx + LCD_WIDTH] &= ~(src_data >> (8 - y_shift));
                else
                    lcd_frame_buffer[idx + LCD_WIDTH] |= (src_data >> (8 - y_shift));
            }
        }
    }
}

/**
 * @brief 正常绘制字符
 */
void lcd_draw_char_buffer(uint8_t x, uint8_t y, uint8_t width, uint8_t height, char c)
{
    lcd_draw_char_core(x, y, width, height, c, false);
}

/**
 * @brief 反色绘制字符 (需先手动绘制背景块)
 */
void lcd_draw_char_buffer_invert(uint8_t x, uint8_t y, uint8_t width, uint8_t height, char c)
{
    // 注意：调用此函数前，建议先用 lcd_fill_rect 在该位置画一个实心矩形作为背景
    lcd_draw_char_core(x, y, width, height, c, true);
}

/**
 * @brief 显示字符串
 */
void lcd_showString_buffer(uint8_t x, uint8_t y, char *str)
{
    while (*str)
    {
        lcd_draw_char_buffer(x, y, 6, 12, *str);
        x += 6;
        str++;
        if (x >= LCD_WIDTH)
            break;
    }
}

/**
 * @brief 反色显示字符串 (自动填充背景)
 */
void lcd_showString_buffer_invert(uint8_t x, uint8_t y, char *str)
{
    // 1. 计算背景宽并填充
    int len = strlen(str);
    lcd_fill_rect(x, y, len * 6, 12, 1); // 画实心黑块

    // 2. 抠字
    while (*str)
    {
        lcd_draw_char_buffer_invert(x, y, 6, 12, *str);
        x += 6;
        str++;
        if (x >= LCD_WIDTH)
            break;
    }
}

// 辅助函数：画圆 (Bresenham 算法)
void lcd_draw_circle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color)
{
    int a = 0, b = r;
    int di = 3 - 2 * r;
    while (a <= b)
    {
        lcd_draw_pixel(x0 + a, y0 - b, color);
        lcd_draw_pixel(x0 + b, y0 - a, color);
        lcd_draw_pixel(x0 + b, y0 + a, color);
        lcd_draw_pixel(x0 + a, y0 + b, color);
        lcd_draw_pixel(x0 - a, y0 + b, color);
        lcd_draw_pixel(x0 - b, y0 + a, color);
        lcd_draw_pixel(x0 - b, y0 - a, color);
        lcd_draw_pixel(x0 - a, y0 - b, color);
        if (di < 0)
            di += 4 * a + 6;
        else
        {
            di += 4 * (a - b) + 10;
            b--;
        }
        a++;
    }
}

// 辅助函数：画实心圆 (用于画转动的球)
void lcd_fill_circle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color)
{
    int a = 0, b = r;
    int di = 3 - 2 * r;
    while (a <= b)
    {
        for (int i = x0 - b; i <= x0 + b; i++)
        {
            lcd_draw_pixel(i, y0 - a, color);
            lcd_draw_pixel(i, y0 + a, color);
        }
        for (int i = x0 - a; i <= x0 + a; i++)
        {
            lcd_draw_pixel(i, y0 - b, color);
            lcd_draw_pixel(i, y0 + b, color);
        }
        if (di < 0)
            di += 4 * a + 6;
        else
        {
            di += 4 * (a - b) + 10;
            b--;
        }
        a++;
    }
}

void idle_disp_task(void *arg)
{
    // 1. 获取标题 (任务参数)
    char *title = (char *)arg;

    // 2. 动画参数
    const uint8_t CENTER_X = LCD_WIDTH / 2;
    const uint8_t CENTER_Y = LCD_HEIGHT / 2 + 8; // 稍微偏下，留出标题位置
    const uint8_t RADIUS_ORBIT = 12;             // 轨道半径
    const uint8_t RADIUS_BALL = 3;               // 小球半径

    int angle = 0; // 角度 0-360

    while (1)
    {
        lcd_clear_buffer();

        // --- 绘制标题 ---
        if (title)
        {
            // 居中计算 (假设每个字宽6)
            int str_w = strlen(title) * 6;
            int start_x = (LCD_WIDTH - str_w) / 2;
            if (start_x < 0)
                start_x = 0;
            lcd_showString_buffer(start_x, 0, title);
        }

        // --- 绘制分割线 ---
        lcd_draw_v_line(0, 12, LCD_WIDTH, 1);

        // --- 绘制中心装饰 (可选：中心画一个小点) ---
        lcd_draw_pixel(CENTER_X, CENTER_Y, 1);

        // --- 计算小球坐标 ---
        // x = cx + r * cos(a)
        // y = cy + r * sin(a)
        // 注意：C语言 sin/cos 需要弧度：角度 * PI / 180
        float radian = angle * 3.1415926 / 180.0;
        int ball_x = CENTER_X + (int)(RADIUS_ORBIT * cos(radian));
        int ball_y = CENTER_Y + (int)(RADIUS_ORBIT * sin(radian));

        // --- 绘制轨道 (虚线圆，可选) ---
        // lcd_draw_circle(CENTER_X, CENTER_Y, RADIUS_ORBIT, 1);

        // --- 绘制旋转的小球 ---
        lcd_fill_circle(ball_x, ball_y, RADIUS_BALL, 1);

        // --- 绘制尾巴 (可选：让动画看起来有拖影) ---
        for (int i = 1; i <= 3; i++)
        {
            float tail_rad = (angle - i * 20) * 3.1415926 / 180.0;
            int tail_x = CENTER_X + (int)(RADIUS_ORBIT * cos(tail_rad));
            int tail_y = CENTER_Y + (int)(RADIUS_ORBIT * sin(tail_rad));
            lcd_draw_pixel(tail_x, tail_y, 1); // 尾巴只画点
        }

        // --- 刷新屏幕 ---
        lcd_refresh_frame();

        // --- 更新状态 ---
        angle += 30; // 每次转30度
        if (angle >= 360)
            angle = 0;

        // --- 延时 (控制转速) ---
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ==================== 仪表盘组件绘制 ====================

/**
 * @brief 绘制电池图标
 * @param x, y 起始坐标 (右上角位置建议 x=110, y=2)
 */
void draw_battery_widget(uint8_t x, uint8_t y)
{
    // 边界检查，防止溢出
    if (x + 20 > LCD_WIDTH || y + 8 > LCD_HEIGHT)
    {
        ESP_LOGE("LCD", "Battery widget exceeds screen boundaries");
        return;
    }

    // 绘制电池边框 (宽16, 高8)
    lcd_fill_rect(x + 16, y + 5, 2, 2, 1);   // 电池头
    lcd_draw_rect_empty(x, y + 2, 16, 8, 1); // 电池框
    lcd_fill_rect(x, y + 2, 16 * g_battery_level / 100, 8, 1);

    // 显示电量百分比
    char buf[5];
    sprintf(buf, "%d%%", g_battery_level);
    lcd_showString_buffer(x - 28, y, buf);
}

/**
 * @brief 绘制WiFi信号图标
 * @param x, y 起始坐标
 */
void draw_wifi_widget(uint8_t x, uint8_t y)
{
    // 简单根据 RSSI 分级
    int bars = 0;
    if (g_wifi_rssi > -50)
        bars = 4;
    else if (g_wifi_rssi > -70)
        bars = 3;
    else if (g_wifi_rssi > -80)
        bars = 2;
    else
        bars = 1;

    if (g_wifi_connected)
    {
        if (bars >= 1)
            lcd_fill_rect(x, y + 6, 2, 2, 1);

        if (bars >= 2)
            lcd_fill_rect(x + 4, y + 4, 2, 4, 1);

        if (bars >= 3)
            lcd_fill_rect(x + 8, y + 2, 2, 6, 1);

        if (bars >= 4)
            lcd_fill_rect(x + 12, y, 2, 8, 1);
    }
    else
    {
        lcd_fill_rect(x, y + 6, 14, 2, 1);
        lcd_draw_rect_empty(x, y, 14, 4, 1);
    }
}

/**
 * @brief 绘制摇杆状态可视化
 * @param cx, cy 圆心坐标
 * @param r      外圈半径
 * @param joy    归一化数据
 */
void draw_joystick_visual(uint8_t cx, uint8_t cy, uint8_t r, joystick_normalized_t joy)
{
    // 1. 绘制外框 (轨道)
    lcd_draw_circle(cx, cy, r, 1);

    // 2. 绘制中心准星 (可选)
    lcd_draw_pixel(cx, cy, 1);
    lcd_draw_pixel(cx - 1, cy, 1);
    lcd_draw_pixel(cx + 1, cy, 1);
    lcd_draw_pixel(cx, cy - 1, 1);
    lcd_draw_pixel(cx, cy + 1, 1);

    // 3. 计算摇杆球的位置
    // joy.x, joy.y 范围 -1.0 ~ 1.0
    // 限制活动范围在半径内 (稍微留2像素边距，防止球画出圈外)
    int max_move = r - 4;

    int dx = (int)(joy.x * max_move);
    int dy = (int)(joy.y * max_move);

    // ESP_LOGI(__func__, "dx: %d, dy: %d",dx, dy);

    // 4. 绘制实心小球 (代表摇杆头)
    // 半径为 3
    lcd_fill_circle(cx + dx, cy - dy, 3, 1);
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

esp_err_t reset_device(void *arg)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_SYS_CONFIG, NVS_READWRITE, &nvs_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(__func__, "open nvs error: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = nvs_erase_all(nvs_handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(__func__, "erase nvs error: %s", esp_err_to_name(ret));
        nvs_close(nvs_handle);
        return ret;
    }
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    esp_restart();
    return ESP_OK;
}

esp_err_t joy_adjust(void *arg)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret = nvs_open(NVS_SYS_CONFIG, NVS_READWRITE, &nvs_handle);
    xTaskCreatePinnedToCore(idle_disp_task, "idle_disp_task", 1024 * 2, "JOY ADJUST", 2, &idle_disp_task_handle, 1);
    if (ret != ESP_OK)
    {
        ESP_LOGE(__func__, "open nvs error: %s", esp_err_to_name(ret));
        vTaskDelete(idle_disp_task_handle);
        return ret;
    }

    joy_adjust_value_t joy_adjust_value = read_joy_adjust_offset();
    memccpy(&joy_adjust_offset_value, &joy_adjust_value, sizeof(joy_adjust_value_t));

    ret = nvs_set_blob(nvs_handle, NVS_SYS_LEFT_X_OFFSET, &joy_adjust_value.left.x, sizeof(float));
    ret = nvs_set_blob(nvs_handle, NVS_SYS_LEFT_Y_OFFSET, &joy_adjust_value.left.y, sizeof(float));
    ret = nvs_set_blob(nvs_handle, NVS_SYS_RIGHT_X_OFFSET, &joy_adjust_value.right.x, sizeof(float));
    ret = nvs_set_blob(nvs_handle, NVS_SYS_RIGHT_Y_OFFSET, &joy_adjust_value.right.y, sizeof(float));

    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    vTaskDelete(idle_disp_task_handle);
    return ESP_OK;
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
            ESP_LOGI(__func__, "check devices resp: %s", resp);
            cJSON *root = cJSON_Parse(resp);
            user_free(__func__, resp);
            if (cJSON_GetObjectItem(root, "success") && cJSON_IsTrue(cJSON_GetObjectItem(root, "success")))
            {
                for (int i = 0; i < cJSON_GetObjectItem(root, "device_len")->valueint && i < 10; i++)
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

                check_menu[0].submenus = cJSON_GetObjectItem(root, "device_len")->valueint > 0 ? dev_list_menu : NULL;
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
    {"joy_cor", "joy adjust", main_menu, 2, NULL, 0, joy_adjust, NULL},
    {"reset", "reset device", main_menu, 2, NULL, 0, reset_device, NULL},
};

disp_menu_t main_menu[] = {
    {"default_disp", "default display", NULL, 0, NULL, 0, NULL, NULL},
    {"setting_disp", "setting display", NULL, 0, settings_menu, 4, NULL, NULL},
};

void draw_dashboard_screen()
{
    // 1. 清屏
    lcd_clear_buffer();

    // ================== 状态栏绘制 ==================

    // 绘制 WiFi (左上角)
    draw_wifi_widget(2, 4);

    // 绘制 标题 (顶部居中)
    // 假设 LCD_WIDTH = 128, "DASHBOARD" 宽约 54px
    lcd_showString_buffer(38, 2, "REMOTE");

    // 绘制 电池 (右上角)
    draw_battery_widget(LCD_WIDTH - 20, 2);

    // ================== 摇杆区域绘制 ==================

    // 屏幕下半部分高度约 52，中心 Y 约为 12 + 26 = 38
    // 两个圆心 X 坐标：
    // 左摇杆: 128 * 1/4 = 32
    // 右摇杆: 128 * 3/4 = 96

    uint8_t joy_y = 34;
    uint8_t joy_r = 17; // 半径18，直径36，上下留空

    // --- 左摇杆 (L) ---
    draw_joystick_visual(32, joy_y, joy_r, joycon_value_L);
    // 标注 L
    lcd_showString_buffer(32 - 3, joy_y + joy_r + 2, "L");

    // --- 右摇杆 (R) ---
    draw_joystick_visual(96, joy_y, joy_r, joycon_value_R);
    // 标注 R
    lcd_showString_buffer(96 - 3, joy_y + joy_r + 2, "R");

    // ESP_LOGI(__func__, "L: x=%.2f y=%.2f; R: x=%.2f y=%.2f", joycon_value_L.x, joycon_value_L.y, joycon_value_R.x, joycon_value_R.y);

    // ================== 数据数值显示 (可选) ==================
    // 如果你想在屏幕中间显示具体数值，可以加在这里
    // char buf[16];
    // sprintf(buf, "X:%.1f", joycon_value_L.x);
    // lcd_showString_buffer(54, 20, buf);

    // 2. 提交显存刷新
    lcd_refresh_frame();
}

void draw_menu_screen(const char *menu_name, disp_menu_t *menu, int len, int sel)
{
    lcd_clear_buffer();

    // 1. 绘制标题栏
    // lcd_showString_buffer(42, 2, "MENU");
    if (strlen(menu_name) * 6 > LCD_WIDTH)
    {
        ESP_LOGW("LCD", "String too long to display!");
        char truncated[16];
        snprintf(truncated, sizeof(truncated), "%.15s", (char *)menu_name); // 截断到15字符
        lcd_showString_buffer(64 - (strlen(truncated) * 6) / 2, 2, truncated);
    }
    else
    {
        lcd_showString_buffer(64 - (strlen(menu_name) * 6) / 2, 2, (char *)menu_name);
    }

    // 2. 计算滚动窗口
    int visible_cnt = (LCD_HEIGHT - MENU_HEADER_H) / MENU_ITEM_H;
    int start_idx = 0;

    if (sel >= visible_cnt)
        start_idx = sel - visible_cnt + 1;

    if (start_idx > len - visible_cnt)
        start_idx = len - visible_cnt;

    if (start_idx < 0)
        start_idx = 0;

    // 3. 绘制可见列表项
    for (int i = 0; i < visible_cnt; i++)
    {
        int curr_idx = start_idx + i;
        if (curr_idx >= len)
            break;

        uint8_t y = MENU_HEADER_H + (i * MENU_ITEM_H);
        disp_menu_t *item = &menu[curr_idx];

        if (curr_idx == sel)
        {
            // 选中：绘制实心背景条 + 反色文字
            lcd_fill_circle(5, y + MENU_ITEM_H / 2, 4, 1); // 选中圆圈背景
            lcd_showString_buffer(12, y + 2, (char *)item->desc);
        }
        else
        {
            lcd_showString_buffer(2, y + 2, (char *)item->desc);
        }
    }

    // 4. 绘制滚动条 (仅当溢出时)
    if (len > visible_cnt)
    {
        const uint8_t BAR_X = LCD_WIDTH - SCROLL_W - 1;
        const uint8_t TRACK_H = LCD_HEIGHT - MENU_HEADER_H;

        // 绘制轨道线
        lcd_draw_v_line(BAR_X + 1, MENU_HEADER_H, LCD_HEIGHT - 1, 1);

        // 计算滑块高度 (最小4像素)
        int thumb_h = (TRACK_H * visible_cnt) / len;
        if (thumb_h < 4)
            thumb_h = 4;

        // 计算滑块位置 (线性映射)
        int thumb_y = MENU_HEADER_H;
        thumb_y += (sel * (TRACK_H - thumb_h)) / (len - 1);

        // 绘制滑块
        lcd_fill_rect(BAR_X, thumb_y, SCROLL_W, thumb_h, 1);
    }

    lcd_refresh_frame();
}

typedef struct
{
    int type; // 0: 摇杆, 1: 按键, 2: 摇杆值
    int id;   // 按键ID
    key_value_t key_val;
    joystick_vatual_button_t joy_val;
    joystick_normalized_t nor_val;
} input_event_t;

QueueHandle_t input_queue;

void read_input_value_task()
{
    static TickType_t last_wake_time = 0;
    while (1)
    {
        input_event_t evt = {0};
        joystick_vatual_button_t vitual_button = joystick_vitual_idle;
        if (xTaskGetTickCount() - last_wake_time > pdMS_TO_TICKS(200))
        {
            bool joystick_active = false;
            vitual_button = read_left_joystick_postion();
            if (vitual_button == joystick_vitual_left || vitual_button == joystick_vitual_right)
            {
                evt.type = 0;
                evt.id = 0;
                evt.joy_val = vitual_button;
                xQueueSend(input_queue, &evt, pdMS_TO_TICKS(50));
                joystick_active = true;
            }

            vitual_button = read_right_joystick_postion();
            if (vitual_button == joystick_vitual_up || vitual_button == joystick_vitual_down)
            {
                evt.type = 0;
                evt.id = 0;
                evt.joy_val = vitual_button;
                xQueueSend(input_queue, &evt, pdMS_TO_TICKS(50));
                joystick_active = true;
            }
            if (joystick_active)
            {
                last_wake_time = xTaskGetTickCount();
            }
        }

        for (int i = 0; i < KEY_NUM; i++)
        {
            if (key_value[i].key_status != KEY_IDLE)
            {
                evt.type = 1;
                evt.id = i;
                evt.key_val = key_value[i];

                if (key_value[i].key_fsm_finished)
                {
                    key_value[i].key_status = KEY_IDLE;
                    key_value[i].key_value = 0;
                    key_value[i].key_fsm_finished = false;
                }

                ESP_LOGI(__func__, "status = %d, value = %lld, fsm_finished = %s", evt.key_val.key_status, evt.key_val.key_value, evt.key_val.key_fsm_finished ? "true" : "false");
                xQueueSend(input_queue, &evt, pdMS_TO_TICKS(50));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void read_joystick_value_task(void *arg)
{
    while (1)
    {
        input_event_t evt = {
            .type = 2,
            .nor_val = {
                .y = joycon_value_L.y - joy_adjust_offset_value.left.y,
                .x = joycon_value_R.x - joy_adjust_offset_value.right.x,
            },
        };

        xQueueSend(input_queue, &evt, pdMS_TO_TICKS(50));

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void show_refresh_rate()
{
    static TickType_t last_tick = 0;
    TickType_t now = xTaskGetTickCount();

    if (last_tick != 0)
    {
        TickType_t dt_ticks = now - last_tick;
        if (dt_ticks > 0)
        {
            uint32_t dt_ms = dt_ticks * portTICK_PERIOD_MS;
            uint32_t fps = 1000U / dt_ms; // integer fps
            ESP_LOGI(__func__, "fps = %" PRIu32, fps);
        }
        else
        {
            ESP_LOGI(__func__, "fps = inf (dt_ticks=0)");
        }
    }

    last_tick = now;
}

void display_task(void *arg)
{
    lcd_spi_mutex = xSemaphoreCreateMutex();
    lcd_st7567_init();

    // 0: 仪表盘模式 1: 设置菜单模式
    int disp_mode = 0;
    disp_menu_t *curr_menu = main_menu;
    int curr_menu_len = 2;
    int cursor = 0;

    input_queue = xQueueCreate(5, sizeof(input_event_t));
    xTaskCreatePinnedToCore(read_input_value_task, "read_input_value_task", 1024 * 4, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(read_joystick_value_task, "read_joystick_value_task", 1024 * 4, NULL, 3, NULL, 0);

    input_event_t evt;
    while (1)
    {
        if (xQueueReceive(input_queue, &evt, pdMS_TO_TICKS(10)) == pdTRUE)
        {
            if (disp_mode == 0)
            {
                switch (evt.type)
                {
                case 0:
                {
                }
                break;
                case 1:
                {
                    ESP_LOGI(__func__, "id = %d", evt.id);
                    switch (evt.id)
                    {
                    case 0:
                    {
                        if (g_bind_car_dev.is_bind)
                        {
                            remote_led_status = !remote_led_status;
                            cJSON *root = cJSON_CreateObject();
                            cJSON_AddNumberToObject(root, "code", 201);
                            cJSON_AddBoolToObject(root, "data", remote_led_status);
                            cJSON_AddStringToObject(root, "remote", g_bind_car_dev.car);
                            char *publish = cJSON_PrintUnformatted(root);
                            mqtt_message_t msg = {
                                .data = publish,
                                .dynamic = true,
                            };
                            if (xQueueSend(publish_queue, &msg, pdMS_TO_TICKS(50)) != pdTRUE)
                            {
                                user_free(__func__, publish);
                            }
                            cJSON_Delete(root);
                        }
                    }
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
                case 2:
                {
                    if (g_bind_car_dev.is_bind)
                    {
                        cJSON *root = cJSON_CreateObject();
                        cJSON_AddNumberToObject(root, "code", 101);
                        cJSON *data = cJSON_CreateObject();
                        cJSON_AddNumberToObject(data, "left", evt.nor_val.y);
                        cJSON_AddNumberToObject(data, "right", evt.nor_val.x);
                        cJSON_AddItemToObject(root, "data", data);
                        cJSON_AddStringToObject(root, "remote", g_bind_car_dev.car);
                        char *publish = cJSON_PrintUnformatted(root);
                        mqtt_message_t msg = {
                            .data = publish,
                            .dynamic = true,
                        };
                        if (xQueueSend(publish_queue, &msg, pdMS_TO_TICKS(50)) != pdTRUE)
                        {
                            ESP_LOGI(__func__, "nor data");
                            user_free(__func__, publish);
                        }
                        cJSON_Delete(root);
                    }
                }

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
                    switch (evt.joy_val)
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
                        if (evt.key_val.key_fsm_finished)
                        {
                            if (curr_menu[cursor].action != NULL)
                            {
                                if (curr_menu[cursor].action(curr_menu[cursor].arg) != ESP_OK)
                                {
                                    ESP_LOGI(__func__, "menu: %s action false", curr_menu[cursor].desc);
                                }
                                else
                                {
                                    ESP_LOGI(__func__, "menu: %s action true", curr_menu[cursor].desc);
                                }
                            }

                            if (curr_menu[cursor].submenus != NULL)
                            {
                                curr_menu_len = curr_menu[cursor].submenu_count;
                                curr_menu = curr_menu[cursor].submenus;
                                cursor = 0;
                            }
                        }
                    }
                    break;
                    case 2:
                        break;
                    case 3: // 取消(Back)
                    {
                        if (evt.key_val.key_fsm_finished)
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
            draw_menu_screen(curr_menu->supmenus ? curr_menu->supmenus->desc : "MAIN MENU", curr_menu, curr_menu_len, cursor);
        }

        // show_refresh_rate();
    }
}
