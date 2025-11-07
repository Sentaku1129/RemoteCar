#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bsp_display.h"

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
        .mosi_io_num = io_LCD_SDA, // MOSI 引脚
        .miso_io_num = -1,           // LCD 不需要 MISO
        .sclk_io_num = io_LCD_SCLK, // SCLK 引脚
        .quadwp_io_num = -1,         // 不使用 WP
        .quadhd_io_num = -1,         // 不使用 HD
        .max_transfer_sz = 4096,     // 最大传输大小
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
        .spics_io_num = io_LCD_CS,      // CS 引脚
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

void display_task(void *arg)
{
    while (1)
    {
           
    }
}