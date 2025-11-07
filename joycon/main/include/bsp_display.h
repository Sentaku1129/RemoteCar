#ifndef _BSP_DISPLAY_H_
#define _BSP_DISPLAY_H_

#define io_LCD_CS     23  //LCD_CS      LCD片选
#define io_LCD_RST    22  //LCD_RES     LCD复位
#define io_LCD_AO     21  //LCD_AO      LCD模式选择 命令模式或者数据模式
#define io_LCD_SCLK   19  //LCD_SCLK    LCD时钟
#define io_LCD_SDA    18  //LCD_SDA     LCD数据

#define i2c_LCD_SCLK  19
#define i2c_LCD_SDA   18
#define i2c_LCD_PORT  0

void display_init();

#endif
