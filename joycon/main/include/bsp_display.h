#ifndef _BSP_DISPLAY_H_
#define _BSP_DISPLAY_H_

// SPI 配置
#define LCD_HOST SPI2_HOST
#define LCD_SPI_CLOCK 8000000

#define io_LCD_CS     23  //LCD_CS      LCD片选
#define io_LCD_RST    22  //LCD_RES     LCD复位
#define io_LCD_AO     21  //LCD_AO      LCD模式选择 命令模式或者数据模式
#define io_LCD_SCLK   19  //LCD_SCLK    LCD时钟
#define io_LCD_SDA    18  //LCD_SDA     LCD数据

// MENU LIST
#define MAX_DEVICE_LIST 10

// 布局配置
#define MENU_HEADER_H   16  // 标题高度
#define MENU_ITEM_H     16  // 列表项高度
#define SCROLL_W        3   // 滚动条宽度

// LCD SCREEN
#define LCD_WIDTH 128
#define LCD_HEIGHT 64
#define LCD_PAGES (LCD_HEIGHT / 8)

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

// menu dec
extern disp_menu_t main_menu[];
extern disp_menu_t settings_menu[];
extern disp_menu_t check_menu[];

void display_task(void *arg);

#endif
