#ifndef _BSP_CONFI_
#define _BSP_CONFI_

#define user_malloc(size, caps) heap_caps_malloc(size, caps)
#define user_iram_malloc(size, caps) heap_caps_malloc(size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM)
#define user_free(msg, ptr) free(ptr)

#define HTTPS_HOST "https://smart-car.sentaku.trade"

typedef enum
{
    beep_none = 0,
    beep_button,
    beep_continue,
} beep_msg_t;

typedef struct
{
    char dev_ssid[32];
    char dev_pswd[32];
    char dev_mac[13];
    char remote_mac[13];
    char dev_name[32];
} dev_config_t;

typedef struct
{
    char ProductKey[16];
    char DeviceName[13];
    char DeviceSecret[36];
} dev_mqtt_t;

typedef struct
{
    uint8_t *data;
    int32_t length;
    uint32_t maxlen;
} file_stream_t;


#define dev_host_web  "http://192.168.4.1"

#define NVS_SYS_CONFIG      "JOYSTICK"
#define NVS_SYS_WIFI_SSID   "wifissid"
#define NVS_SYS_WIFI_PSWD   "wifipswd"
#define NVS_SYS_DEV_NAME    "devname"

// soft ap
#define SOFT_AP_SSID        "JOYSTICK"
#define SOFT_AP_PSWD        "88888888"

extern const char root_start[]         asm("_binary_root_html_start");
extern const char root_end[]           asm("_binary_root_html_end");

#endif
