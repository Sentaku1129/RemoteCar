set (srcs
    "main.c"
    "bsp_joystick.c"
    "bsp_display.c"
    "bsp_key_fsm.c"
    "bsp_joystick.c"
    "bsp_dns.c"
)

set (res
            res/root.html
)

idf_component_register(SRCS ${srcs}
                    PRIV_REQUIRES spi_flash driver esp_adc nvs_flash esp_wifi esp_http_server json
                    INCLUDE_DIRS "include"
                    EMBED_FILES ${res})
