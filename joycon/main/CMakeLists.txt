idf_component_register(SRCS "main.c" "bsp_display.c" "bsp_key_fsm.c"
                    PRIV_REQUIRES spi_flash driver
                    INCLUDE_DIRS "include")
