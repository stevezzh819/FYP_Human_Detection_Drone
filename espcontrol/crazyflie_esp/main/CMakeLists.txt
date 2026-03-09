idf_component_register(SRCS "crazyflie_esp_main.c"
                            "amg8833.c"
                            "uart_cf_comm.c"
                       PRIV_REQUIRES driver esp_timer spi_flash
                       INCLUDE_DIRS "")
