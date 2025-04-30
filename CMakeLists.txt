idf_component_register(SRCS "hello_world_main.c"
                    PRIV_REQUIRES spi_flash
                    REQUIRES driver nvs_flash esp_wifi esp_http_server
                    INCLUDE_DIRS "")
