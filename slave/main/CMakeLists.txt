idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES canopennodeesp32 efuse esp_timer driver nvs_flash log freertos
                    )