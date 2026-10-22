idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_driver_gpio esp_driver_rmt
                                  esp_event esp_netif esp_wifi esp_http_server
                                  nvs_flash app_update)
