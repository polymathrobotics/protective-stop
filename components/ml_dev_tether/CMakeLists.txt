idf_component_register(
    SRCS "src/ml_dev_tether.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_netif esp_event lwip
)
