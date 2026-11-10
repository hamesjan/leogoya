idf_component_register(
    SRC_DIRS "../src" "../src/app" "../src/vision" "../src/planning" "../src/motion" "../src/hal/esp32"
    INCLUDE_DIRS "../src"
    PRIV_REQUIRES esp_driver_gpio esp_driver_uart esp_timer esp_partition app_update
)
target_compile_options(${COMPONENT_LIB} PRIVATE -O2 -Wno-unused-parameter)
