set(COMPONENT_SRCS main.c uart_rx_tasks.c i2c_test.c test.c)
set(COMPONENT_ADD_INCLUDEDIRS ".")
set(COMPONENT_PRIV_INCLUDEDIRS G:/sysgcc/esp32/esp-idf/v3.2/components G:/sysgcc/esp32/esp-idf/v3.2/components/esp32/include G:/sysgcc/esp32/esp-idf/v3.2/components/soc/esp32/include/soc)

register_component()
