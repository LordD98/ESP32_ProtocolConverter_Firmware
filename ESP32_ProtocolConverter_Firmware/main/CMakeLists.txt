set(COMPONENT_SRCS main.h main.c Task_PRO.c Task_APP.c UART_Interface.c Deserialize.h Deserialize.c i2c_test.c)
set(COMPONENT_ADD_INCLUDEDIRS ".")
set(COMPONENT_PRIV_INCLUDEDIRS G:/sysgcc/esp32/esp-idf/v3.2/components G:/sysgcc/esp32/esp-idf/v3.2/components/soc/esp32/include/soc)

register_component()
