set(COMPONENT_SRCS main.c Task_PRO.c Task_APP.c UART_Interface.c Deserialize.c Protocols/setup.c)
set(COMPONENT_ADD_INCLUDEDIRS . Protocols) # /Protocols
set(COMPONENT_ADD_SRCDIRS Protocols)

register_component()
