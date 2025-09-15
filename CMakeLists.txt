
set(SRCS
    "src/modbus_hub.cpp"
    "src/modbus_data.cpp"
    "src/modbus_node.cpp"
    "src/modbus_device.cpp"
    "src/modbus_task_pool.cpp"
    "src/modbus_rtu_iface.cpp"
    "src/modbus_utils.cpp"
)

set(INCLUDE_DIRS "src")

idf_component_register(SRCS "${SRCS}"
                    INCLUDE_DIRS "${INCLUDE_DIRS}"
                    REQUIRES ezmodbus esp_netif lwip) 