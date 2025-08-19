idf_component_register(SRCS "modbus_hub.cpp" "modbus_data.cpp" "modbus_node.cpp" "modbus_device.cpp" "modbus_task_pool.cpp" "modbus_rtu_iface.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES psram  network ezmodbus) 