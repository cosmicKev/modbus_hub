#pragma once

#include "modbus_device.h"
#include "modbus_node.h"
#include "psram.h"
#include <cstdint>
#include <forward_list>

class ModbusHub
{
  public:
    ModbusHub();
    ~ModbusHub();

    ModbusDevice *add_rtu_device(const char *name, char *iface, uint32_t baudrate, char *parity, uint8_t address);
    ModbusDevice *add_tcp_device(const char *name, char *ip, uint16_t port, uint8_t address);
    void remove_device(ModbusDevice *device);

    void *operator new(size_t size);
    void operator delete(void *ptr) noexcept;
    void *operator new[](size_t size);
    void operator delete[](void *ptr) noexcept;

  private:
    std::forward_list<ModbusNode *, PsramAllocator<ModbusNode *>> nodes;

    // void create_tcp_device(const char *ip, uint16_t port, uint8_t slave_addr);
    // void create_rs485_device(const char *iface, uint32_t baudrate, uart_parity_t parity);

    // ModbusNode *get_node(const char *iface);
    ModbusNode *create_node(const char *ip, uint16_t port);
    ModbusNode *create_node(const char *iface);
    ModbusNode *get_node(const char *ip, uint16_t port);
    ModbusNode *get_node(const char *iface);
};