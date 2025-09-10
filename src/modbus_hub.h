// ModbusNode is a class that represents a Modbus node.
// The node is the modbus client that is connected to multiple servers via a unique interface, UART, or IP:PORT
#pragma once

#include "EZModbus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "modbus_device.h"
#include "modbus_node.h"
#include "mutex_utils.h"
#include <forward_list>
#include <memory>
#include "modbus_rtu_iface.h"


class ModbusHub
{
  public:
    ModbusHub();
    ~ModbusHub();

    ModbusDevice *add_tcp_device(const char *name, char *ip, uint16_t port, uint8_t address);
    ModbusDevice *add_rtu_device(const char *name, const char *iface, uint32_t baudrate, uart_word_length_t data_bits, uart_parity_t parity, uart_stop_bits_t stop_bits, uint8_t address);
    void remove_device(ModbusDevice *device);


    // Add physical RTU interface. In this case UART NUM 2 of ESP32-S3 device.
    void add_phy_rtu_iface(const char *iface, uart_port_t uart_num, uint8_t tx_pin, uint8_t rx_pin, uint8_t rts_pin, uint32_t baudrate, uart_word_length_t data_bits, uart_parity_t parity, uart_stop_bits_t stop_bits);

    void *operator new(size_t size);
    void operator delete(void *ptr) noexcept;
    void *operator new[](size_t size);
    void operator delete[](void *ptr) noexcept;

  private:
    // List of devices managed by the node.
    std::forward_list<ModbusNode *, ModbusAllocator<ModbusNode *>> nodes;
    
    // Mutex for thread safety
    mutable SemaphoreHandle_t _mutex;
    
    ModbusNode *create_node(const char *ip, uint16_t port);
    ModbusNode *create_node(const char *iface);
    ModbusNode *get_node(const char *ip, uint16_t port);
    ModbusNode *get_node(const char *iface);

    // Available PHY RTU, added at startup by the user.
    std::forward_list<ModbusRtuIface*, ModbusAllocator<ModbusRtuIface*>> phy_rtu_ifaces;
};

extern ModbusHub modbus_hub;