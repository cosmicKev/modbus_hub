#pragma once

#include "hal/uart_types.h"
#include "modbus_node.h"

class ModbusRtuIface
{
    public:
    ModbusRtuIface(char *iface, uart_port_t uart_num, uint8_t tx_pin, uint8_t rx_pin, uint8_t rts_pin, uint32_t baudrate, uart_word_length_t data_bits, uart_parity_t parity, uart_stop_bits_t stop_bits);
    ~ModbusRtuIface();

    uart_port_t get_uart_num() const { return uart_num_; }
    uint8_t get_tx_pin() const { return tx_pin_; }
    uint8_t get_rx_pin() const { return rx_pin_; }
    uint8_t get_rts_pin() const { return rts_pin_; }
    uint32_t get_baudrate() const { return baudrate_; }
    uart_word_length_t get_data_bits() const { return data_bits_; }
    uart_parity_t get_parity() const { return parity_; }
    uart_stop_bits_t get_stop_bits() const { return stop_bits_; }
    const char *get_iface() const { return iface_; }

    ModbusHAL::UART::Config get_uart_cfg() const;

    private:
    char iface_[100];
    uart_port_t uart_num_;
    uint8_t tx_pin_;
    uint8_t rx_pin_;
    uint8_t rts_pin_;
    uint32_t baudrate_;
    uart_word_length_t data_bits_;
    uart_parity_t parity_;
    uart_stop_bits_t stop_bits_;
};  