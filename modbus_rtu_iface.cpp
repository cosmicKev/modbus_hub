#include "modbus_rtu_iface.h"
#include "string.h"

ModbusRtuIface::ModbusRtuIface(char *iface, uart_port_t uart_num, uint8_t tx_pin, uint8_t rx_pin, uint8_t rts_pin, uint32_t baudrate, uart_word_length_t data_bits, uart_parity_t parity, uart_stop_bits_t stop_bits)
{
    strncpy(iface_, iface, sizeof(iface_));
    uart_num_ = uart_num;
    tx_pin_ = tx_pin;
    rx_pin_ = rx_pin;
    rts_pin_ = rts_pin;
    baudrate_ = baudrate;
    data_bits_ = data_bits;
    parity_ = parity;
    stop_bits_ = stop_bits;
}

ModbusHAL::UART::Config ModbusRtuIface::get_uart_cfg() const
{
    ModbusHAL::UART::Config uartCfg;
    uartCfg.uartNum = uart_num_;
    uartCfg.baud = baudrate_;
    uartCfg.config = data_bits_;
    uartCfg.rxPin = rx_pin_;
    uartCfg.txPin = tx_pin_;
    uartCfg.dePin = rts_pin_;
    return uartCfg;
};
