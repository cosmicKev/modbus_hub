#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "modbus_data.h"
#include "mutex_utils.h"
#include <forward_list>
#include "modbus_allocator.h"

// #include "EZModbus.h" // Commented out until EzModbus is properly included

#define MODBUS_RS485_LABEL_PARITY_DISABLE "disable"
#define MODBUS_RS485_LABEL_PARITY_EVEN "even"
#define MODBUS_RS485_LABEL_PARITY_ODD "odd"

// ModbusPeriodicRead enum is defined in modbus_data.h

constexpr size_t MODBUS_DEVICE_MAX_FRAMES = 20;

// Forward declarations
class ModbusData;

// Enums
enum class ModbusReadWrite : uint8_t
{
    READ = 0x00,
    WRITE = 0x01,
};

enum class ModbusDataRefresh : uint8_t
{
    PERIODIC = 0x00,   // Refreshes descriptor data periodically when the device has bus access
    ON_REQUEST = 0x01, // Refreshes descriptor only when asked
};

// enum class ModbusDataStatus : uint8_t
// {
//     NO_STATUS = 0x00,
//     BUSY_ON_REQUEST = 0x01,    // Refreshes descriptor data periodically when the device has bus access
//     SUCCESS_ON_REQUEST = 0x02, // Default state, gets erased when new request is made
//     INVALID_RESPONSE = 0x03,   // Invalid response from the device
//     WRITE_TIMEOUT = 0x04,      // Timeout on write operation
//     IDLE = 0x05,
// };

enum class ModbusStatus : uint8_t
{
    NOT_INITIALIZED,
    IDLE,
    UPDATED,
    TIMEOUT,
    ERROR,
};

enum class ModbusConnectionMode : uint8_t
{
    INVALID = 0x00,
    RTU = 0x01,
    TCP = 0x02,
};

enum class ModbusDeviceError : uint8_t
{
    NONE = 0,
    DUPLICATED_ADDRESS = 1,
};

// Main ModbusDevice class
class ModbusDevice
{
  public:
    // Constructor and destructor
    ModbusDevice(const char *name, uint8_t address);
    ~ModbusDevice();

    // RTU configuration
    void set_rtu_config(uint32_t baudrate, uart_word_length_t data_bits, uart_parity_t parity, uart_stop_bits_t stop_bits);

    // Thread safety - legacy methods for backward compatibility
    bool lock(uint32_t timeout = 1000);
    bool unlock();

    // Modbus data management
    ModbusData *add_read_request(const char *mb_name, uint16_t mb_address, uint16_t mb_size, uint8_t mb_function_code=Modbus::READ_HOLDING_REGISTERS,
                                 ModbusPeriodicRead periodic = ModbusPeriodicRead::ON_REQUEST,
                                 uint32_t polling_interval_ms = 5000);

    ModbusData *add_write_request(const char *mb_name, uint16_t mb_address, uint16_t mb_size, uint8_t mb_function_code);

    const std::forward_list<ModbusData *, ModbusAllocator<ModbusData *>> &get_requests_list();
    const char *get_name();
    uint8_t get_address() const;
    uint32_t get_baudrate() const { return baudrate_; }
    uart_parity_t get_parity() const { return parity_; }
    void set_parity(uart_parity_t parity) { parity_ = parity; }
    void remove_request(ModbusData *dev_data);

    void *operator new(size_t size);
    void operator delete(void *ptr) noexcept;
    void *operator new[](size_t size);
    void operator delete[](void *ptr) noexcept;

    uint64_t get_timestamp() const { return epoch; }
    void set_timestamp(uint64_t timestamp) { epoch = timestamp; }

    void set_wait_after_query(uint32_t wait_after_query_ms);
    uint32_t get_wait_after_query() const { return wait_after_query_ms_; }

    void sync_periodic_data();

    void set_error(ModbusDeviceError error);
    ModbusDeviceError get_error() const { return error_; }

  private:
    // Member variables
    char name_[100];
    uint8_t modbus_address_;
    uint64_t epoch;

    // Mutex for thread safety
    SemaphoreHandle_t mutex_;

    // We dont know how many frames we will have.
    std::forward_list<ModbusData *, ModbusAllocator<ModbusData *>> data;
    // RTU configuration
    uint32_t baudrate_;
    uart_word_length_t data_bits_;
    uart_parity_t parity_;
    uart_stop_bits_t stop_bits_;


    ModbusDeviceError error_;
    // Wait after query
    uint32_t wait_after_query_ms_ = 0;
};
