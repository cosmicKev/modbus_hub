#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <cstdint>
#include <cstring>

// Forward declaration
namespace Modbus
{
class Frame;
}

enum class ModbusPeriodicRead : uint8_t
{
    ON_REQUEST = 0x00,
    PERIODIC = 0x01,
};

class ModbusData
{
  public:
    // Factory method - returns nullptr on failure
    static ModbusData *create(const char *mb_name, uint16_t mb_address, uint16_t mb_size, uint8_t mb_function_code,
                              ModbusPeriodicRead periodic = ModbusPeriodicRead::ON_REQUEST,
                              uint32_t polling_interval_ms = 5000);

    // Destructor
    ~ModbusData();

    // // RAII Scoped Lock class
    // class ScopedLock
    // {
    //   private:
    //     ModbusData *data_;
    //     bool locked_;

    //   public:
    //     ScopedLock(ModbusData *data, uint32_t timeout = 1000) : data_(data), locked_(false)
    //     {
    //         locked_ = data_->lock(timeout);
    //     }

    //     ~ScopedLock()
    //     {
    //         if (locked_)
    //         {
    //             data_->unlock();
    //         }
    //     }

    //     bool is_locked() const
    //     {
    //         return locked_;
    //     }

    //     // Prevent copying
    //     ScopedLock(const ScopedLock &) = delete;
    //     ScopedLock &operator=(const ScopedLock &) = delete;

    //     // Allow moving
    //     ScopedLock(ScopedLock &&other) noexcept : data_(other.data_), locked_(other.locked_)
    //     {
    //         other.locked_ = false;
    //     }
    // };

    // Thread-safe access methods
    uint16_t get_address() const;
    uint16_t get_size() const;
    uint8_t get_function_code() const;
    uint8_t *get_registers_map() const;
    ModbusPeriodicRead get_periodic() const;
    uint32_t get_polling_interval() const;
    const char *get_name() const;
    // Check if object is valid and ready to use
    bool is_valid() const;

    // Legacy methods (kept for backward compatibility)
    bool lock(uint32_t timeout = 1000);
    bool unlock();

    void *operator new(size_t size);
    void operator delete(void *ptr) noexcept;
    void *operator new[](size_t size);
    void operator delete[](void *ptr) noexcept;

    uint32_t get_last_read_time_ms() const;
    void set_last_read_time_ms(uint32_t last_read_time_ms);

  private:
    // Private constructor - use create() instead
    ModbusData(const char *mb_name, uint16_t mb_address, uint16_t mb_size, uint8_t mb_function_code,
               ModbusPeriodicRead periodic, uint32_t polling_interval_ms);

    // Initialize method - called by factory
    bool initialize();

    // Helper method to allocate registers map
    bool allocate_registers_map();

    // Member variables
    char mb_name_[100];
    uint16_t mb_address_;
    uint16_t mb_size_;
    uint8_t mb_function_code_;
    uint8_t *registers_map_;
    // Modbus::Frame frame_; // TODO: Uncomment when EzModbus is included
    SemaphoreHandle_t mutex_;
    ModbusPeriodicRead periodic_;
    uint32_t polling_interval_ms_;
    uint32_t last_read_time_ms_;
};