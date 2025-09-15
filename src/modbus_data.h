#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <cstdint>
#include <cstring>
#include "EZModbus.h"
#include "esp_log.h"


enum class ModbusDataStatus
{
    IDLE,
    REQUESTED,
    TIMEDOUT,
    ERROR,
    SUCCESS,
};

// short version from esp-modbus
enum class ModbusBytesOrder: uint8_t
{
  SIZE_16 = 0x20,          /*!< 16 bit size */
  AB = 0x21,               /*!< big endian */
  BA = 0x22,               /*!<little endian */
  SIZE_32 = 0x40,          /*!< 32 bit size */
  ABCD = 0x41,             /*!< big endian */
  CDAB = 0x42,             /*!< big endian, reversed register order */
  BADC = 0x43,             /*!< little endian, reversed register order */
  DCBA = 0x44,             /*!< little endian */
  SIZE_64 = 0x80,          /*!< 64 bit size */
  ABCDEFGH = 0x81,         /*!< ABCDEFGH, big endian */
  HGFEDCBA = 0x82,         /*!< HGFEDCBA, little endian */
  GHEFCDAB = 0x83,         /*!< GHEFCDAB, big endian, reversed register order */
  BADCFEHG = 0x84,         /*!< BADCFEHG, little endian, reversed register order */
  EFGHABCD = 0x85,         /*!< EFGHABCD, big endian, reversed register order */
  CDABEFGH = 0x86,         /*!< CDABEFGH, little endian, reversed register order */
};

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

    // RAII Scoped Lock class
    class ScopedLock
    {
      private:
        ModbusData *data_;
        bool locked_;

      public:
        ScopedLock(ModbusData *data, uint32_t timeout = 1000) : data_(data), locked_(false)
        {
            locked_ = data_->lock(timeout);
        }

        ~ScopedLock()
        {
            if (locked_)
            {
                data_->unlock();
            }
        }

        bool is_locked() const
        {
            return locked_;
        }

        // Prevent copying
        ScopedLock(const ScopedLock &) = delete;
        ScopedLock &operator=(const ScopedLock &) = delete;

        // Allow moving
        ScopedLock(ScopedLock &&other) noexcept : data_(other.data_), locked_(other.locked_)
        {
            other.locked_ = false;
        }
    };

    // Thread-safe access methods
    uint16_t get_address() const;
    uint16_t get_size() const;
    uint8_t get_function_code() const;
    uint8_t *get_registers_map() const;
    ModbusPeriodicRead get_periodic() const;
    void set_request_type(ModbusPeriodicRead periodic);
    uint32_t get_polling_interval() const;
    const char *get_name() const;
    void clear_data();
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

    bool is_idle() const { return status_ == ModbusDataStatus::IDLE; }
    bool error() const { return status_ == ModbusDataStatus::ERROR; }
    bool timeout() const { return status_ == ModbusDataStatus::TIMEDOUT; }
    bool success() const { return status_ == ModbusDataStatus::SUCCESS; }

    uint8_t *data() const { return registers_map_; }
    void clear_status() { status_ = ModbusDataStatus::IDLE; }
    void request_data();
    void set_status(ModbusDataStatus status) { status_ = status; }

    void update_data(const std::array<uint16_t, Modbus::FRAME_DATASIZE> &data);

    void get_data(void *data_out, size_t size, ModbusBytesOrder bytes_order);
    static void swap_bytes(void *data_in, void *data_out, size_t size, ModbusBytesOrder bytes_order);
    
    static uint32_t get_uint32(uint32_t data_in, ModbusBytesOrder bytes_order);
    static int32_t get_int32(int32_t data_in, ModbusBytesOrder bytes_order);
    static uint16_t get_uint16(uint16_t data_in, ModbusBytesOrder bytes_order);
    static int16_t get_int16(int16_t data_in, ModbusBytesOrder bytes_order);
    static uint64_t get_uint64(uint64_t data_in, ModbusBytesOrder bytes_order);
    static double get_double(double data_in, ModbusBytesOrder bytes_order);
    static float get_float(float data_in, ModbusBytesOrder bytes_order);
    static void get_string(char *data_in, size_t size, ModbusBytesOrder bytes_order);

    void set_address(uint16_t address) { mb_address_ = address; }
    void set_size(uint16_t size) { mb_size_ = size; }
    ModbusDataStatus get_status() const { return status_; }

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
    SemaphoreHandle_t mutex_;
    ModbusPeriodicRead periodic_;
    uint32_t polling_interval_ms_;
    uint32_t last_read_time_ms_;
    Modbus::ExceptionCode exception_code_;
    ModbusDataStatus status_;

};