#include "modbus_data.h"
#include "esp_log.h"
#include "psram.h"
#include <cstdlib>
#include <cstring>

static const char TAG[] = "ModbusData";

// Factory method implementation
ModbusData *ModbusData::create(const char *mb_name, uint16_t mb_address, uint16_t mb_size, uint8_t mb_function_code,
                               ModbusPeriodicRead periodic, uint32_t polling_interval_ms)
{
    // Create the object
    ModbusData *data = new ModbusData(mb_name, mb_address, mb_size, mb_function_code, periodic, polling_interval_ms);
    if (!data)
    {
        ESP_LOGE(TAG, "%s: Failed to create ModbusData", mb_name);
        return nullptr; // Memory allocation failed
    }

    // Try to initialize
    if (!data->initialize())
    {
        ESP_LOGE(TAG, "%s: Failed to initialize ModbusData", mb_name);
        delete data;
        return nullptr; // Initialization failed
    }

    return data; // Success
}

// Private constructor - just sets values, no initialization
ModbusData::ModbusData(const char *mb_name, uint16_t mb_address, uint16_t mb_size, uint8_t mb_function_code,
                       ModbusPeriodicRead periodic, uint32_t polling_interval_ms)
    : mb_name_(), mb_address_(mb_address), mb_size_(mb_size), mb_function_code_(mb_function_code),
      registers_map_(nullptr), mutex_(nullptr), periodic_(periodic), polling_interval_ms_(polling_interval_ms),
      last_read_time_ms_(0)
{
    // Copy name (truncate if too long)
    strncpy(mb_name_, mb_name, sizeof(mb_name_) - 1);
    mb_name_[sizeof(mb_name_) - 1] = '\0'; // Ensure null termination
}

ModbusData::~ModbusData()
{
    ESP_LOGI(TAG, "%s: ModbusData destroyed", mb_name_);
    if (mutex_)
    {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }

    if (registers_map_)
    {
        free(registers_map_);
        registers_map_ = nullptr;
    }
}

bool ModbusData::initialize()
{
    // Create mutex
    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_)
    {
        return false; // Mutex creation failed
    }

    // Allocate registers map
    if (!allocate_registers_map())
    {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
        return false; // Allocation failed
    }

    return true; // Success
}

bool ModbusData::allocate_registers_map()
{
    if (mb_size_ == 0)
    {
        registers_map_ = nullptr;
        return true;
    }

    // Calculate size needed (2 bytes per register for uint16_t)
    size_t bytes_needed = mb_size_ * sizeof(uint16_t);

    // Try to allocate memory
    registers_map_ = static_cast<uint8_t *>(malloc(bytes_needed));
    if (!registers_map_)
    {
        return false; // Allocation failed
    }

    // Initialize to zero
    memset(registers_map_, 0, bytes_needed);
    return true;
}

bool ModbusData::is_valid() const
{
    return (mutex_ != nullptr && registers_map_ != nullptr);
}

bool ModbusData::lock(uint32_t timeout)
{
    if (mutex_ == nullptr)
    {
        return false;
    }
    TickType_t ticks = pdMS_TO_TICKS(timeout);
    return xSemaphoreTake(mutex_, ticks) == pdTRUE;
}

bool ModbusData::unlock()
{
    if (mutex_ == nullptr)
    {
        return false;
    }
    return xSemaphoreGive(mutex_) == pdTRUE;
}

// Thread-safe access methods
uint16_t ModbusData::get_address() const
{
    return mb_address_;
}

uint16_t ModbusData::get_size() const
{
    return mb_size_;
}

uint8_t ModbusData::get_function_code() const
{
    return mb_function_code_;
}

uint8_t *ModbusData::get_registers_map() const
{
    return registers_map_;
}

// Modbus::Frame ModbusData::get_frame() const
// {
//     // TODO: Implement when EzModbus is included
//     return frame_;
// }

ModbusPeriodicRead ModbusData::get_periodic() const
{
    return periodic_;
}

uint32_t ModbusData::get_polling_interval() const
{
    return polling_interval_ms_;
}

const char *ModbusData::get_name() const
{
    return mb_name_;
}

void *ModbusData::operator new(size_t size)
{
    return psram_malloc(size);
}

void ModbusData::operator delete(void *ptr) noexcept
{
    free(ptr);
}

void *ModbusData::operator new[](size_t size)
{
    return psram_malloc(size);
}

void ModbusData::operator delete[](void *ptr) noexcept
{
    free(ptr);
}

uint32_t ModbusData::get_last_read_time_ms() const
{
    return last_read_time_ms_;
}

void ModbusData::set_last_read_time_ms(uint32_t last_read_time_ms)
{
    last_read_time_ms_ = last_read_time_ms;
}