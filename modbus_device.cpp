#include "modbus_device.h"
// #include "EzModbus.h" // Commented out until EzModbus is properly included
#include "esp_log.h"
#include "modbus_data.h"
#include "psram.h"
#include <cstdint>
#include <cstring>
#include <esp_timer.h>
#include <psram_allocator.h>

#define MODBUS_DEVICE_WAIT_MUTEX_LOCK_MS 1000
#define MODBUS_DEVICE_DEFAULT_POLLING_INTERVAL_MS 5000
#define MODBUS_DEVICE_DEFAULT_TIMEOUT_WAIT_TIME_MS 2 * MODBUS_DEVICE_DEFAULT_POLLING_INTERVAL_MS

const char TAG[] = "modbus_device";

// ============================================================================
// ModbusDevice Implementation
// ============================================================================

ModbusDevice::ModbusDevice(const char *name, uint8_t address) : modbus_address_(address), mutex_(nullptr)
{
    // Copy name safely
    strncpy(name_, name, sizeof(name_) - 1);
    name_[sizeof(name_) - 1] = '\0'; // Ensure null termination

    // Initialize mutex
    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_)
    {
        ESP_LOGE(TAG, "Failed to create mutex for ModbusDevice %s", name);
    }

    ESP_LOGI(TAG, "%s: ModbusDevice created, initial list size: %zu", name_, std::distance(data.begin(), data.end()));
}

ModbusDevice::~ModbusDevice()
{
    ESP_LOGI(TAG, "%s: ModbusDevice destroyed", name_);
    // Clean up mutex if it exists
    if (mutex_)
    {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }
}
bool ModbusDevice::lock(uint32_t timeout)
{
    if (mutex_ == nullptr)
    {
        return false;
    }

    TickType_t ticks = pdMS_TO_TICKS(timeout);
    return xSemaphoreTake(mutex_, ticks) == pdTRUE;
}

bool ModbusDevice::unlock()
{
    if (mutex_ == nullptr)
    {
        return false;
    }

    return xSemaphoreGive(mutex_) == pdTRUE;
}

ModbusData *ModbusDevice::add_read_request(const char *mb_name, uint16_t mb_address, uint16_t mb_size,
                                           uint8_t mb_function_code, ModbusPeriodicRead periodic,
                                           uint32_t polling_interval_ms)
{
    ESP_LOGI(TAG, "%s: Adding read request %s", name_, mb_name);
    if (!lock())
    {
        ESP_LOGE(TAG, "%s: Failed to lock mutex", name_);
        return nullptr;
    }

    ESP_LOGI(TAG, "%s: Before adding, list size: %zu", name_, std::distance(data.begin(), data.end()));

    ModbusData *tmp = ModbusData::create(mb_name, mb_address, mb_size, mb_function_code, periodic, polling_interval_ms);
    if (tmp)
    {
        data.push_front(tmp); // Store the pointer, not a copy
        ESP_LOGI(TAG, "%s: Read request %s added, new list size: %zu", name_, mb_name,
                 std::distance(data.begin(), data.end()));

        // Verify the list still has the element after push_front
        ESP_LOGI(TAG, "%s: After push_front, list size: %zu, first element: %p", name_,
                 std::distance(data.begin(), data.end()), data.empty() ? nullptr : *data.begin());

        // Check list size right before unlock
        ESP_LOGI(TAG, "%s: Right before unlock, list size: %zu", name_, std::distance(data.begin(), data.end()));

        unlock();
        return tmp;
    }
    else
    {
        ESP_LOGE(TAG, "%s: Failed to create ModbusData for %s", name_, mb_name);
    }

    unlock();
    return nullptr; // Return nullptr if creation failed
}

bool ModbusDevice::add_write_request(uint16_t mb_address, uint16_t mb_size, uint8_t mb_function_code,
                                     uint8_t *registers_map, ModbusPeriodicRead periodic)
{

    return false;
}

const std::forward_list<ModbusData *, PsramAllocator<ModbusData *>> &ModbusDevice::get_requests_list()
{
    ESP_LOGI(TAG, "%s: get_requests_list called, returning list with %zu elements", name_,
             std::distance(data.begin(), data.end()));
    return data; // Returns const reference, caller can't modify
}

const char *ModbusDevice::get_name()
{
    return name_;
}

uint8_t ModbusDevice::get_address() const
{
    return modbus_address_;
}

void *ModbusDevice::operator new(size_t size)
{
    return psram_malloc(size);
}

void ModbusDevice::operator delete(void *ptr) noexcept
{
    free(ptr);
}

void *ModbusDevice::operator new[](size_t size)
{
    return psram_malloc(size);
}

void ModbusDevice::operator delete[](void *ptr) noexcept
{
    free(ptr);
}