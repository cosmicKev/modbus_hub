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
    strncpy(name_, name, strlen(name) + 1);

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
    // Clean up mutex if it exists
    if (mutex_)
    {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }

    // Clean up data list
    for (auto &data : data)
    {
        delete data;
    }
    data.clear();

    ESP_LOGI(TAG, "%s: ModbusDevice destroyed", name_);
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
    ESP_LOGI(TAG, "Adding read request %s", mb_name);
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

void ModbusDevice::remove_request(ModbusData *dev_data)
{
    if (!lock())
    {
        ESP_LOGE(TAG, "%s: Failed to lock mutex", name_);
        return;
    }

    ESP_LOGI(TAG, "%s: Removing request %s", name_, dev_data->get_name());
    if(dev_data && data.remove(dev_data) == 1)
    {
        ESP_LOGI(TAG, "%s: Removed request %s", name_, dev_data->get_name());
    }
    else
    {
        ESP_LOGE(TAG, "%s: Failed to remove request %s", name_, dev_data->get_name());
    }
    unlock();
}