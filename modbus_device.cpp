#include "modbus_device.h"
// #include "EzModbus.h" // Commented out until EzModbus is properly included
#include "esp_log.h"
#include "modbus_data.h"
#include "mutex_utils.h"
#include "psram.h"
#include <cstdint>
#include <cstring>
#include <esp_timer.h>
#include <psram_allocator.h>
#include "time_utils.h"

#define MODBUS_DEVICE_WAIT_MUTEX_LOCK_MS 1000
#define MODBUS_DEVICE_DEFAULT_POLLING_INTERVAL_MS 5000
#define MODBUS_DEVICE_DEFAULT_WAIT_AFTER_QUERY_MS 50
#define MODBUS_DEVICE_DEFAULT_TIMEOUT_WAIT_TIME_MS 2 * MODBUS_DEVICE_DEFAULT_POLLING_INTERVAL_MS
#define MODBUS_DEVICE_MAX_DELAY_MS 30000
const char TAG[] = "modbus_device";

// ============================================================================
// ModbusDevice Implementation
// ============================================================================

ModbusDevice::ModbusDevice(const char *name, uint8_t address) : modbus_address_(address), mutex_(nullptr)
{
    // Copy name safely
    strncpy(name_, name, strlen(name) + 1);
    epoch_ms = 0;
    wait_after_query_ms_ = MODBUS_DEVICE_DEFAULT_WAIT_AFTER_QUERY_MS;
    baudrate_ = 0;
    data_bits_ = UART_DATA_8_BITS;
    parity_ = UART_PARITY_DISABLE;
    stop_bits_ = UART_STOP_BITS_1;

    // Initialize mutex
    mutex_ = xSemaphoreCreateRecursiveMutex();
    if (!mutex_)
    {
        ESP_LOGE(TAG, "Failed to create mutex for ModbusDevice %s", name);
    }
}

ModbusDevice::~ModbusDevice()
{

    if (!lock(portMAX_DELAY))
    {
        ESP_LOGE(TAG, "%s: Failed to lock mutex for ModbusDevice destruction", name_);
        return;
    }
    // Clean up data list
    for (auto &request : data)
    { 
        delete request;
    }

    data.clear();
    // Clean up mutex if it exists
    if (mutex_)
    {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }



    ESP_LOGI(TAG, "%s: ModbusDevice destroyed", name_);
}

bool ModbusDevice::lock(uint32_t timeout)
{
    if (mutex_ == nullptr)
    {
        return false;
    }

    TickType_t ticks = pdMS_TO_TICKS(timeout);
    return xSemaphoreTakeRecursive(mutex_, ticks) == pdTRUE;
}

bool ModbusDevice::unlock()
{
    if (mutex_ == nullptr)
    {
        return false;
    }

    return xSemaphoreGiveRecursive(mutex_) == pdTRUE;
}

ModbusData *ModbusDevice::add_read_request(const char *mb_name, uint16_t mb_address, uint16_t mb_size,
                                           uint8_t mb_function_code, ModbusPeriodicRead periodic,
                                           uint32_t polling_interval_ms)
{   
    ModbusData *tmp = nullptr;
    if(!lock(MODBUS_DEVICE_MAX_DELAY_MS))
    {
        ESP_LOGE(TAG, "%s: Failed to lock mutex", name_);
        return nullptr;
    }

    tmp = ModbusData::create(mb_name, mb_address, mb_size, mb_function_code, periodic, polling_interval_ms);
    if (tmp)
    {
        data.push_front(tmp); // Store the pointer, not a copy
        ESP_LOGI(TAG, "%s: Successfully added read request %s: Addr: %d(%x) reg_len: %d", name_, mb_name, mb_address, mb_address, mb_size);
    }
    else
    {
        ESP_LOGE(TAG, "%s: Failed to create ModbusData for %s: Addr: %d(%x) reg_len: %d", name_, mb_name, mb_address, mb_address, mb_size);
    }
    unlock();
    return tmp; // Return nullptr if creation failed
}

ModbusData *ModbusDevice::add_write_request(const char *mb_name, uint16_t mb_address, uint16_t mb_size, uint8_t mb_function_code)
{
    ModbusData *tmp = nullptr;
    if(!lock(MODBUS_DEVICE_MAX_DELAY_MS))
    {
        ESP_LOGE(TAG, "%s: Failed to lock mutex", name_);
        return nullptr;
    }
    tmp = ModbusData::create(mb_name, mb_address, mb_size, mb_function_code, ModbusPeriodicRead::ON_REQUEST, 0);
    if (tmp)
    {
        data.push_front(tmp); // Store the pointer, not a copy
        ESP_LOGI(TAG, "%s: Successfully added write request %s", name_, mb_name);
    }
    else
    {
        ESP_LOGE(TAG, "%s: Failed to create ModbusData for %s", name_, mb_name);
    }
    unlock();
    return tmp;
}

const std::forward_list<ModbusData *, PsramAllocator<ModbusData *>> &ModbusDevice::get_requests_list()
{
    return data;
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
    if (!dev_data) {
        ESP_LOGE(TAG, "%s: Cannot remove null request", name_);
        return;
    }
    if(!lock(MODBUS_DEVICE_MAX_DELAY_MS))
    {
        ESP_LOGE(TAG, "%s: Failed to lock mutex for request removal", name_);
        return;
    }

    ESP_LOGI(TAG, "%s: Removing request %s", name_, dev_data->get_name());
    if(data.remove(dev_data) == 1)
    {
        delete dev_data; // Clean up the removed request
    }
    else
    {
        ESP_LOGW(TAG, "%s: Failed to remove request %s (not found in list)", name_, dev_data->get_name());
    }
    unlock();
}


/*
 * @brief Set the RTU configuration for the device.
 * @param baudrate The baudrate to use
 * @param data_bits The data bits to use
 * @param parity The parity to use
 * @param stop_bits The stop bits to use
    */
void ModbusDevice::set_rtu_config(uint32_t baudrate, uart_word_length_t data_bits, uart_parity_t parity, uart_stop_bits_t stop_bits)
{
    baudrate_ = baudrate;
    data_bits_ = data_bits;
    parity_ = parity;
    stop_bits_ = stop_bits;
}

void ModbusDevice::set_wait_after_query(uint32_t wait_after_query_ms)
{
    ScopedMutex lock(mutex_, portMAX_DELAY);
    if (!lock.isLocked())
    {
        ESP_LOGE(TAG, "%s: Failed to lock mutex for wait after query. %s", name_, __func__);
        return;
    }
    wait_after_query_ms_ = wait_after_query_ms;
}

void ModbusDevice::sync_periodic_data()
{
    if(!lock(MODBUS_DEVICE_MAX_DELAY_MS))
    {
        ESP_LOGE(TAG, "%s: Failed to lock mutex for sync periodic data", name_);
        return;
    }
    for(auto &request : data)
    {
        if(request->get_periodic() == ModbusPeriodicRead::PERIODIC)
        {
            // Resets last read time.
            request->set_last_read_time_ms(0);
        }
    }
    unlock();
}