#include "modbus_data.h"
#include "esp_log.h"
#include "psram.h"
#include <cstdint>
#include <cstdlib>
#include <cstring>

static constexpr const char *TAG = "ModbusData";

// Factory method implementation
ModbusData *ModbusData::create(const char *mb_name, uint16_t mb_address, uint16_t mb_size, uint8_t mb_function_code,
                               ModbusPeriodicRead periodic, uint32_t polling_interval_ms)
{
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


    // We dont request data imediatly for write requests
    if (periodic == ModbusPeriodicRead::ON_REQUEST && !(mb_function_code == Modbus::WRITE_REGISTER || mb_function_code == Modbus::WRITE_MULTIPLE_REGISTERS || mb_function_code == Modbus::WRITE_COIL || mb_function_code == Modbus::WRITE_MULTIPLE_COILS))
    {
        data->request_data();
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
    if (!lock(portMAX_DELAY))
    {
        ESP_LOGE(TAG, "%s: Failed to lock mutex for ModbusData destruction", mb_name_);
        return;
    }

    // Clean up registers map
    if (registers_map_)
    {
        free(registers_map_);
        registers_map_ = nullptr;
    }

    if (mutex_)
    {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
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

void ModbusData::set_periodic(ModbusPeriodicRead periodic)
{
    periodic_ = periodic;
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

// Non Thread-safe
void ModbusData::update_data(const std::array<uint16_t, Modbus::FRAME_DATASIZE> &data)
{
    memcpy(registers_map_, data.data(), mb_size_ * sizeof(uint16_t));
}

void ModbusData::swap_bytes(void *data_in, void *data_out, size_t size, ModbusBytesOrder bytes_order)
{
    // Tread all data as unsigned
    uint8_t *pdata_in = static_cast<uint8_t *>(data_in);
    uint8_t *pdata_out = static_cast<uint8_t *>(data_out);

    if (data_in == nullptr || data_out == nullptr || size == 0)
    {
        ESP_LOGE(TAG, "Invalid data");
        return;
    }

    // Double checking sizes from given on array and choosen byte order. 
    if(size != (uint8_t)((uint8_t)bytes_order >> 4 & 0x0F))
    {
        ESP_LOGE(TAG, "Invalid size. %u != %u", (uint8_t)size, (uint8_t)((uint8_t)bytes_order >> 4 & 0x0F));
        return;
    }

    switch (bytes_order)
    {
    case ModbusBytesOrder::AB:
        memcpy(data_out, data_in, size);
        break;
    case ModbusBytesOrder::BA:
        pdata_out[0] = pdata_in[1];
        pdata_out[1] = pdata_in[0];
        break;
    case ModbusBytesOrder::ABCD:
        memcpy(data_out, data_in, size);
        break;
    case ModbusBytesOrder::CDAB:
        pdata_out[0] = pdata_in[2];
        pdata_out[1] = pdata_in[3];
        pdata_out[2] = pdata_in[0];
        pdata_out[3] = pdata_in[1];
        break;
    case ModbusBytesOrder::BADC:
        pdata_out[0] = pdata_in[3];
        pdata_out[1] = pdata_in[2];
        pdata_out[2] = pdata_in[1];
        pdata_out[3] = pdata_in[0];
        break;
    case ModbusBytesOrder::DCBA:
        pdata_out[0] = pdata_in[3];
        pdata_out[1] = pdata_in[2];
        pdata_out[2] = pdata_in[1];
        pdata_out[3] = pdata_in[0];
        break;
    case ModbusBytesOrder::ABCDEFGH:
        // No swap - direct copy
        memcpy(data_out, data_in, size);
        break;
    case ModbusBytesOrder::HGFEDCBA:
        // Complete reverse order
        pdata_out[0] = pdata_in[7];
        pdata_out[1] = pdata_in[6];
        pdata_out[2] = pdata_in[5];
        pdata_out[3] = pdata_in[4];
        pdata_out[4] = pdata_in[3];
        pdata_out[5] = pdata_in[2];
        pdata_out[6] = pdata_in[1];
        pdata_out[7] = pdata_in[0];
        break;
    case ModbusBytesOrder::GHEFCDAB:
        // Swap 4-byte groups: GHEF + CDAB
        pdata_out[0] = pdata_in[6]; // G
        pdata_out[1] = pdata_in[7]; // H
        pdata_out[2] = pdata_in[4]; // E
        pdata_out[3] = pdata_in[5]; // F
        pdata_out[4] = pdata_in[2]; // C
        pdata_out[5] = pdata_in[3]; // D
        pdata_out[6] = pdata_in[0]; // A
        pdata_out[7] = pdata_in[1]; // B
        break;
    case ModbusBytesOrder::BADCFEHG:
        // Swap 4-byte groups: BADC + FEHG
        pdata_out[0] = pdata_in[1]; // B
        pdata_out[1] = pdata_in[0]; // A
        pdata_out[2] = pdata_in[3]; // D
        pdata_out[3] = pdata_in[2]; // C
        pdata_out[4] = pdata_in[5]; // F
        pdata_out[5] = pdata_in[4]; // E
        pdata_out[6] = pdata_in[7]; // H
        pdata_out[7] = pdata_in[6]; // G
        break;
    case ModbusBytesOrder::EFGHABCD:
        // Swap 4-byte groups: EFGH + ABCD
        pdata_out[0] = pdata_in[4]; // E
        pdata_out[1] = pdata_in[5]; // F
        pdata_out[2] = pdata_in[6]; // G
        pdata_out[3] = pdata_in[7]; // H
        pdata_out[4] = pdata_in[0]; // A
        pdata_out[5] = pdata_in[1]; // B
        pdata_out[6] = pdata_in[2]; // C
        pdata_out[7] = pdata_in[3]; // D
        break;
    case ModbusBytesOrder::CDABEFGH:
        // Swap 4-byte groups: CDAB + EFGH
        pdata_out[0] = pdata_in[2]; // C
        pdata_out[1] = pdata_in[3]; // D
        pdata_out[2] = pdata_in[0]; // A
        pdata_out[3] = pdata_in[1]; // B
        pdata_out[4] = pdata_in[4]; // E
        pdata_out[5] = pdata_in[5]; // F
        pdata_out[6] = pdata_in[6]; // G
        pdata_out[7] = pdata_in[7]; // H
        break;
    default:
        ESP_LOGE(TAG, "Invalid bytes order");
        break;
    }
    return;
}

void ModbusData::get_string(char *data_in, size_t size, ModbusBytesOrder bytes_order)
{
    if(size == 0)
    {
        return;
    }
    if(bytes_order == ModbusBytesOrder::AB)
    {
        return;
    }
    else if(bytes_order == ModbusBytesOrder::BA)
    {
        char tmp;
        for(int i = 1; i < size; i+=2)
        {
            tmp = data_in[i];
            data_in[i] = data_in[i-1];
            data_in[i-1] = tmp;
        }
        return;
    }
    else {
        ESP_LOGE(TAG, "Invalid bytes order for string. Choose AB or BA");
    }
}

uint32_t ModbusData::get_uint32(uint32_t data_in, ModbusBytesOrder bytes_order)
{
    uint32_t value = 0;
    swap_bytes((void*)&data_in, &value, 4, bytes_order);
    return value;
}

int32_t ModbusData::get_int32(int32_t data_in, ModbusBytesOrder bytes_order)
{
    int32_t value = 0;
    swap_bytes((void*)&data_in, &value, 4, bytes_order);
    return value;
}

uint16_t ModbusData::get_uint16(uint16_t data_in, ModbusBytesOrder bytes_order)
{
    uint16_t value = 0;
    swap_bytes((void*)&data_in, &value, 2, bytes_order);
    return value;
}

int16_t ModbusData::get_int16(int16_t data_in, ModbusBytesOrder bytes_order)
{
    int16_t value = 0;
    swap_bytes((void*)&data_in, &value, 2, bytes_order);
    return value;
}


uint64_t ModbusData::get_uint64(uint64_t data_in, ModbusBytesOrder bytes_order)
{
    uint64_t value = 0;
    swap_bytes((void*)&data_in, &value, 8, bytes_order);
    return value;
}

float ModbusData::get_float(float data_in, ModbusBytesOrder bytes_order)
{
    float value = 0;
    swap_bytes((void*)&data_in, &value, 4, bytes_order);
    return value;
}

double ModbusData::get_double(double data_in, ModbusBytesOrder bytes_order){
    double value = 0;
    swap_bytes((void*)&data_in, &value, 8, bytes_order);
    return value;
}

void ModbusData::clear_data()
{
    if (lock())
    {
        memset(registers_map_, 0, mb_size_ * sizeof(uint16_t));
        unlock();
    }
}

void ModbusData::request_data()
{
    status_ = ModbusDataStatus::REQUESTED;
    last_read_time_ms_ = 0;
}