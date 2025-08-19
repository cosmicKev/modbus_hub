#include "modbus_hub.h"
// #include "modbus_device.h"
#include "config.h"
#include "hal/uart_types.h"
#include "modbus_utils.h"
#include "psram.h"
#include <cstdint>
#include "string.h"

constexpr char TAG[] = "ModbusHub";

ModbusHub::ModbusHub()
{
    ESP_LOGI(TAG, "Creating ModbusHub");
    // Initialize mutex for thread safety
    _mutex = xSemaphoreCreateMutex();
    if (!_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex for ModbusHub - this is critical!");
        // In a real system, you might want to abort or handle this differently
        // For now, we'll continue but all mutex operations will fail
    } else {
        ESP_LOGI(TAG, "ModbusHub mutex created successfully");
    }
}

ModbusHub::~ModbusHub()
{
    ESP_LOGI(TAG, "Destroying ModbusHub");
    
    // Clean up all nodes
    ScopedMutex lock(_mutex);
    if (lock.isLocked()) {
        for (auto node : nodes) {
            if (node) {
                delete node;
            }
        }
        nodes.clear();
    }
    
    if (_mutex) {
        vSemaphoreDelete(_mutex);
    }
}

ModbusNode *ModbusHub::create_node(const char *ip, uint16_t port)
{

    ModbusNode *tmp = ModbusNode::create(ip, port);
    if (tmp)
    {
        nodes.push_front(tmp); // Store the pointer, not a copy
        // Gets the pointer we just added.
        ModbusNode *node = nodes.front();
        ESP_LOGI(TAG, "Node created for TCP device %s:%u with Node %p", ip, port, node);
        return node;
    }
    ESP_LOGE(TAG, "Failed to create node for TCP device %s", ip);
    return nullptr;
}

ModbusNode *ModbusHub::create_node(const char *iface)
{
    // Get any configuration
    ModbusHAL::UART::Config uartCfg;
    bool found = false;
    for(auto phy_iface : phy_rtu_ifaces)
    {
        if(strncmp(phy_iface->get_iface(), iface, sizeof(phy_iface->get_iface())) == 0)
        {
            ESP_LOGI(TAG, "Found PHY RTU interface for %s", phy_iface->get_iface());
            uartCfg = {
                .uartNum = phy_iface->get_uart_num(),
                .baud = phy_iface->get_baudrate(),
                .config = phy_iface->get_data_bits(),
                .rxPin = phy_iface->get_rx_pin(),
                .txPin = phy_iface->get_tx_pin(),
                .dePin = phy_iface->get_rts_pin(),
            };
            found = true;
        }
    }

    if(!found)
    {
        ESP_LOGE(TAG, "No PHY RTU interface found for %s", iface);
        return nullptr;
    }

    ModbusNode *tmp = ModbusNode::create(iface, uartCfg);
    if (tmp)
    {
        nodes.push_front(tmp); // Store the pointer, not a copy
        ModbusNode *node = nodes.front();
        ESP_LOGI(TAG, "Node created for RTU device %s with Node %p", iface, node);
        return node;
    }
    return nullptr;
}

ModbusNode *ModbusHub::get_node(const char *ip, uint16_t port)
{

    char iface[100];
    snprintf(iface, sizeof(iface), "%s:%u", ip, port);
    for (auto &node : nodes)
    {
        if (node && node->interface_type == ModbusNodeInterfaceType::TCP && strncmp(iface, node->get_iface(), sizeof(iface)) == 0)
        {
            ESP_LOGI(TAG, "Found node for %s", iface);
            return node;
        }
    }
    return nullptr;
}

ModbusNode *ModbusHub::get_node(const char *iface)
{
    for (auto &node : nodes)
    {
        if (node && node->interface_type == ModbusNodeInterfaceType::RTU && strncmp(iface, node->get_iface(), strlen(iface)) == 0)
        {
            ESP_LOGI(TAG, "Found node for %s", iface);
            return node;
        }
    }
    return nullptr;
}

ModbusDevice *ModbusHub::add_tcp_device(const char *name, char *ip, uint16_t port, uint8_t address)
{
    if (!_mutex) {
        ESP_LOGE(TAG, "Cannot add TCP device - mutex not initialized");
        return nullptr;
    }
    
    ScopedMutex lock(_mutex);
    if (!lock.isLocked()) {
        ESP_LOGE(TAG, "Failed to acquire mutex for TCP device addition");
        return nullptr;
    }
    
    // Check if node exists, if not create it
    ModbusNode *node = get_node(ip, port);
    if (node == nullptr)
    {
        // Create node while holding the mutex
        node = create_node(ip, port);
        if (!node) {
            ESP_LOGE(TAG, "Failed to create node for TCP device %s", name);
            return nullptr;
        }
        // Created, then start it.
        node->start();
    }
    ModbusDevice *device = new ModbusDevice(name, address);
    node->add_device(device);
    return device;
}

ModbusDevice *ModbusHub::add_rtu_device(const char *name, char *iface, uint32_t baudrate, uart_word_length_t data_bits, uart_parity_t parity, uart_stop_bits_t stop_bits, uint8_t address)
{
    ScopedMutex lock(_mutex);
    if (!lock.isLocked()) {
        ESP_LOGE(TAG, "Failed to acquire mutex for RTU device addition");
        return nullptr;
    }
    
    ModbusNode *node = get_node(iface);
    if (node == nullptr)
    {
        // Create node while holding the mutex
        node = create_node(iface);
        if(!node) 
        {
            ESP_LOGE(TAG, "Failed to create node for RTU device %s", name);
            return nullptr;
        }
        // Created, then start it.
        node->start();
    }
    
    ModbusDevice *device = new ModbusDevice(name, address);
    device->set_rtu_config(baudrate, data_bits, parity, stop_bits);
    node->add_device(device);
    return device;
}

void ModbusHub::remove_device(ModbusDevice *device)
{
    if (!device) {
        return;
    }
    
    ScopedMutex lock(_mutex);
    if (!lock.isLocked()) {
        ESP_LOGE(TAG, "Failed to acquire mutex for device removal");
        return;
    }
    
    // Find the node that contains this device and remove it
    for (auto &node : nodes)
    {
        if (node && node->has_device(device))
        {
            node->remove_device(device);
            break;
        }
    }
}

void *ModbusHub::operator new(size_t size)
{
    return psram_malloc(size);
}

void ModbusHub::operator delete(void *ptr) noexcept
{
    free(ptr);
}

void *ModbusHub::operator new[](size_t size)
{
    return psram_malloc(size);
}

void ModbusHub::operator delete[](void *ptr) noexcept
{
    free(ptr);
}

void ModbusHub::add_phy_rtu_iface(char *iface, uart_port_t uart_num, uint8_t tx_pin, uint8_t rx_pin, uint8_t rts_pin, uint32_t baudrate, uart_word_length_t data_bits, uart_parity_t parity, uart_stop_bits_t stop_bits)
{
    ModbusRtuIface *tmp = new ModbusRtuIface(iface, uart_num, tx_pin, rx_pin, rts_pin, baudrate, data_bits, parity, stop_bits);
    if(tmp)
    {
        phy_rtu_ifaces.push_front(tmp);
        ESP_LOGI(TAG, "PHY RTU interface for with name %s added. Tx_pin: %d, Rx_pin: %d, RTS_pin: %d, Baudrate: %u, Data_bits: %u, Parity: %u, Stop_bits: %u", iface, (int)tx_pin, (int)rx_pin, (int)rts_pin, (int)baudrate, (int)data_bits, (int)parity, (int)stop_bits);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to create PHY RTU interface for %s", iface);
    }
}

ModbusHub modbus_hub;