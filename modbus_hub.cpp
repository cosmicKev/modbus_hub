#include "modbus_hub.h"
// #include "modbus_device.h"
#include "config.h"
#include "modbus_utils.h"
#include "psram.h"

constexpr char TAG[] = "ModbusHub";

ModbusHub::ModbusHub()
{
}

ModbusHub::~ModbusHub()
{
}

ModbusNode *ModbusHub::create_node(const char *ip, uint16_t port)
{
    ModbusNode *tmp = ModbusNode::create(ip, port);
    if (tmp)
    {
        nodes.push_front(tmp); // Store the pointer, not a copy
        // Gets the pointer we just added.
        ModbusNode *node = nodes.front();
        // Initializes EzModbus
        node->initialize_communication();
        return node;
    }
    return nullptr;
}

ModbusNode *ModbusHub::create_node(const char *iface)
{
    ModbusNode *tmp = ModbusNode::create(iface);
    if (tmp)
    {
        nodes.push_front(tmp); // Store the pointer, not a copy
        ModbusNode *node = nodes.front();
        node->initialize_communication();
        return node;
    }
    return nullptr;
}

ModbusDevice *ModbusHub::add_tcp_device(const char *name, char *ip, uint16_t port, uint8_t address)
{
    // Check if node exists, if not create it
    ModbusNode *node = get_node(ip, port);
    if (node == nullptr)
    {
        node = create_node(ip, port);
        node->start();
    }
    ModbusDevice *device = new ModbusDevice(name, address);
    node->add_device(device);
    return device;
}

ModbusDevice *ModbusHub::add_rtu_device(const char *name, char *iface, uint32_t baudrate, char *parity, uint8_t address)
{
    ModbusNode *node = get_node(iface);
    if (node == nullptr)
    {
        node = create_node(iface);
    }
    ModbusDevice *device = new ModbusDevice(name, address);
    node->add_device(device);
    return nullptr;
}

void ModbusHub::remove_device(ModbusDevice *device)
{
    // for (auto &node : nodes)
    // {
    //     node.remove_device(device);
    // }
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

// /**
//  * @brief Create a device and assigns the proper node according to the port
//  * @param name The name of the device
//  * @param iface The interface to use for the device. RTU:/dev/ttyUSB0:115200:8N1:O,RTU:UART_1:115200:8N1:E,
//  * @param address The modbus address of the device
//  * @return The device
//  */
// ModbusDevice *ModbusHub::add_device(const char *name, char *iface, uint8_t address)
// {
//     char iface_type[sizeof(config)];
//     char iface[sizeof(config)];
//     char ip[sizeof(config)];
//     uint16_t port;
//     uint32_t baudrate = 0;
//     char parity[4] = {0};

//     if (modbus_get_iface(config))
//     {
//         modbus_config_parse_tcp(config, &iface_type, &iface, &ip, &port);
//     }
//     else if (modbus_iface_is_rs485(config))
//     {
//         modbus_config_parse_rs485(config, &iface_type, &iface, &baudrate, &parity);
//     }

//     // Adds device to the right node that will manage it till its removed.
//     ModbusNode *node = get_node(iface);
//     if (node == nullptr)
//     {
//         node = new ModbusNode(iface);
//         node->start();
//     }

//     ModbusDevice *device = new ModbusDevice(name, address);

//     return device;
// }

ModbusNode *ModbusHub::get_node(const char *ip, uint16_t port)
{
    char iface[64];
    snprintf(iface, sizeof(iface), "%s:%u", ip, port);
    for (auto node : nodes) // Note: no & since we're iterating over pointers
    {
        if (strcmp(node->get_iface(), iface) == 0)
        {
            return node; // Return the pointer directly
        }
    }
    return nullptr;
}

ModbusNode *ModbusHub::get_node(const char *iface)
{
    for (auto node : nodes) // Note: no & since we're iterating over pointers
    {
        if (strcmp(node->get_iface(), iface) == 0)
        {
            return node; // Return the pointer directly
        }
    }
    return nullptr;
}
