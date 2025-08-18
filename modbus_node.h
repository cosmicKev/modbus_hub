// ModbusNode is a class that represents a Modbus node.
// The node is the modbus client that is connected to multiple servers via a unique interface, UART, or IP:PORT
#pragma once

#include "EZModbus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "modbus_device.h"
#include "mutex_utils.h"
#include "psram.h"
#include <forward_list>
#include <memory>



enum class ModbusNodeState
{
    IDLE,
    CONNECTING,
    CONNECTED,
    RUNNING,
    DISCONNECTED,
};

enum class ModbusNodeInterfaceType
{
    NONE, // invalid
    TCP,
    RTU,
};

class ModbusNode
{
  public:
    ModbusNode();
    ~ModbusNode();

    // Handles nodes creation
    static ModbusNode *create(const char *ip, uint16_t port);
    static ModbusNode *create(const char *iface, ModbusHAL::UART::Config uartCfg);

    // Handles devices creation
    void add_device(ModbusDevice *device);
    void remove_device(ModbusDevice *device);

    // Handles node start/stop
    void start();
    void stop();

    const char *get_iface();
    bool has_device(ModbusDevice *device);
    bool empty();

    // Handles memory allocation
    void *operator new(size_t size);
    void operator delete(void *ptr) noexcept;
    void *operator new[](size_t size);
    void operator delete[](void *ptr) noexcept;

  private:
    void initialize_communication();
    void deinitialize_communication();
    // List of devices managed by the node.
    std::forward_list<ModbusDevice *, PsramAllocator<ModbusDevice *>> devices;
    ModbusNodeState state;
    char name_[64]; // RTU:/dev/ttyUSB0 or TCP:192.168.1.100:502
    // Thread-related members
    TaskHandle_t thread_;
    bool running_;
    TaskHandle_t ez_phy_task_;
    TaskHandle_t ez_interface_task_;

    void run_worker();
    static void thread_wrapper(void *arg);

    const char *ip;
    uint16_t port;

    // EzModbus Related we only support either RTU or TCP per Node.
    ModbusHAL::TCP *tcp_phy;
    ModbusInterface::TCP *tcp_interface;
    Modbus::Client *client;
    ModbusHAL::UART *rtu_phy;
    ModbusInterface::RTU *rtu_interface;
    
    // Mutex for thread safety
    mutable SemaphoreHandle_t _mutex;

    // RTU related
    ModbusHAL::UART::Config uartCfg;
    
public:
    ModbusNodeInterfaceType interface_type;
private:
};
