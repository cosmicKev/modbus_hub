// ModbusNode is a class that represents a Modbus node.
// The node is the modbus client that is connected to multiple servers via a unique interface, UART, or IP:PORT
#pragma once

#include "EZModbus.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "modbus_device.h"
#include "mutex_utils.h"
#include "psram.h"
#include <forward_list>
#include <memory>



enum class ModbusNodeState
{
    IDLE,
    WAITING_FOR_NETWORK,
    CONNECTING_TO_DEVICE,
    RUNNING,
    SUSPENDED,  // Add suspended state
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

    // Add suspend/resume methods
    void suspend();
    void resume();
    bool is_suspended() const { return _is_suspended; }

    bool read_request(ModbusDevice *device, ModbusData *request);
    bool write_request(ModbusDevice *device, ModbusData *request);

    const char *get_iface();
    bool has_device(ModbusDevice *device);
    bool empty();

    // States
    void running_state();

    // Handles mutex
    bool lock(uint32_t timeout = portMAX_DELAY);
    bool unlock();


    EventGroupHandle_t notification_event_group;

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
    ModbusNodeState _previous_state;
    char name_[64]; // RTU:/dev/ttyUSB0 or TCP:192.168.1.100:502
    // Thread-related members
    TaskHandle_t thread_;
    volatile bool running_;
    TaskHandle_t ez_phy_task_;
    TaskHandle_t ez_interface_task_;

    void run_worker();
    static void thread_wrapper(void *arg);

    char ip[16];
    uint16_t port;

    // EzModbus Related we only support either RTU or TCP per Node.
    ModbusHAL::TCP *tcp_phy;
    ModbusInterface::TCP *tcp_interface;
    Modbus::Client *client;
    ModbusHAL::UART *rtu_phy;
    ModbusInterface::RTU *rtu_interface;
    
    // Mutex for thread safety
    mutable SemaphoreHandle_t _mutex;

    // Add suspension state
    volatile bool _is_suspended;
    volatile bool _has_state_change;

    // RTU related
    ModbusHAL::UART::Config uartCfg;
    
public:
    ModbusNodeInterfaceType interface_type;
private:
};
