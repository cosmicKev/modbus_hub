#include "modbus_node.h"
#include "drivers/ModbusHAL_UART.h"
#include "EZModbus.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "modbus_task_pool.h"
#include "modbus_utils.h"
#include "psram.h"

constexpr char TAG[] = "ModbusNode";

ModbusNode::ModbusNode()
{
    // Initialize mutex for thread safety
    _mutex = xSemaphoreCreateMutex();
    if (!_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex for ModbusNode");
    }
    
    running_ = false;
    tcp_phy = nullptr;
    tcp_interface = nullptr;
    client = nullptr;
    rtu_phy = nullptr;
    rtu_interface = nullptr;
    interface_type = ModbusNodeInterfaceType::NONE;
    state = ModbusNodeState::IDLE;
    uartCfg = {
        .uartNum = UART_NUM_MAX,
        .baud = 0,
        .config = ModbusHAL::UART::CONFIG_8N1,
        .rxPin = 0,
        .txPin = 0,
        .dePin = 0,
    };
}

/*
 * @brief Create a node for a TCP interface
 * @param ip The IP address of the node
 * @param port The port of the node
 * @return The node
 */
ModbusNode *ModbusNode::create(const char *ip, uint16_t port)
{
    ModbusNode *node = new ModbusNode();
    if (node)
    {
        ESP_LOGI(TAG, "Creating node for TCP interface: %s:%u", ip, port);
        snprintf(node->name_, sizeof(node->name_), "%s:%u", ip, port);
        node->ip = ip;
        node->port = port;
        node->interface_type = ModbusNodeInterfaceType::TCP;
        return node;
    }
    return nullptr;
}

/*
 * @brief Create a node for a RTU interface
 * @param iface The interface to use for the node. RTU:/dev/ttyUSB0:115200:8N1:O,RTU:UART_1:115200:8N1:E,
 * @return The node
 */
ModbusNode *ModbusNode::create(const char *iface, ModbusHAL::UART::Config uartCfg)
{
    
    ModbusNode *node = new ModbusNode();
    if (node)
    {
        ESP_LOGI(TAG, "Creating node for RTU interface: %s", iface);
        snprintf(node->name_, sizeof(node->name_), "%s", iface);
        node->interface_type = ModbusNodeInterfaceType::RTU;
        node->uartCfg = uartCfg;
        return node;
    }
    return nullptr;
}

ModbusNode::~ModbusNode()
{
    ESP_LOGI(TAG, "%s: Destroying ModbusNode.", name_);

    if(thread_)
    {
        ModbusTaskPool::g_taskPool.deleteTask(thread_);
    }
    
    // Clean up all devices first
    for (auto device : devices) {
        if (device) {
            delete device;
        }
    }
    devices.clear();
    
    deinitialize_communication();
    
    if (_mutex) {
        vSemaphoreDelete(_mutex);
    }
}

void ModbusNode::start()
{
    initialize_communication();
    ESP_LOGI(TAG, "%s: Starting ModbusNode.", name_);
    if (running_)
    {
        ESP_LOGI(TAG, "%s: Already running.", name_);
        return; // Already running
    }

    running_ = true;
    thread_ = ModbusTaskPool::g_taskPool.createTask(thread_wrapper, name_, 10*4096, this, 5);
    if (thread_)
    {
        ESP_LOGI(TAG, "%s: Thread created", name_);
        state = ModbusNodeState::RUNNING;
    }
    else
    {
        ESP_LOGE(TAG, "%s: Failed to create thread", name_);
        running_ = false;
        thread_ = nullptr;
        return;
    }
}

void ModbusNode::deinitialize_communication()
{
    ScopedMutex lock(_mutex);
    if (!lock.isLocked()) {
        ESP_LOGE(TAG, "%s: Failed to acquire mutex for communication deinitialization", name_);
        return;
    }
    
    if (interface_type == ModbusNodeInterfaceType::TCP)
    {
        // Delete the tasks already
        ModbusTaskPool::g_taskPool.deleteTask(ez_phy_task_);
        ModbusTaskPool::g_taskPool.deleteTask(ez_interface_task_);
        // Kill all rest.
        tcp_phy->stop();
        delete tcp_phy;
        tcp_phy = nullptr;
        delete tcp_interface;
        tcp_interface = nullptr;
        delete client;
        client = nullptr;

    }
    else if (interface_type == ModbusNodeInterfaceType::RTU)
    {
        ModbusTaskPool::g_taskPool.deleteTask(ez_interface_task_);
        delete client;
        client = nullptr;

        delete rtu_interface;
        rtu_interface = nullptr;

        rtu_phy->end();
        delete rtu_phy;
        rtu_phy = nullptr;
    }
}

const char *ModbusNode::get_iface()
{
    return name_;
}

// Thread wrapper function implementation for FreeRTOS task
void ModbusNode::thread_wrapper(void *arg)
{
    ModbusNode *node = static_cast<ModbusNode *>(arg);
    if (node)
    {
        ESP_LOGI(TAG, "ModbusNode thread wrapper running for interface: %s", node->get_iface());
        node->run_worker();
    }
}


void ModbusNode::initialize_communication()
{
    if (interface_type == ModbusNodeInterfaceType::TCP)
    {
        // Allocate & Create for PHY
        tcp_phy = static_cast<ModbusHAL::TCP*>(psram_malloc(sizeof(ModbusHAL::TCP)));
        tcp_interface = static_cast<ModbusInterface::TCP*>(psram_malloc(sizeof(ModbusInterface::TCP)));
        client = static_cast<Modbus::Client*>(psram_malloc(sizeof(Modbus::Client)));

        if(!tcp_phy || !tcp_interface || !client)
        {
            ESP_LOGE(TAG, "Failed to allocate TCP PHY, interface or client in PSRAM");
            if(tcp_phy)
            {
                free(tcp_phy);
                tcp_phy = nullptr;
            }
            if(tcp_interface)
            {
                free(tcp_interface);
                tcp_interface = nullptr;
            }
            if(client)
            {
                free(client);
                client = nullptr;
            }
            return;
        }

        tcp_phy = new (tcp_phy) ModbusHAL::TCP(ip, port);
        tcp_interface = new (tcp_interface) ModbusInterface::TCP(*tcp_phy,Modbus::CLIENT);

        // Initialize the TCP connection
        bool success = tcp_phy->begin();
        if (!success)
        {
            ESP_LOGE(TAG, "%s:123 Failed to initialize communication on TCP", name_);
            return;
        }
        ez_phy_task_ = ModbusTaskPool::g_taskPool.createTask(tcp_phy->tcpTask, name_, 10*4096, tcp_phy, 5);

        client = new (client) Modbus::Client(*tcp_interface, 10000);
        Modbus::Client::Result result = client->begin();
        if (result != Modbus::Client::SUCCESS)
        {
            ESP_LOGE(TAG, "%s: Failed to initialize client on TCP: %s", name_, Modbus::Client::toString(result));
            return;
        }
        else
        {
            ESP_LOGI(TAG, "%s: Initialized communication on TCP", name_);
        }
        ez_interface_task_ = ModbusTaskPool::g_taskPool.createTask(tcp_interface->rxTxTask, name_, 10*4096, tcp_interface, 5);

    }
    else if (interface_type == ModbusNodeInterfaceType::RTU)
    {

        ESP_LOGI(TAG, "%s: Initializing communication on RTU", name_);
        ESP_LOGI(TAG, "%s: UART config: %d, %lu, %lu, %d, %d %d", name_, uartCfg.uartNum, (unsigned long)uartCfg.baud, (unsigned long)uartCfg.config, uartCfg.rxPin, uartCfg.txPin, uartCfg.dePin);
        
        // Pre-allocate PSRAM memory for RTU objects
        rtu_phy = static_cast<ModbusHAL::UART*>(psram_malloc(sizeof(ModbusHAL::UART)));
        if (!rtu_phy) {
            ESP_LOGE(TAG, "%s: Failed to allocate RTU PHY in PSRAM", name_);
            return;
        }
        // Use placement new to construct the object in PSRAM
        rtu_phy = new (rtu_phy) ModbusHAL::UART(uartCfg);
        
        rtu_interface = static_cast<ModbusInterface::RTU*>(psram_malloc(sizeof(ModbusInterface::RTU)));
        if (!rtu_interface) {
            ESP_LOGE(TAG, "%s: Failed to allocate RTU interface in PSRAM", name_);
            // Clean up PHY
            // rtu_phy->~UART();
            free(rtu_phy);
            rtu_phy = nullptr;
            return;
        }
        // Use placement new to construct the object in PSRAM
        rtu_interface = new (rtu_interface) ModbusInterface::RTU(*rtu_phy, Modbus::CLIENT);
        
        client = static_cast<Modbus::Client*>(psram_malloc(sizeof(Modbus::Client)));
        if (!client) {
            ESP_LOGE(TAG, "%s: Failed to allocate Modbus client in PSRAM", name_);
            // Clean up interface and PHY
            rtu_interface->~RTU();
            free(rtu_interface);
            rtu_interface = nullptr;
            rtu_phy->~UART();
            free(rtu_phy);
            rtu_phy = nullptr;
            return;
        }
        // Use placement new to construct the object in PSRAM
        client = new (client) Modbus::Client(*rtu_interface, 10000);
        

        esp_err_t rtu_result = rtu_phy->begin();
        if (rtu_result != ESP_OK)
        {
            ESP_LOGE(TAG, "%s: Failed to initialize communication on RTU, error: %s", name_,
                     esp_err_to_name(rtu_result));
        }


        Modbus::Client::Result result = client->begin();
        if (result != Modbus::Client::SUCCESS)
        {
            ESP_LOGE(TAG, "%s: Failed to initialize client on RTU: %s", name_, Modbus::Client::toString(result));
            return;
        }
        else
        {
            ESP_LOGI(TAG, "%s: Initialized communication on RTU", name_);
        }
        ESP_LOGI(TAG, "RTU::interface %p", rtu_interface);
        ez_interface_task_ = ModbusTaskPool::g_taskPool.createTask(ModbusInterface::RTU::rxTxTask, name_, 10*4096, rtu_interface, 5);

    }
}

// Worker thread main function
void ModbusNode::run_worker()
{
    ESP_LOGI(TAG, "%s: Worker started.", name_);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    while (running_)
    {
        // Check if client is available
        if (!client)
        {
            ESP_LOGE(TAG, "%s: Client not initialized", name_);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        for (auto device : devices)
        {
            if (!device)
                continue;
            const auto &requests = device->get_requests_list();
            for (auto request : requests)
            {
                if(!request){
                    ESP_LOGE(TAG, "%s: Request is null", name_);
                    continue;
                }
                if (pdTICKS_TO_MS(xTaskGetTickCount()) <=
                    request->get_last_read_time_ms() + request->get_polling_interval())
                {
                    continue;
                }

                ESP_LOGI(TAG, "%s: Device %s request %s", name_, device->get_name(), request->get_name());


                Modbus::Frame frame;
                frame.type = Modbus::REQUEST;
                frame.slaveId = device->get_address();
                frame.fc = static_cast<Modbus::FunctionCode>(request->get_function_code());
                frame.regAddress = request->get_address();
                frame.regCount = request->get_size();
                ESP_LOGI(TAG, "%s: Sending request slaveId =%u %s(%u) - Addr:%u len:%u", name_, frame.slaveId,
                         request->get_name(), frame.fc, frame.regAddress, frame.regCount);
                Modbus::Client::Result result = client->sendRequest(frame, frame);

                if (result == Modbus::Client::SUCCESS)
                {
                    modbus_print_response(device->get_name(), frame, request->get_size() * 2);
                }
                else if (result == Modbus::Client::ERR_BUSY)
                {
                    ESP_LOGE(TAG, "Modbus exception. Result: %s Frame: %s", Modbus::Client::toString(result),
                             Modbus::toString(frame.exceptionCode));
                    // Should not be busy
                    ESP_LOGE(TAG, "Modbus busy. Aborting transaction.");
                }
                else
                {
                    ESP_LOGE(TAG, "Modbus exception. Result: %s Frame: %s", Modbus::Client::toString(result),
                             Modbus::toString(frame.exceptionCode));
                }
                request->set_last_read_time_ms(pdTICKS_TO_MS(xTaskGetTickCount()));
                ESP_LOGI(TAG, "%s: Request %s completed", name_, request->get_name());
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_LOGI(TAG, "%s: Worker stopped.", name_);
    vTaskDelete(nullptr);
}

// Add device to this node
void ModbusNode::add_device(ModbusDevice *device)
{
    ScopedMutex lock(_mutex);
    if (!lock.isLocked()) {
        ESP_LOGE(TAG, "%s: Failed to acquire mutex for device addition", name_);
        return;
    }
    
    if (device)
    {
        devices.push_front(device); // Store the pointer, not a copy
        ModbusDevice *tmp = devices.front();
        ESP_LOGI(TAG, "%s: Device %s added.", name_, tmp->get_name());

    }
}

// Remove device from this node
void ModbusNode::remove_device(ModbusDevice *device)
{
    ScopedMutex lock(_mutex);
    if (!lock.isLocked()) {
        ESP_LOGE(TAG, "%s: Failed to acquire mutex for device removal", name_);
        return;
    }
    
    if (device)
    {
        ESP_LOGI(TAG, "%s: Device %s removed.", name_, device->get_name());
        // Destruct 
        delete device;
        // Removes the pointer
        devices.remove(device);
    }
    else
    {
        ESP_LOGE(TAG, "%s: Device is null", name_);
    }
}

void *ModbusNode::operator new(size_t size)
{
    return psram_malloc(size);
}

void ModbusNode::operator delete(void *ptr) noexcept
{
    free(ptr);
}

void *ModbusNode::operator new[](size_t size)
{
    return psram_malloc(size);
}

void ModbusNode::operator delete[](void *ptr) noexcept
{
    free(ptr);
}

bool ModbusNode::has_device(ModbusDevice *device)
{
    ScopedMutex lock(_mutex);
    if (!lock.isLocked()) {
        ESP_LOGE(TAG, "%s: Failed to acquire mutex for device check", name_);
        return false;
    }
    
    for (auto &dev : devices)
    {
        if (dev == device){
            return true;
        }
    }
    return false;
}

bool ModbusNode::empty()
{
    ScopedMutex lock(_mutex);
    if (!lock.isLocked()) {
        ESP_LOGE(TAG, "%s: Failed to acquire mutex for empty check", name_);
        return true;
    }
    
    return devices.empty();
}