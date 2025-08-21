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
#include "time_utils.h"
#include "network.h"
    
constexpr char TAG[] = "ModbusNode";

ModbusNode::ModbusNode()
{
    // Initialize mutex for thread safety
    _mutex = xSemaphoreCreateMutex();
    if (!_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex for ModbusNode");
    }

    running_ = false;
    suspended_ = false;  // Initialize suspension state
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
        strncpy(node->ip, ip, sizeof(node->ip));
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
    if (!lock(portMAX_DELAY))
    {
        ESP_LOGE(TAG, "%s: Failed to lock mutex for ModbusNode destruction", name_);
        return;
    }
    stop();
    // Give thread some time to stop.
    vTaskDelay(pdMS_TO_TICKS(100));

    if(thread_)
    {
        ModbusTaskPool::g_taskPool.deleteTask(thread_);
    }

    // Deinitialize communication
    deinitialize_communication();
    // Give thread some time to stop.
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Clean up all devices first
    for (auto device : devices) {
        if (device) {
            delete device;
        }
    }
    devices.clear();
    
    
    if (_mutex) {
        vSemaphoreDelete(_mutex);
    }

    ESP_LOGI(TAG, "%s: Destroying ModbusNode.", name_);

    
}

void ModbusNode::start()
{
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

void ModbusNode::stop()
{
    ESP_LOGI(TAG, "%s: Stopping ModbusNode.", name_);
    if (running_)
    {
        running_ = false;
        thread_ = nullptr;
        state = ModbusNodeState::IDLE;
        ESP_LOGI(TAG, "%s: Stopped.", name_);
    }

}

void ModbusNode::suspend()
{
    ESP_LOGI(TAG, "%s: Suspending ModbusNode.", name_);
    
    if (!running_) {
        ESP_LOGW(TAG, "%s: Cannot suspend - not running", name_);
        return;
    }
    
    if (suspended_) {
        ESP_LOGW(TAG, "%s: Already suspended", name_);
        return;
    }
    
    suspended_ = true;
    state = ModbusNodeState::SUSPENDED;
    ESP_LOGI(TAG, "%s: ModbusNode suspended.", name_);
}

void ModbusNode::resume()
{
    ESP_LOGI(TAG, "%s: Resuming ModbusNode.", name_);
    
    if (!suspended_) {
        ESP_LOGW(TAG, "%s: Not suspended", name_);
        return;
    }
    
    suspended_ = false;
    state = ModbusNodeState::RUNNING;
    
    ESP_LOGI(TAG, "%s: ModbusNode resumed.", name_);
}

void ModbusNode::deinitialize_communication()
{
    if (interface_type == ModbusNodeInterfaceType::TCP)
    {
        ModbusTaskPool::g_taskPool.deleteTask(ez_phy_task_);
        ModbusTaskPool::g_taskPool.deleteTask(ez_interface_task_);
        delete tcp_interface;
        tcp_interface = nullptr;
        // Kill all rest.
        tcp_phy->stop();
        delete tcp_phy;
        tcp_phy = nullptr;
        delete client;
        client = nullptr;
        vTaskDelay(pdMS_TO_TICKS(100));
        // Delete the tasks already


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
        ESP_LOGI(TAG, "%s: Initializing communication on TCP. %s:%u", name_, ip, port);
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

        client = new (client) Modbus::Client(*tcp_interface, 5000);
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
        ESP_LOGI(TAG, "%s: UART config: UartNum:%d, Baud:%lu, Config:0x%04" PRIx32 ", RxPin:%d, TxPin:%d, DePin:%d", name_, uartCfg.uartNum, (unsigned long)uartCfg.baud, uartCfg.config, uartCfg.rxPin, uartCfg.txPin, uartCfg.dePin);
        
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

bool ModbusNode::read_request(ModbusDevice *device, ModbusData *request)
{
    Modbus::Frame frame;
    frame.type = Modbus::REQUEST;
    frame.slaveId = device->get_address();
    frame.fc = static_cast<Modbus::FunctionCode>(request->get_function_code());
    frame.regAddress = request->get_address();
    frame.regCount = request->get_size();
    ESP_LOGI(TAG, "%s: Sending request %s(%u)- %s Address:0x%x(%d) Code:0x%x Len:%u", name_, device->get_name(), device->get_address(), request->get_name(), request->get_address(), request->get_address(), frame.fc, frame.regCount);
    Modbus::Client::Result result = client->sendRequest(frame, frame);

    if (result == Modbus::Client::SUCCESS)
    {
        request->set_status(ModbusDataStatus::SUCCESS);
        request->set_last_read_time_ms(pdTICKS_TO_MS(xTaskGetTickCount()));
        request->update_data(frame.data);
        // modbus_print_response(device->get_name(), frame, request->get_size() * 2);
    }
    else if (result == Modbus::Client::ERR_BUSY)
    {
        ESP_LOGE(TAG, "%s: Request: %s(%u)- %s Modbus exception. Result: %s Frame: %s", name_, device->get_name(), device->get_address(), request->get_name(), Modbus::Client::toString(result),
                 Modbus::toString(frame.exceptionCode));
        return false;
    }
    else if (result == Modbus::Client::ERR_TIMEOUT)
    {
        request->set_status(ModbusDataStatus::TIMEDOUT);
        ESP_LOGE(TAG, "%s: Request: %s(%u)- %s Modbus exception. Result: %s Frame: %s", name_, device->get_name(), device->get_address(), request->get_name(), Modbus::Client::toString(result),
                 Modbus::toString(frame.exceptionCode));
        return false;
    }
    else
    {
        request->set_status(ModbusDataStatus::ERROR);
        ESP_LOGE(TAG, "%s: %s(%u)- %s Exception: %s | Frame: %s", 
            name_, device->get_name(), device->get_address(), request->get_name(), Modbus::Client::toString(result), Modbus::toString(frame.exceptionCode));
        return false;
    }
    return true;
}

bool ModbusNode::write_request(ModbusDevice *device, ModbusData *request)
{

    // We can only make write requests once.
    if(request->get_status()!=ModbusDataStatus::REQUESTED)
    {
        return false;
    }

    Modbus::Frame frame;
    frame.type = Modbus::REQUEST;
    frame.slaveId = device->get_address();
    frame.fc = static_cast<Modbus::FunctionCode>(request->get_function_code());
    frame.regAddress = request->get_address();
    frame.regCount = request->get_size();

    memcpy(frame.data.data(), request->data(), request->get_size() * 2);
    ESP_LOGI(TAG, "%s: Sending write request %s(%u)- %s Address:0x%x(%d) Code:0x%x Len:%u", name_, device->get_name(), device->get_address(), request->get_name(), request->get_address(), request->get_address(), frame.fc, frame.regCount);
    Modbus::Client::Result result = client->sendRequest(frame, frame);

    if (result == Modbus::Client::SUCCESS)
    {
        request->set_status(ModbusDataStatus::SUCCESS);
        request->set_last_read_time_ms(pdTICKS_TO_MS(xTaskGetTickCount()));
        request->update_data(frame.data);
    }
    else if (result == Modbus::Client::ERR_BUSY)
    {
        ESP_LOGE(TAG, "Modbus exception. Result: %s Frame: %s", Modbus::Client::toString(result),
                 Modbus::toString(frame.exceptionCode));
        // Should not be busy
        ESP_LOGE(TAG, "Modbus busy. Aborting transaction.");
        return false;
    }
    else if (result == Modbus::Client::ERR_TIMEOUT)
    {
        request->set_status(ModbusDataStatus::TIMEDOUT);
        ESP_LOGE(TAG, "Modbus exception. Result: %s Frame: %s", Modbus::Client::toString(result),
                 Modbus::toString(frame.exceptionCode));
        // Should not be busy
        ESP_LOGE(TAG, "Modbus busy. Aborting transaction.");
        return false;
    }
    else
    {
        request->set_status(ModbusDataStatus::ERROR);
        ESP_LOGE(TAG, "%s: %s(%u)- %s Exception: %s | Frame: %s", 
            name_, device->get_name(), device->get_address(), request->get_name(), Modbus::Client::toString(result), Modbus::toString(frame.exceptionCode));
        return false;
    }
    return true;
}



// Worker thread main function
void ModbusNode::run_worker()
{
    //Wait for network to be ready
    while (strlen(get_network_ip_address()) < 1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    // Initialize communication
    initialize_communication();
    ESP_LOGI(TAG, "%s: Worker started.", name_);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    while (running_)
    {
        // Check for suspension at the beginning of the loop
        if (suspended_) {
            ESP_LOGI(TAG, "%s: Task suspended, waiting for resume...", name_);
            
            // Wait in a tight loop until resumed or stopped
            while (suspended_ && running_) {
                vTaskDelay(pdMS_TO_TICKS(100));  // Small delay to prevent busy waiting
            }
            
            if (!running_) {
                ESP_LOGI(TAG, "%s: Task stopped while suspended", name_);
                break;
            }
            
            ESP_LOGI(TAG, "%s: Task resumed, continuing...", name_);
        }

        for (auto device : devices)
        {
            if (!device)
                continue;
            if(!device->lock(1000))
            {
                ESP_LOGE(TAG, "%s: Failed to lock device %s", name_, device->get_name());
                continue;
            }

            bool updated_once = false;
            const auto &requests = device->get_requests_list();
            for (auto request : requests)
            {
                {
                    ModbusData::ScopedLock lock(request, 100);
                    if (!lock.is_locked())
                    {
                        ESP_LOGE(TAG, "%s: Failed to lock request %s", name_, request->get_name());
                        continue;
                    }
                    if(suspended_ || !running_)
                    {
                        break;
                    }

                    if(!request){
                        ESP_LOGE(TAG, "%s: Request is null", name_);
                        continue;
                    }

                    // Check if its time to read or if its a single request.
                    if ((pdTICKS_TO_MS(xTaskGetTickCount()) <= request->get_last_read_time_ms() + request->get_polling_interval()) &&
                        request->get_status() != ModbusDataStatus::REQUESTED)
                    {
                        continue;
                    }

                    if(request->get_function_code() == Modbus::FunctionCode::READ_HOLDING_REGISTERS && read_request(device, request))
                    {
                        ESP_LOGD(TAG, "%s: Success Read request %s(%u)- %s", name_, device->get_name(), device->get_address(), request->get_name());
                        updated_once = true;
                    }
                    else if((request->get_function_code() == Modbus::FunctionCode::WRITE_REGISTER  || (request->get_function_code() == Modbus::FunctionCode::WRITE_MULTIPLE_REGISTERS)) && write_request(device, request))
                    {
                        updated_once = true;
                        ESP_LOGD(TAG, "%s: Success write request %s(%u)- %s", name_, device->get_name(), device->get_address(), request->get_name());
                        continue;
                    }
                    // Required before next query.
                    vTaskDelay(pdMS_TO_TICKS(device->get_wait_after_query()));
                }
            }

            // Only update time if we sucessefully had at least one good communication.
            if(updated_once)
            {
                device->set_timestamp(get_time());
            }
            device->unlock();

            // Fast out of loop if suspended.
            if(suspended_ || !running_)
            {
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
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
        ESP_LOGI(TAG, "%s: Removing device %s.", name_, device->get_name());
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

bool ModbusNode::lock(uint32_t timeout)
{
    if (!_mutex)
    {
        ESP_LOGE(TAG, "%s: Mutex is not initialized", name_);
        return false;
    }
    return xSemaphoreTake(_mutex, timeout) == pdTRUE;
}

bool ModbusNode::unlock()
{
    return xSemaphoreGive(_mutex) == pdTRUE;
}