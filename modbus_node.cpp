#include "modbus_node.h"
#include "drivers/ModbusHAL_UART.h"
#define EZMODBUS_USE_DYNAMIC_QUEUE 1
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
    running_ = false;
    tcp_phy = nullptr;
    tcp_interface = nullptr;
    client = nullptr;
    rtu_phy = nullptr;
    rtu_interface = nullptr;
    interface_type = ModbusNodeInterfaceType::NONE;
    state = ModbusNodeState::IDLE;
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
        ESP_LOGI(TAG, "Creating node for interface: %s:%u", ip, port);
        snprintf(node->name_, sizeof(node->name_), "%s:%u", ip, port);
        node->ip = ip;
        node->port = port;
        node->interface_type = ModbusNodeInterfaceType::TCP;
    }
    return node;
}

/*
 * @brief Create a node for a RTU interface
 * @param iface The interface to use for the node. RTU:/dev/ttyUSB0:115200:8N1:O,RTU:UART_1:115200:8N1:E,
 * @return The node
 */
ModbusNode *ModbusNode::create(const char *iface)
{
    ModbusNode *node = new ModbusNode();
    if (node)
    {
        ESP_LOGI(TAG, "Creating node for interface: %s", iface);
        snprintf(node->name_, sizeof(node->name_), "%s", iface);
        node->interface_type = ModbusNodeInterfaceType::RTU;
    }
    return node;
}

ModbusNode::~ModbusNode()
{
    if (interface_type == ModbusNodeInterfaceType::TCP)
    {
        ModbusTaskPool::g_taskPool.deleteTask(thread_);

        tcp_phy->stop();
        ModbusTaskPool::g_taskPool.deleteTask(ez_phy_task_);
        ModbusTaskPool::g_taskPool.deleteTask(ez_interface_task_);
    }
    else if (interface_type == ModbusNodeInterfaceType::RTU)
    {
        ModbusTaskPool::g_taskPool.deleteTask(thread_);
        ModbusTaskPool::g_taskPool.deleteTask(ez_interface_task_);
    }

    // Clean up other objects
    if (rtu_interface)
    {
        delete rtu_interface;
    }
    if (rtu_phy)
    {
        delete rtu_phy;
    }
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
    initialize_communication();
    vTaskDelay(pdMS_TO_TICKS(1000));
    thread_ = ModbusTaskPool::g_taskPool.createTask(thread_wrapper, name_, 4096, this, 5);
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
    if (!running_)
    {
        return; // Already stopped
    }

    running_ = false;

    if (thread_)
    {
        vTaskDelete(thread_);
        thread_ = nullptr;
    }

    state = ModbusNodeState::IDLE;
}

const char *ModbusNode::get_iface()
{
    return name_;
}

// Thread wrapper function implementation
void ModbusNode::thread_wrapper(void *arg)
{
    ModbusNode *node = static_cast<ModbusNode *>(arg);
    ESP_LOGI(TAG, "ModbusNode thread wrapper started for interface: %s", node->get_iface());
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
        tcp_phy = new ModbusHAL::TCP(ip, port, false);
        tcp_interface = new ModbusInterface::TCP(*tcp_phy, Modbus::CLIENT);
        client = new Modbus::Client(*tcp_interface, 10000);


        // Initialize the TCP connection
        bool success = tcp_phy->begin();
        if (!success)
        {
            ESP_LOGE(TAG, "%s: Failed to initialize communication on TCP", name_);
            return;
        }
        ez_phy_task_ = ModbusTaskPool::g_taskPool.createTask(tcp_phy->tcpTask, name_, 4096, tcp_phy, 5);

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
        ez_interface_task_ = ModbusTaskPool::g_taskPool.createTask(tcp_interface->rxTxTask, name_, 4096, tcp_interface, 5);

    }
    else if (interface_type == ModbusNodeInterfaceType::RTU)
    {
        ModbusHAL::UART::Config uartCfg = {
    .uartNum = UART_NUM_2, .baud = 38400, .config = ModbusHAL::UART::CONFIG_8N1, .rxPin = 44, .txPin = 43, .dePin = 6};

        rtu_phy = new ModbusHAL::UART(uartCfg);
        rtu_interface = new ModbusInterface::RTU(*rtu_phy, Modbus::CLIENT);
        client = new Modbus::Client(*rtu_interface, 1000);

        esp_err_t rtu_result = rtu_phy->begin();
        if (rtu_result != ESP_OK)
        {
            ESP_LOGE(TAG, "%s: Failed to initialize communication on RTU, error: %s", name_,
                     esp_err_to_name(rtu_result));
        }
        ez_interface_task_ = ModbusTaskPool::g_taskPool.createTask(rtu_interface->rxTxTask, name_, 4096, rtu_interface, 5);


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
    }
    else
    {
        ESP_LOGE(TAG, "%s: Interface type not supported.", name_);
    }
}

// Worker thread main function
void ModbusNode::run_worker()
{
    ESP_LOGI(TAG, "%s: Worker started.", name_);

    while (running_)
    {
        for (auto device : devices)
        {
            if (!device)
                continue;

            const auto &requests = device->get_requests_list();
            for (auto request : requests)
            {

                if (pdTICKS_TO_MS(xTaskGetTickCount()) <=
                    request->get_last_read_time_ms() + request->get_polling_interval())
                {
                    continue;
                }

                if (!request)
                    continue;

                ESP_LOGI(TAG, "%s: Device %s request %s", name_, device->get_name(), request->get_name());

                // Check if client is available
                if (!client)
                {
                    ESP_LOGE(TAG, "%s: Client not initialized", name_);
                    continue;
                }

                Modbus::Frame frame;
                frame.type = Modbus::REQUEST;
                frame.slaveId = device->get_address();
                frame.fc = static_cast<Modbus::FunctionCode>(request->get_function_code());
                frame.regAddress = request->get_address();
                frame.regCount = request->get_size();
                ESP_LOGI(TAG, "%s: Sending request slaveId =%u %s(%u) - %u / %u", name_, frame.slaveId,
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
                ESP_LOGI(TAG, "%s: Request %s sent", device->get_name(), request->get_name());
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "%s: Worker stopped.", name_);
    vTaskDelete(nullptr);
}

// Main worker function
void ModbusNode::worker()
{
}

// Add device to this node
void ModbusNode::add_device(ModbusDevice *device)
{
    if (device)
    {
        devices.push_front(device); // Store the pointer, not a copy
        ESP_LOGI(TAG, "%s: Device %s added.", name_, device->get_name());
    }
}

// Remove device from this node
void ModbusNode::remove_device(ModbusDevice *device)
{
    if (device)
    {
        // Note: std::forward_list doesn't have remove() method
        // You might want to use a different container or implement custom removal
        ESP_LOGW(TAG, "Device removal not implemented for forward_list");
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
