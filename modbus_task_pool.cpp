/* @file modbus_task_pool.cpp
 * @brief Implementation of task pool for creating FreeRTOS tasks with static allocation but PSRAM stacks
 */

#include "modbus_task_pool.h"
#include "esp_log.h"
#include <cstdlib>

static const char *TAG = "ModbusTaskPool";

namespace ModbusTaskPool
{

TaskPool::TaskPool()
{
    // Initialize all task slots as unused
    for (size_t i = 0; i < MAX_TASKS; i++)
    {
        _taskPool[i].handle = nullptr;
        _taskPool[i].isUsed = false;
        _taskPool[i].buffer = &_staticTaskBuffers[i]; // Point to pre-allocated buffer
        _taskPool[i].stack = nullptr;
        _taskPool[i].stackSize = 0;
    }

    // Create mutex for thread safety
    _mutex = xSemaphoreCreateMutexStatic(&_mutexBuffer);
    if (!_mutex)
    {
        ESP_LOGE(TAG, "Failed to create mutex for task pool");
    }
}

TaskPool::~TaskPool()
{
    // Clean up all tasks and free PSRAM
    for (size_t i = 0; i < MAX_TASKS; i++)
    {
        if (_taskPool[i].isUsed && _taskPool[i].handle)
        {
            vTaskDelete(_taskPool[i].handle);
        }
        if (_taskPool[i].stack)
        {
            free(_taskPool[i].stack);
        }
    }

    if (_mutex)
    {
        vSemaphoreDelete(_mutex);
    }
}

TaskHandle_t TaskPool::createTask(TaskFunction_t pxTaskCode, const char *pcName, size_t usStackDepth,
                                  void *pvParameters, UBaseType_t uxPriority)
{
    if (!_mutex)
    {
        ESP_LOGE(TAG, "Task pool not initialized");
        return nullptr;
    }

    // Take mutex
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to take mutex");
        return nullptr;
    }

    TaskHandle_t result = nullptr;

    // Find available slot in pool
    for (size_t i = 0; i < MAX_TASKS; i++)
    {
        if (!_taskPool[i].isUsed)
        {
            // Allocate stack in PSRAM
            StackType_t *stack = static_cast<StackType_t *>(psram_malloc(usStackDepth * sizeof(StackType_t)));
            if (!stack)
            {
                ESP_LOGE(TAG, "Failed to allocate PSRAM for task stack, size: %zu", usStackDepth);
                xSemaphoreGive(_mutex);
                return nullptr;
            }

            // Create the task
            TaskHandle_t handle = xTaskCreateStatic(pxTaskCode, pcName, usStackDepth, pvParameters, uxPriority, stack,
                                                    &_staticTaskBuffers[i]);
            if (handle)
            {
                // Mark slot as used and store info
                _taskPool[i].handle = handle;
                _taskPool[i].isUsed = true;
                _taskPool[i].buffer = &_staticTaskBuffers[i]; // Use pre-allocated buffer
                _taskPool[i].stack = stack;
                _taskPool[i].stackSize = usStackDepth;

                result = handle;
                ESP_LOGI(TAG, "Task created successfully, slot %zu, name: %s", i, pcName);
            }
            else
            {
                // Task creation failed, free PSRAM
                free(stack);
                ESP_LOGE(TAG, "Failed to create task: %s", pcName);
            }
            break;
        }
    }

    if (!result)
    {
        ESP_LOGE(TAG, "No available slots in task pool (max: %zu)", MAX_TASKS);
    }

    // Give mutex
    xSemaphoreGive(_mutex);
    return result;
}

void TaskPool::deleteTask(TaskHandle_t xTaskToDelete)
{
    if (!_mutex || !xTaskToDelete)
    {
        return;
    }

    // Take mutex
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(1000)) != pdTRUE)
    {
        ESP_LOGE(TAG, "Failed to take mutex for task deletion");
        return;
    }

    // Find task in pool and free the slot
    for (size_t i = 0; i < MAX_TASKS; i++)
    {
        if (_taskPool[i].isUsed && _taskPool[i].handle == xTaskToDelete)
        {
            // Free PSRAM stack
            if (_taskPool[i].stack)
            {
                free(_taskPool[i].stack);
            }

            // Mark slot as unused
            _taskPool[i].handle = nullptr;
            _taskPool[i].isUsed = false;
            _taskPool[i].buffer = nullptr;
            _taskPool[i].stack = nullptr;
            _taskPool[i].stackSize = 0;

            ESP_LOGI(TAG, "Task deleted and slot %zu freed", i);
            break;
        }
    }

    // Give mutex
    xSemaphoreGive(_mutex);

    // Delete the task (this should be done after freeing the slot)
    vTaskDelete(xTaskToDelete);
}

size_t TaskPool::getAvailableTasks() const
{
    if (!_mutex)
    {
        return 0;
    }

    size_t count = 0;
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        for (size_t i = 0; i < MAX_TASKS; i++)
        {
            if (!_taskPool[i].isUsed)
            {
                count++;
            }
        }
        xSemaphoreGive(_mutex);
    }
    return count;
}

size_t TaskPool::getUsedTasks() const
{
    if (!_mutex)
    {
        return 0;
    }

    size_t count = 0;
    if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        for (size_t i = 0; i < MAX_TASKS; i++)
        {
            if (_taskPool[i].isUsed)
            {
                count++;
            }
        }
        xSemaphoreGive(_mutex);
    }
    return count;
}

bool TaskPool::isPoolFull() const
{
    return getAvailableTasks() == 0;
}

// Global instance
TaskPool g_taskPool;

} // namespace ModbusTaskPool