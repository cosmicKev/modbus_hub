/* @file modbus_task_pool.h
 * @brief Task pool for creating FreeRTOS tasks with static allocation but PSRAM stacks
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <cstddef>

namespace ModbusTaskPool
{

class TaskPool
{
  public:
    static constexpr size_t MAX_TASKS = CONFIG_MODBUSHUB_MAX_ALLOWED_TASKS_POOL;

    struct TaskInfo
    {
        TaskHandle_t handle = nullptr;
        bool isUsed = false;
        StaticTask_t *buffer = nullptr;
        StackType_t *stack = nullptr;
        size_t stackSize = 0;
    };

    TaskPool();
    ~TaskPool();

    // Disable copy and assignment
    TaskPool(const TaskPool &) = delete;
    TaskPool &operator=(const TaskPool &) = delete;

    // Create a task from the pool with PSRAM stack and StaticTask_t
    TaskHandle_t createTask(TaskFunction_t pxTaskCode, const char *pcName, size_t usStackDepth, void *pvParameters,
                            UBaseType_t uxPriority);

    // Delete a task and return it to the pool
    void deleteTask(TaskHandle_t xTaskToDelete);

    // Get pool statistics
    size_t getAvailableTasks() const;
    size_t getUsedTasks() const;
    bool isPoolFull() const;

  private:
    TaskInfo _taskPool[MAX_TASKS];
    StaticTask_t _staticTaskBuffers[MAX_TASKS]; // Pre-allocated StaticTask_t buffers
    mutable SemaphoreHandle_t _mutex;
    StaticSemaphore_t _mutexBuffer;
};

// Global instance
extern TaskPool g_taskPool;

} // namespace ModbusTaskPool