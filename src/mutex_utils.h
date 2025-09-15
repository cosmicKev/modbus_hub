/**
 * @file mutex_utils.h
 * @brief RAII mutex wrapper utilities for thread safety
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/**
 * @brief RAII Mutex wrapper for automatic lock/unlock
 * 
 * This class automatically locks a mutex in the constructor and unlocks it in the destructor,
 * ensuring proper resource management even when exceptions occur.
 */
class ScopedMutex {
private:
    SemaphoreHandle_t mutex_;
    bool locked_;

public:
    /**
     * @brief Constructor - automatically locks the mutex
     * @param mutex The mutex handle to lock
     * @param timeout Timeout for mutex acquisition in ticks
     */
    ScopedMutex(SemaphoreHandle_t mutex, TickType_t timeout = pdMS_TO_TICKS(1000)) 
        : mutex_(mutex), locked_(false) {
        if (mutex_) {
            locked_ = (xSemaphoreTake(mutex_, timeout) == pdTRUE);
        }
    }

    /**
     * @brief Destructor - automatically unlocks the mutex if it was locked
     */
    ~ScopedMutex() {
        if (mutex_ && locked_) {
            xSemaphoreGive(mutex_);
        }
    }

    /**
     * @brief Check if the mutex was successfully locked
     * @return true if locked, false otherwise
     */
    bool isLocked() const { return locked_; }
    
    /**
     * @brief Explicitly unlock the mutex before destruction
     * 
     * Useful for early release when you need to unlock before the scope ends
     */
    void unlock() {
        if (mutex_ && locked_) {
            xSemaphoreGive(mutex_);
            locked_ = false;
        }
    }

    // Prevent copying to avoid double-unlock issues
    ScopedMutex(const ScopedMutex &) = delete;
    ScopedMutex &operator=(const ScopedMutex &) = delete;

    // Allow moving (useful for returning from functions)
    ScopedMutex(ScopedMutex &&other) noexcept 
        : mutex_(other.mutex_), locked_(other.locked_) {
        other.locked_ = false; // Prevent other from unlocking in destructor
    }

    ScopedMutex &operator=(ScopedMutex &&other) noexcept {
        if (this != &other) {
            // Unlock current mutex if locked
            if (mutex_ && locked_) {
                xSemaphoreGive(mutex_);
            }
            
            mutex_ = other.mutex_;
            locked_ = other.locked_;
            other.locked_ = false;
        }
        return *this;
    }
}; 