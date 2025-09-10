#pragma once

#include <cstdlib>
#include <type_traits>
#include "esp_heap_caps.h"


template <typename T> struct ModbusAllocator
{
    using value_type = T;
    using propagate_on_container_move_assignment = std::true_type;
    using is_always_equal = std::true_type;
    using pointer = T *;
    using const_pointer = const T *;

    ModbusAllocator() = default;

    template <typename U> constexpr ModbusAllocator(const ModbusAllocator<U> &) noexcept
    {
    }

    T *allocate(std::size_t n)
    {
        if (auto p = static_cast<T *>(heap_caps_malloc(n * sizeof(T), MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM)))
        {
            return p;
        }
        std::abort();
    }

    void deallocate(T *p, std::size_t) noexcept
    {
        free(p);
    }

    template <typename U> struct rebind
    {
        using other = ModbusAllocator<U>;
    };
};

template <typename T, typename U> bool operator==(const ModbusAllocator<T> &, const ModbusAllocator<U> &)
{
    return true;
}

template <typename T, typename U> bool operator!=(const ModbusAllocator<T> &, const ModbusAllocator<U> &)
{
    return false;
}
