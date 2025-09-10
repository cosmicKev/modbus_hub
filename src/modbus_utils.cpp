#include "modbus_utils.h"   
#include <esp_sntp.h> 
#include <ctime>
#include <esp_netif.h>

#define EPOCH_TIME_2025_01_01 1735689600

uint64_t modbus_utils_get_time()
{
    sntp_sync_status_t status  =sntp_get_sync_status();
    time_t now;
    time(&now);
    if((uint64_t) now > EPOCH_TIME_2025_01_01)
    {
        return now;
    }
    return 0;
}

bool modbus_utils_has_valid_ip_address()
{
    esp_netif_t *netif = nullptr;
    esp_netif_ip_info_t ip_info;
    
    // Get the first network interface
    bool valid_ip = false;
    do{
        // Safe because we dont remove interface.
        netif = esp_netif_next_unsafe(nullptr);
        esp_netif_get_ip_info(netif, &ip_info);
        if (ip_info.ip.addr != 0)
        {
            return true;
        }   
    }while(netif!=nullptr);
    return false;
}
