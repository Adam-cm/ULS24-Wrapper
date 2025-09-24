//#include "stdafx.h"
#include "InterfaceObj.h"
#include "HidMgr.h"
#include <cstdio>
#include <vector>
#include <thread>  // Add this
#include <chrono>  // Add this
#include <hidapi/hidapi.h> // Add this

// Linux-specific headers
#ifdef __linux__
#include <unistd.h>
#include <sys/resource.h>
#include <sys/mman.h>
#endif

extern CInterfaceObject theInterfaceObject;
extern uint8_t RxData[RxNum];

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif

extern "C" {

    // Remove the extern declarations like:
    // extern size_t GetBufferSize(); - already in HidMgr.h

    EXPORT int get_buffer_capacity() {
        return CIRCULAR_BUFFER_SIZE;
    }

    EXPORT int get_buffer_used() {
        return static_cast<int>(GetBufferSize());
    }

    EXPORT int get_buffer_stats(int* stats, int length) {
        if (length >= 3) {
            stats[0] = CIRCULAR_BUFFER_SIZE;
            stats[1] = static_cast<int>(GetBufferSize());
            stats[2] = check_data_flow();  // Use the function from HidMgr.h
            return 3;
        }
        return 0;
    }

    EXPORT void optimize_for_pi() {
    #ifdef __linux__
        // Set process priority
        nice(-20);  // Use nice() instead of setpriority()
        
        // If running as root, we can lock pages in memory
        if (geteuid() == 0) {
            mlockall(MCL_CURRENT | MCL_FUTURE);
        }
        
        // Disable CPU scaling to maintain consistent performance
        system("echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor >/dev/null 2>&1");
    #endif
        
        return;
    }

    int reset_usb_endpoints() {
#ifdef __linux__
        if (DeviceHandle) {
            // First close and reopen device
            hid_close(DeviceHandle);

            // Small delay to let USB settle
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            // Reopen
            DeviceHandle = hid_open(VENDOR_ID, PRODUCT_ID, nullptr);
            if (!DeviceHandle) {
                return 0;  // Failed to reopen
            }

            // Reset USB endpoints via sysfs if possible
            if (geteuid() == 0) {
                // Try to find and reset the USB endpoints
                system("for ep in /sys/bus/usb/devices/*/ep_*; do "
                    "  echo 0 > $ep/buffer_size; "
                    "  echo 512 > $ep/buffer_size; "
                    "done");
            }

            // Set non-blocking mode for our read thread
            hid_set_nonblocking(DeviceHandle, 1);
            return 1;  // Success
        }
#endif
        return 0;  // Not implemented or failed
    }

    EXPORT void cancel_capture() {
        Continue_Flag = false;
    }

    // New: Print all queued HID reports (non-blocking)
    EXPORT void print_hid_reports() {
        std::vector<uint8_t> report;
        while (ReadHIDInputReportFromQueue()) {
            // RxData is filled by ReadHIDInputReportFromQueue
            printf("Received report: ");
            for (int i = 0; i < RxNum; ++i) {
                printf("%02x ", RxData[i]);
            }
            printf("\n");
        }
    }
}