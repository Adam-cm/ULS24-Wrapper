// Copyright 2023, All rights reserved

#include "InterfaceObj.h"
#include "HidMgr.h"
#include <cstdio>
#include <vector>
#include <thread>
#include <chrono>
#include <hidapi/hidapi.h>

// Linux-specific headers
#ifdef __linux__
#include <unistd.h>
#include <sys/resource.h>
#include <sys/mman.h>
#endif

// External references
extern CInterfaceObject theInterfaceObject;
extern uint8_t RxData[RxNum];

// Platform-specific export macros
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif

// Forward declaration of the internal check_data_flow function
extern int check_data_flow();

// C++ linkage function - KEEP THIS OUTSIDE extern "C" block
int reset_usb_endpoints() {
#ifdef __linux__
    if (DeviceHandle) {
        // Implementation for Linux could go here
        return 1;  // Success
    }
#endif
    return 0;  // Not implemented or failed
}

extern "C" {
    // Channel selection
    EXPORT void selchan(int chan) {
        theInterfaceObject.SelSensor(chan);
    }

    // Frame capture with retries for improved reliability
    EXPORT void get(int chan) {
        // Try multiple times if needed
        for (int attempts = 0; attempts < 3; attempts++) {
            theInterfaceObject.CaptureFrame12(chan);

            // Check if data looks valid
            bool hasGaps = false;
            for (int i = 1; i < 12; i += 2) {
                if (theInterfaceObject.frame_data[0][i] == 0) {
                    hasGaps = true;
                    break;
                }
            }

            if (!hasGaps) break;

            // Small delay between retries
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    // Copy frame data to output buffer
    EXPORT void get_frame12(int* outbuf) {
        for (int i = 0; i < 12; ++i) {
            for (int j = 0; j < 12; ++j) {
                outbuf[i * 12 + j] = theInterfaceObject.frame_data[i][j];
            }
        }
    }

    // Set integration time
    EXPORT void setinttime(float itime) {
        theInterfaceObject.SetIntTime(itime);
    }

    // Set gain mode
    EXPORT void setgain(int gain) {
        theInterfaceObject.SetGainMode(gain);
    }

    // Reset device connection
    EXPORT void reset() {
        FindTheHID();
    }

    // Buffer-related functions
    EXPORT int get_buffer_capacity() {
        return CIRCULAR_BUFFER_SIZE;
    }

    EXPORT int get_buffer_used() {
        return static_cast<int>(GetBufferSize());
    }

    // Export the check_data_flow function correctly
    EXPORT int check_data_flow() {
        // Call the function directly - no scope qualifier needed
        return ::check_data_flow();
    }

    EXPORT int get_buffer_stats(int* stats, int length) {
        if (length >= 3) {
            stats[0] = CIRCULAR_BUFFER_SIZE;
            stats[1] = static_cast<int>(GetBufferSize());
            stats[2] = ::check_data_flow();  // Use scope qualifier here too
            return 3;
        }
        return 0;
    }

    // Add this function to cancel captures
    EXPORT void cancel_capture() {
        if (DeviceHandle) {
            Continue_Flag = false;
        }
    }

    // Add this function for Raspberry Pi optimization
    EXPORT void optimize_for_pi() {
#ifdef __linux__
        // Try to lock memory to prevent paging
        mlockall(MCL_CURRENT | MCL_FUTURE);

        // Set process priority
        setpriority(PRIO_PROCESS, 0, -20);

        printf("Applied Raspberry Pi optimizations\n");
#endif
    }
}