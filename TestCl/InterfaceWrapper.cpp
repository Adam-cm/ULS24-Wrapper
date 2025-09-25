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

// Forward declaration of the internal C++ check_data_flow function
// Rename to cpp_check_data_flow to avoid conflict with the C function
namespace {
    int cpp_check_data_flow();  // This will be implemented by using the HidMgr.cpp version
}

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

// Implement our wrapper to call the C++ function
namespace {
    int cpp_check_data_flow() {
        // This will call the function from HidMgr.cpp
        return ::check_data_flow();
    }
}

extern "C" {
    // Channel selection
    EXPORT void selchan(int chan) {
        theInterfaceObject.SelSensor(chan);
    }

    // Frame capture with retries for improved reliability
    EXPORT void get(int chan) {
        const int MAX_ATTEMPTS = 5;  // Increase retry count
        bool success = false;

        printf("Starting capture with up to %d attempts\n", MAX_ATTEMPTS);

        for (int attempts = 0; attempts < MAX_ATTEMPTS; attempts++) {
            printf("Attempt %d of %d\n", attempts + 1, MAX_ATTEMPTS);

            // Call CaptureFrame12 with potential retries inside
            int result = theInterfaceObject.CaptureFrame12(chan);

            // Check if capture was successful (all 12 rows)
            if (result == 0) {
                printf("Capture successful on attempt %d\n", attempts + 1);
                success = true;
                break;
            }

            // Check if data looks valid even if not all rows were received
            bool hasGaps = false;
            int nonZeroCount = 0;

            for (int i = 0; i < 12; i++) {
                for (int j = 0; j < 12; j++) {
                    if (theInterfaceObject.frame_data[i][j] != 0) {
                        nonZeroCount++;
                    }
                }
            }

            printf("Frame has %d non-zero values out of 144\n", nonZeroCount);

            // If we have a mostly complete frame, we can break
            if (nonZeroCount > 100) {  // Accept if >70% of values are non-zero
                printf("Frame has sufficient data, proceeding\n");
                success = true;
                break;
            }

            // Delay between retries, increasing with each attempt
            std::this_thread::sleep_for(std::chrono::milliseconds(50 * (attempts + 1)));

            // Optional: Reset USB endpoints between attempts
            if (attempts > 0) {
                reset_usb_endpoints();
            }
        }

        if (!success) {
            printf("WARNING: Failed to capture a complete frame after %d attempts\n", MAX_ATTEMPTS);
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

    // Export the check_data_flow function correctly - calls our wrapper
    EXPORT int check_data_flow() {
        // Call our wrapper which calls the C++ function
        return cpp_check_data_flow();
    }

    EXPORT int get_buffer_stats(int* stats, int length) {
        if (length >= 3) {
            stats[0] = CIRCULAR_BUFFER_SIZE;
            stats[1] = static_cast<int>(GetBufferSize());
            stats[2] = cpp_check_data_flow();  // Use our wrapper
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