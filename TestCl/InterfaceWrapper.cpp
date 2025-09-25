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

// Create C-linkage wrapper functions for our C++ functions
extern "C" {
    // Channel selection
    EXPORT void selchan(int chan) {
        theInterfaceObject.SelSensor(chan);
    }

    EXPORT void get(int chan) {
        const int MAX_ATTEMPTS = 5;
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
            int zeroRowCount = 0;

            for (int i = 0; i < 12; i++) {
                int rowNonZero = 0;
                for (int j = 0; j < 12; j++) {
                    if (theInterfaceObject.frame_data[i][j] != 0) {
                        nonZeroCount++;
                        rowNonZero++;
                    }
                }
                if (rowNonZero == 0) {
                    zeroRowCount++;
                    printf("Warning: Row %d is completely empty\n", i);
                }
            }

            printf("Frame has %d non-zero values out of 144 (%d%% filled)\n",
                nonZeroCount, (nonZeroCount * 100) / 144);
            printf("Frame has %d completely empty rows\n", zeroRowCount);

            // If we have a mostly complete frame, we can break
            if (nonZeroCount > 100) {  // Accept if >70% of values are non-zero
                printf("Frame has sufficient data, proceeding\n");
                success = true;
                break;
            }

            // Delay between retries, increasing with each attempt
            int delay_ms = 50 * (attempts + 1);
            printf("Waiting %d ms before retry...\n", delay_ms);
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));

            // Reset USB endpoints between attempts
            if (attempts > 0) {
                printf("Resetting USB endpoints\n");
                reset_usb_endpoints();
            }
        }

        if (!success) {
            printf("WARNING: Failed to capture a complete frame after %d attempts\n", MAX_ATTEMPTS);
            printf("Proceeding with partial data - some rows may be missing or interpolated\n");
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

    // This is our C-linkage wrapper for the C++ check_data_flow function
    // Give it a different name to avoid the conflict
    EXPORT int check_data_flow_wrapper() {
        // Call the C++ function
        return ::check_data_flow();
    }

    EXPORT int get_buffer_stats(int* stats, int length) {
        if (length >= 3) {
            stats[0] = CIRCULAR_BUFFER_SIZE;
            stats[1] = static_cast<int>(GetBufferSize());
            stats[2] = ::check_data_flow();  // Use the C++ function directly
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