// Copyright 2023, All rights reserved

#include "InterfaceObj.h"
#include "HidMgr.h"
#include <cstdio>
#include <vector>
#include <thread>
#include <chrono>
#include <hidapi/hidapi.h>

// Platform-specific export macros
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif

// --- HIDAPI device handle and IDs for cross-platform use ---
#ifndef VENDOR_ID
#define VENDOR_ID 0x0683
#endif
#ifndef PRODUCT_ID
#define PRODUCT_ID 0x5850
#endif

// Use extern for DeviceHandle so it is shared with other files (if needed)
extern "C" hid_device* DeviceHandle;

// Buffer size constants (from HidMgr.h)
#define CIRCULAR_BUFFER_SIZE 1024

// Forward declarations for buffer/stat helpers
extern int GetBufferSize();
extern int check_data_flow();
extern bool FindTheHID();
extern bool Continue_Flag;

extern CInterfaceObject theInterfaceObject;
extern uint8_t RxData[RxNum];

// C++ linkage function - KEEP THIS OUTSIDE extern "C" block
int reset_usb_endpoints() {
    if (DeviceHandle) {
#ifdef _WIN32
        // Windows implementation with enhanced diagnostics
        printf("\n====== USB ENDPOINT RESET PROCEDURE STARTING (WINDOWS) ======\n");
        auto start_time = std::chrono::high_resolution_clock::now();
        
        struct hid_device_info* devs = hid_enumerate(VENDOR_ID, PRODUCT_ID);
        if (devs) {
            printf("Current device path: %s\n", devs->path);
            printf("  VID/PID: %04X:%04X\n", devs->vendor_id, devs->product_id);
            printf("  Manufacturer: %ls\n", devs->manufacturer_string ? devs->manufacturer_string : L"(unknown)");
            printf("  Product: %ls\n", devs->product_string ? devs->product_string : L"(unknown)");
            printf("  Serial: %ls\n", devs->serial_number ? devs->serial_number : L"(unknown)");
            printf("  Interface: %d\n", devs->interface_number);
            
            hid_free_enumeration(devs);
        } else {
            printf("WARNING: Could not enumerate devices - %ls\n", hid_error(NULL));
        }
        
        // On Windows, try to send a reset command first
        printf("\nSTEP 1: Sending device-specific reset command...\n");
        unsigned char reset_data[HIDREPORTNUM] = { 0 };
        reset_data[0] = 0;  // Report ID
        reset_data[1] = 0xaa;  // Preamble
        reset_data[2] = 0x01;  // Command type
        reset_data[3] = 0x10;  // Reset command
        
        int res = hid_write(DeviceHandle, reset_data, sizeof(reset_data));
        if (res >= 0) {
            printf("  Reset command sent successfully (%d bytes)\n", res);
            printf("  Waiting 100ms for device to process reset...\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            printf("  Failed to send reset command: %ls\n", hid_error(DeviceHandle));
        }
        
        printf("\nSTEP 2: Closing and reopening device with HIDAPI...\n");
        printf("  Closing HID device...\n");
        hid_close(DeviceHandle);
        DeviceHandle = nullptr;
        printf("  Waiting 200ms for device resources to be released...\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        printf("  Reopening HID device...\n");
        DeviceHandle = hid_open(VENDOR_ID, PRODUCT_ID, nullptr);
        if (DeviceHandle) {
            printf("  Successfully reopened HIDAPI device\n");
            if (hid_set_nonblocking(DeviceHandle, 1) == 0) {
                printf("  Set non-blocking mode successfully\n");
            } else {
                printf("  Failed to set non-blocking mode: %ls\n", hid_error(DeviceHandle));
            }
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            printf("\n====== USB ENDPOINT RESET COMPLETED SUCCESSFULLY ======\n");
            printf("Total reset time: %lld ms\n", duration.count());
            return 1;  // Success
        } else {
            printf("  Failed to reopen HIDAPI device on Windows: %ls\n", hid_error(NULL));
            printf("  Waiting 500ms before retrying...\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            printf("  Retrying device open...\n");
            DeviceHandle = hid_open(VENDOR_ID, PRODUCT_ID, nullptr);
            if (DeviceHandle) {
                printf("  Successfully reopened HIDAPI device on second attempt\n");
                if (hid_set_nonblocking(DeviceHandle, 1) == 0) {
                    printf("  Set non-blocking mode successfully\n");
                } else {
                    printf("  Failed to set non-blocking mode: %ls\n", hid_error(DeviceHandle));
                }
                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                printf("\n====== USB ENDPOINT RESET COMPLETED SUCCESSFULLY (SECOND ATTEMPT) ======\n");
                printf("Total reset time: %lld ms\n", duration.count());
                return 1;  // Success
            } else {
                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                printf("\n====== USB ENDPOINT RESET FAILED ======\n");
                printf("Total time spent attempting reset: %lld ms\n", duration.count());
                printf("Error: %ls\n", hid_error(NULL));
            }
        }
#else // Linux and others
        printf("\n====== USB ENDPOINT RESET PROCEDURE STARTING (LINUX) ======\n");
        auto start_time = std::chrono::high_resolution_clock::now();

        struct hid_device_info* devs = hid_enumerate(VENDOR_ID, PRODUCT_ID);
        if (devs) {
            printf("Current device path: %s\n", devs->path);
            printf("  VID/PID: %04X:%04X\n", devs->vendor_id, devs->product_id);
            printf("  Manufacturer: %ls\n", devs->manufacturer_string ? devs->manufacturer_string : L"(unknown)");
            printf("  Product: %ls\n", devs->product_string ? devs->product_string : L"(unknown)");
            printf("  Serial: %ls\n", devs->serial_number ? devs->serial_number : L"(unknown)");
            printf("  Interface: %d\n", devs->interface_number);
            hid_free_enumeration(devs);
        } else {
            printf("WARNING: Could not enumerate devices - %ls\n", hid_error(NULL));
        }

        printf("\nSTEP 1: Sending device-specific reset command...\n");
        unsigned char reset_data[HIDREPORTNUM] = { 0 };
        reset_data[0] = 0;  // Report ID
        reset_data[1] = 0xaa;  // Preamble
        reset_data[2] = 0x01;  // Command type
        reset_data[3] = 0x10;  // Reset command

        int res = hid_write(DeviceHandle, reset_data, sizeof(reset_data));
        if (res >= 0) {
            printf("  Reset command sent successfully (%d bytes)\n", res);
            printf("  Waiting 100ms for device to process reset...\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            unsigned char response[HIDREPORTNUM] = { 0 };
            res = hid_read_timeout(DeviceHandle, response, sizeof(response), 100);
            if (res > 0) {
                printf("  Received response after reset command (%d bytes):\n  ", res);
                for (int i = 0; i < std::min(res, 16); i++) {
                    printf("%02X ", response[i]);
                }
                printf("%s\n", res > 16 ? "..." : "");
            } else {
                printf("  No response received after reset command\n");
            }
        } else {
            printf("  Failed to send reset command: %ls\n", hid_error(DeviceHandle));
        }

        printf("\nSTEP 2: Closing and reopening device with HIDAPI...\n");
        printf("  Closing HID device...\n");
        hid_close(DeviceHandle);
        DeviceHandle = nullptr;
        printf("  Waiting 100ms for USB reset to complete...\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        printf("  Reopening HID device...\n");
        DeviceHandle = hid_open(VENDOR_ID, PRODUCT_ID, nullptr);
        if (DeviceHandle) {
            printf("  Successfully reopened HIDAPI device\n");
            if (hid_set_nonblocking(DeviceHandle, 1) == 0) {
                printf("  Set non-blocking mode successfully\n");
            } else {
                printf("  Failed to set non-blocking mode: %ls\n", hid_error(DeviceHandle));
            }
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            printf("\n====== USB ENDPOINT RESET COMPLETED SUCCESSFULLY ======\n");
            printf("Total reset time: %lld ms\n", duration.count());
            return 1;  // Success
        } else {
            printf("  Failed to reopen HIDAPI device: %ls\n", hid_error(NULL));
            printf("  Waiting 500ms before retrying...\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            printf("  Retrying device open...\n");
            DeviceHandle = hid_open(VENDOR_ID, PRODUCT_ID, nullptr);
            if (DeviceHandle) {
                printf("  Successfully reopened HIDAPI device on second attempt\n");
                if (hid_set_nonblocking(DeviceHandle, 1) == 0) {
                    printf("  Set non-blocking mode successfully\n");
                } else {
                    printf("  Failed to set non-blocking mode: %ls\n", hid_error(DeviceHandle));
                }
                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                printf("\n====== USB ENDPOINT RESET COMPLETED SUCCESSFULLY (SECOND ATTEMPT) ======\n");
                printf("Total reset time: %lld ms\n", duration.count());
                return 1;  // Success
            } else {
                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                printf("\n====== USB ENDPOINT RESET FAILED ======\n");
                printf("Total time spent attempting reset: %lld ms\n", duration.count());
            }
        }
#endif
    } else {
        printf("Cannot reset USB endpoints: No active device handle\n");
    }
    
    return 0;  // Not implemented or failed
}

// Create C-linkage wrapper functions for our C++ functions
extern "C" {
    EXPORT void selchan(int chan) {
        theInterfaceObject.SelSensor((BYTE)chan);
    }

    EXPORT void get(int chan) {
        const int MAX_ATTEMPTS = 5;
        bool success = false;
        printf("Starting capture with up to %d attempts\n", MAX_ATTEMPTS);
        for (int attempts = 0; attempts < MAX_ATTEMPTS; attempts++) {
            printf("Attempt %d of %d\n", attempts + 1, MAX_ATTEMPTS);
            int result = theInterfaceObject.CaptureFrame12((BYTE)chan);
            if (result == 0) {
                printf("Capture successful on attempt %d\n", attempts + 1);
                success = true;
                break;
            }
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
            printf("Frame has %d non-zero values out of 144 (%d%% filled)\n", nonZeroCount, (nonZeroCount * 100) / 144);
            printf("Frame has %d completely empty rows\n", zeroRowCount);
            if (nonZeroCount > 100) {
                printf("Frame has sufficient data, proceeding\n");
                success = true;
                break;
            }
            int delay_ms = 50 * (attempts + 1);
            printf("Waiting %d ms before retry...\n", delay_ms);
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
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

    EXPORT void get_frame12(int* outbuf) {
        for (int i = 0; i < 12; ++i) {
            for (int j = 0; j < 12; ++j) {
                outbuf[i * 12 + j] = theInterfaceObject.frame_data[i][j];
            }
        }
    }

    EXPORT void setinttime(float itime) {
        theInterfaceObject.SetIntTime(itime);
    }

    EXPORT void setgain(int gain) {
        theInterfaceObject.SetGainMode(gain);
    }

    EXPORT void reset() {
        FindTheHID();
    }

    EXPORT int get_buffer_capacity() {
        return CIRCULAR_BUFFER_SIZE;
    }

    EXPORT int get_buffer_used() {
        return GetBufferSize();
    }

    EXPORT int check_data_flow_wrapper() {
        return check_data_flow();
    }

    EXPORT int get_buffer_stats(int* stats, int length) {
        if (length >= 3) {
            stats[0] = CIRCULAR_BUFFER_SIZE;
            stats[1] = GetBufferSize();
            stats[2] = check_data_flow();
            return 3;
        }
        return 0;
    }

    EXPORT void cancel_capture() {
        if (DeviceHandle) {
            Continue_Flag = false;
        }
    }

    EXPORT void optimize_for_pi() {
#ifdef __linux__
        mlockall(MCL_CURRENT | MCL_FUTURE);
        setpriority(PRIO_PROCESS, 0, -20);
        printf("Applied Raspberry Pi optimizations\n");
#endif
    }
}