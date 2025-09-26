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
#include <libusb-1.0/libusb.h>  // For direct libusb access
#include <errno.h>
#include <fcntl.h>  // For open()
#include <string.h> // For strerror()
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
    if (DeviceHandle) {
#ifdef __linux__
        printf("\n====== USB ENDPOINT RESET PROCEDURE STARTING ======\n");
        printf("Device handle: %p\n", (void*)DeviceHandle);
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Try to get the current device path for diagnostic purposes
        struct hid_device_info* devs = hid_enumerate(VENDOR_ID, PRODUCT_ID);
        const char* device_path = nullptr;
        if (devs) {
            device_path = devs->path;
            printf("Current device path: %s\n", device_path);
            
            // Try to get more device information
            printf("  VID/PID: %04X:%04X\n", devs->vendor_id, devs->product_id);
            printf("  Manufacturer: %ls\n", devs->manufacturer_string ? devs->manufacturer_string : L"(unknown)");
            printf("  Product: %ls\n", devs->product_string ? devs->product_string : L"(unknown)");
            printf("  Serial: %ls\n", devs->serial_number ? devs->serial_number : L"(unknown)");
            printf("  Interface: %d\n", devs->interface_number);
            
            // If we're using the libusb backend of hidapi, this will be useful
            if (devs->bus_type == HID_API_BUS_USB) {
                printf("  Bus type: USB\n");
            } else if (devs->bus_type == HID_API_BUS_BLUETOOTH) {
                printf("  Bus type: Bluetooth\n");
            } else if (devs->bus_type == HID_API_BUS_I2C) {
                printf("  Bus type: I2C\n");
            } else if (devs->bus_type == HID_API_BUS_SPI) {
                printf("  Bus type: SPI\n");
            } else {
                printf("  Bus type: Unknown (%d)\n", devs->bus_type);
            }
            
            // Don't free yet - we'll use this info later
        } else {
            printf("WARNING: Could not enumerate devices - %ls\n", hid_error(NULL));
        }
        
        // Try sending a special reset command first (device-specific)
        printf("\nSTEP 1: Sending device-specific reset command...\n");
        unsigned char reset_data[HIDREPORTNUM] = { 0 };
        reset_data[0] = 0;  // Report ID
        reset_data[1] = 0xaa;  // Preamble
        reset_data[2] = 0x01;  // Command type
        reset_data[3] = 0x10;  // Reset command
        
        // Try to send reset command
        int res = hid_write(DeviceHandle, reset_data, sizeof(reset_data));
        if (res >= 0) {
            printf("  Reset command sent successfully (%d bytes)\n", res);
            
            // Give device time to process reset
            printf("  Waiting 100ms for device to process reset...\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            // Try to read any response
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

        // Try direct USB reset using libusb if possible
        printf("\nSTEP 2: Attempting direct libusb reset...\n");
        bool libusb_reset_successful = false;
        
        // Only try libusb reset if we have the device path
        if (device_path) {
            // Extract bus/device number from the device path if possible
            // Path format is often like "/dev/bus/usb/XXX/YYY" or contains hidrawZ
            int bus_num = -1, dev_addr = -1;
            
            // Try to extract bus/device numbers from the path
            if (sscanf(device_path, "/dev/bus/usb/%d/%d", &bus_num, &dev_addr) == 2) {
                printf("  Extracted bus=%d, device=%d from path\n", bus_num, dev_addr);
            } else {
                // For hidraw devices, we need to look at the USB devices
                printf("  Could not extract bus/device from path, attempting to find device by VID/PID\n");
            }
            
            // Initialize libusb context
            libusb_context *context = NULL;
            if (libusb_init(&context) == 0) {
                printf("  Initialized libusb context\n");
                
                // Get device list
                libusb_device **list;
                ssize_t count = libusb_get_device_list(context, &list);
                
                printf("  Found %zd USB devices\n", count);
                
                libusb_device *target_device = NULL;
                for (ssize_t i = 0; i < count; i++) {
                    libusb_device *device = list[i];
                    struct libusb_device_descriptor desc;
                    
                    if (libusb_get_device_descriptor(device, &desc) == 0) {
                        // Check if this is our device by VID/PID
                        if (desc.idVendor == VENDOR_ID && desc.idProduct == PRODUCT_ID) {
                            // If bus/device were extracted, confirm they match
                            int this_bus = libusb_get_bus_number(device);
                            int this_addr = libusb_get_device_address(device);
                            
                            printf("  Found matching device - bus=%d, addr=%d\n", this_bus, this_addr);
                            
                            if (bus_num == -1 || (this_bus == bus_num && this_addr == dev_addr)) {
                                target_device = device;
                                printf("  Target device found\n");
                                break;
                            }
                        }
                    }
                }
                
                if (target_device) {
                    // Open the device
                    libusb_device_handle *handle = NULL;
                    if (libusb_open(target_device, &handle) == 0) {
                        printf("  Opened device with libusb\n");
                        
                        // Reset the device
                        int reset_result = libusb_reset_device(handle);
                        if (reset_result == 0) {
                            printf("  Successfully reset device using libusb\n");
                            libusb_reset_successful = true;
                        } else {
                            printf("  Failed to reset device: %s\n", libusb_error_name(reset_result));
                            
                            // Try individual endpoint reset
                            printf("  Attempting to clear halt on endpoints...\n");
                            
                            // Try standard HID endpoints
                            int in_ep = 0x81;  // IN endpoint (device to host)
                            int out_ep = 0x01; // OUT endpoint (host to device)
                            
                            if (libusb_clear_halt(handle, in_ep) == 0) {
                                printf("  Successfully cleared halt on IN endpoint 0x%02x\n", in_ep);
                                libusb_reset_successful = true;
                            } else {
                                printf("  Failed to clear halt on IN endpoint 0x%02x\n", in_ep);
                            }
                            
                            if (libusb_clear_halt(handle, out_ep) == 0) {
                                printf("  Successfully cleared halt on OUT endpoint 0x%02x\n", out_ep);
                                libusb_reset_successful = true;
                            } else {
                                printf("  Failed to clear halt on OUT endpoint 0x%02x\n", out_ep);
                            }
                        }
                        
                        // Close the device
                        libusb_close(handle);
                        printf("  Closed libusb device handle\n");
                    } else {
                        printf("  Failed to open device with libusb\n");
                    }
                } else {
                    printf("  Could not find target device in libusb device list\n");
                }
                
                // Free the list
                libusb_free_device_list(list, 1);
                
                // Exit libusb context
                libusb_exit(context);
                printf("  Cleaned up libusb context\n");
            } else {
                printf("  Failed to initialize libusb context\n");
            }
        } else {
            printf("  No device path available, skipping libusb direct reset\n");
        }
        
        // Now free the enumeration if we used it
        if (devs) {
            hid_free_enumeration(devs);
        }

        // STEP 3: Close and reopen with HIDAPI
        printf("\nSTEP 3: Closing and reopening device with HIDAPI...\n");
        
        // Close the device
        printf("  Closing HID device...\n");
        hid_close(DeviceHandle);
        DeviceHandle = nullptr;
        
        // Short delay to let the device settle
        printf("  Waiting 100ms for USB reset to complete...\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Reopen the device
        printf("  Reopening HID device...\n");
        DeviceHandle = hid_open(VENDOR_ID, PRODUCT_ID, nullptr);
        if (DeviceHandle) {
            printf("  Successfully reopened HIDAPI device\n");
            
            // Set non-blocking mode for maximum throughput
            if (hid_set_nonblocking(DeviceHandle, 1) == 0) {
                printf("  Set non-blocking mode successfully\n");
            } else {
                printf("  Failed to set non-blocking mode: %ls\n", hid_error(DeviceHandle));
            }
            
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            printf("\n====== USB ENDPOINT RESET COMPLETED SUCCESSFULLY ======\n");
            printf("Total reset time: %lld ms\n", duration.count());
            
            if (libusb_reset_successful) {
                printf("libusb direct reset was successful\n");
            }
            
            return 1;  // Success
        } else {
            printf("  Failed to reopen HIDAPI device: %ls\n", hid_error(NULL));
            
            // Try one more time with a longer delay
            printf("  Waiting 500ms before retrying...\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            printf("  Retrying device open...\n");
            DeviceHandle = hid_open(VENDOR_ID, PRODUCT_ID, nullptr);
            if (DeviceHandle) {
                printf("  Successfully reopened HIDAPI device on second attempt\n");
                
                // Set non-blocking mode for maximum throughput
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
                printf("  Failed to reopen HIDAPI device on second attempt: %ls\n", hid_error(NULL));
                
                // Try system commands as a last resort
                printf("\nSTEP 4: Attempting system-level USB reset...\n");
                
                // Try to unbind and rebind the driver using sysfs
                if (device_path && strstr(device_path, "hidraw")) {
                    // Extract hidraw number
                    int hidraw_num = -1;
                    if (sscanf(strstr(device_path, "hidraw"), "hidraw%d", &hidraw_num) == 1) {
                        printf("  Found hidraw device number: %d\n", hidraw_num);
                        
                        // Try to read the uevent file to get more info
                        char uevent_path[256];
                        snprintf(uevent_path, sizeof(uevent_path), "/sys/class/hidraw/hidraw%d/uevent", hidraw_num);
                        
                        FILE* uevent_file = fopen(uevent_path, "r");
                        if (uevent_file) {
                            char line[256];
                            printf("  Device uevent information:\n");
                            while (fgets(line, sizeof(line), uevent_file)) {
                                printf("    %s", line);
                            }
                            fclose(uevent_file);
                        } else {
                            printf("  Could not open uevent file: %s\n", strerror(errno));
                        }
                    }
                }
                
                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                printf("\n====== USB ENDPOINT RESET FAILED ======\n");
                printf("Total time spent attempting reset: %lld ms\n", duration.count());
            }
        }
#elif defined(_WIN32)
        // Windows implementation with enhanced diagnostics
        printf("\n====== USB ENDPOINT RESET PROCEDURE STARTING (WINDOWS) ======\n");
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Try to get the current device path for diagnostic purposes
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
        
        // Try to send reset command
        int res = hid_write(DeviceHandle, reset_data, sizeof(reset_data));
        if (res >= 0) {
            printf("  Reset command sent successfully (%d bytes)\n", res);
            
            // Give device time to process reset
            printf("  Waiting 100ms for device to process reset...\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
            printf("  Failed to send reset command: %ls\n", hid_error(DeviceHandle));
        }
        
        // Close the device
        printf("\nSTEP 2: Closing and reopening device with HIDAPI...\n");
        printf("  Closing HID device...\n");
        hid_close(DeviceHandle);
        DeviceHandle = nullptr;
        
        // Give the OS time to clean up the device handle
        printf("  Waiting 200ms for device resources to be released...\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // Reopen the device
        printf("  Reopening HID device...\n");
        DeviceHandle = hid_open(VENDOR_ID, PRODUCT_ID, nullptr);
        if (DeviceHandle) {
            printf("  Successfully reopened HIDAPI device\n");
            
            // Set non-blocking mode for maximum throughput
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
            
            // Try one more time with a longer delay
            printf("  Waiting 500ms before retrying...\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            printf("  Retrying device open...\n");
            DeviceHandle = hid_open(VENDOR_ID, PRODUCT_ID, nullptr);
            if (DeviceHandle) {
                printf("  Successfully reopened HIDAPI device on second attempt\n");
                
                // Set non-blocking mode for maximum throughput
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
#endif
    } else {
        printf("Cannot reset USB endpoints: No active device handle\n");
    }
    
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