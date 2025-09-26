// Copyright 2023, All rights reserved

#include "DirectUSB.h"
#include <cstdio>
#include <cstring>
#include <thread>
#include <chrono>

// Global instance
DirectUSB g_DirectUSB;

// Base constructor/destructor for both platforms
DirectUSB::DirectUSB() : m_Running(false) {
    m_DeviceHandle = nullptr;
    m_VerboseLogging = true;
}

DirectUSB::~DirectUSB() {
    StopAsyncRead();
    
    #ifdef __linux__
    // Linux-specific cleanup
    if (m_DeviceHandle) {
        // This will be replaced with actual HIDAPI call on Linux
        // hid_close(m_DeviceHandle);
        m_DeviceHandle = nullptr;
    }
    // hid_exit();
    #endif
}

// Core functionality implementations - all dummy implementations for Windows

bool DirectUSB::Initialize() {
    #ifdef __linux__
    // The Linux version will contain real implementation
    printf("DirectUSB::Initialize - Full implementation will be used on Linux\n");
    #else
    // Windows implementation
    printf("DirectUSB::Initialize - Windows dummy implementation\n");
    #endif
    return false;
}

bool DirectUSB::IsConnected() const {
    return m_DeviceHandle != nullptr;
}

bool DirectUSB::SendReport(const uint8_t* data, size_t length) {
    #ifdef __linux__
    // The Linux version will contain real implementation
    printf("DirectUSB::SendReport - Full implementation will be used on Linux\n");
    #else
    // Windows implementation
    printf("DirectUSB::SendReport - Windows dummy implementation\n");
    #endif
    return false;
}

bool DirectUSB::GetReport(std::vector<uint8_t>& report, int timeout_ms) {
    #ifdef __linux__
    // The Linux version will contain real implementation
    printf("DirectUSB::GetReport - Full implementation will be used on Linux\n");
    #else
    // Windows implementation
    printf("DirectUSB::GetReport - Windows dummy implementation\n");
    #endif
    return false;
}

bool DirectUSB::GetNextReport(std::vector<uint8_t>& report, int timeout_ms) {
    #ifdef __linux__
    // The Linux version will contain real implementation
    printf("DirectUSB::GetNextReport - Full implementation will be used on Linux\n");
    #else
    // Windows implementation
    printf("DirectUSB::GetNextReport - Windows dummy implementation\n");
    #endif
    return false;
}

bool DirectUSB::ResetDevice() {
    #ifdef __linux__
    // The Linux version will contain real implementation
    printf("DirectUSB::ResetDevice - Full implementation will be used on Linux\n");
    #else
    // Windows implementation
    printf("DirectUSB::ResetDevice - Windows dummy implementation\n");
    #endif
    return false;
}

void DirectUSB::ListDevices() {
    #ifdef __linux__
    // The Linux version will contain real implementation
    printf("DirectUSB::ListDevices - Full implementation will be used on Linux\n");
    #else
    // Windows implementation
    printf("DirectUSB::ListDevices - Windows dummy implementation\n");
    #endif
}

void DirectUSB::DumpDeviceInfo() {
    #ifdef __linux__
    // The Linux version will contain real implementation
    printf("DirectUSB::DumpDeviceInfo - Full implementation will be used on Linux\n");
    #else
    // Windows implementation
    printf("DirectUSB::DumpDeviceInfo - Windows dummy implementation\n");
    #endif
}

void DirectUSB::PrintEndpointInfo() {
    #ifdef __linux__
    // The Linux version will contain real implementation
    printf("DirectUSB::PrintEndpointInfo - Full implementation will be used on Linux\n");
    #else
    // Windows implementation
    printf("DirectUSB::PrintEndpointInfo - Windows dummy implementation\n");
    #endif
}

void DirectUSB::DumpRawDescriptors() {
    #ifdef __linux__
    // The Linux version will contain real implementation
    printf("DirectUSB::DumpRawDescriptors - Full implementation will be used on Linux\n");
    #else
    // Windows implementation
    printf("DirectUSB::DumpRawDescriptors - Windows dummy implementation\n");
    #endif
}

bool DirectUSB::TryAlternativeAccess() {
    #ifdef __linux__
    // The Linux version will contain real implementation
    printf("DirectUSB::TryAlternativeAccess - Full implementation will be used on Linux\n");
    #else
    // Windows implementation
    printf("DirectUSB::TryAlternativeAccess - Windows dummy implementation\n");
    #endif
    return false;
}

void DirectUSB::LogPacket(const char* prefix, const uint8_t* data, size_t length) {
    #ifdef __linux__
    // The Linux version will contain real implementation
    printf("DirectUSB::LogPacket - Full implementation will be used on Linux\n");
    #else
    // Windows implementation
    printf("%s: %zu bytes (Windows dummy implementation)\n", prefix, length);
    #endif
}

void DirectUSB::StartAsyncRead() {
    #ifdef __linux__
    // The Linux version will contain real implementation
    printf("DirectUSB::StartAsyncRead - Full implementation will be used on Linux\n");
    #else
    // Windows implementation
    printf("DirectUSB::StartAsyncRead - Windows dummy implementation\n");
    #endif
}

void DirectUSB::StopAsyncRead() {
    #ifdef __linux__
    // The Linux version will contain real implementation
    printf("DirectUSB::StopAsyncRead - Full implementation will be used on Linux\n");
    #else
    // Windows implementation
    printf("DirectUSB::StopAsyncRead - Windows dummy implementation\n");
    #endif
}

void DirectUSB::ReadThreadFunc() {
    #ifdef __linux__
    // The Linux version will contain real implementation
    printf("DirectUSB::ReadThreadFunc - Full implementation will be used on Linux\n");
    #else
    // Windows implementation
    printf("DirectUSB::ReadThreadFunc - Windows dummy implementation\n");
    #endif
}

// LINUX IMPLEMENTATION: 
// The code below represents the Linux implementation which will be added
// to the Linux build file. It's commented out here for Windows compilation.

/*
// Linux implementation - will be uncommented in the Linux build

#include <hidapi/hidapi.h>

bool DirectUSB::Initialize() {
    printf("\n====== INITIALIZING USB DEVICE ======\n");
    
    // Initialize HIDAPI
    if (hid_init() != 0) {
        printf("Failed to initialize HIDAPI\n");
        return false;
    }
    
    // List devices to help with debugging
    ListDevices();
    
    // Try to open our target device
    printf("Attempting to open device with VID=0x%04X, PID=0x%04X\n", VENDOR_ID, PRODUCT_ID);
    
    m_DeviceHandle = hid_open(VENDOR_ID, PRODUCT_ID, NULL);
    if (!m_DeviceHandle) {
        printf("Failed to open device: %ls\n", hid_error(NULL));
        return TryAlternativeAccess();
    }
    
    printf("Device opened successfully\n");
    
    // Set non-blocking mode
    if (hid_set_nonblocking(m_DeviceHandle, 1) != 0) {
        printf("Warning: Failed to set non-blocking mode\n");
    }
    
    printf("====== DEVICE INITIALIZED ======\n");
    return true;
}

void DirectUSB::ListDevices() {
    printf("\n====== LISTING USB HID DEVICES ======\n");
    
    // Initialize HIDAPI if needed
    if (hid_init() != 0) {
        printf("Failed to initialize HIDAPI\n");
        return;
    }
    
    // Enumerate all devices
    struct hid_device_info *devs = hid_enumerate(0x0, 0x0);  // All devices
    if (!devs) {
        printf("No HID devices found or enumeration failed\n");
        return;
    }
    
    // Print all devices
    struct hid_device_info *cur_dev = devs;
    int count = 0;
    
    while (cur_dev) {
        printf("Device %d:\n", count++);
        printf("  VID: 0x%04hX, PID: 0x%04hX\n", cur_dev->vendor_id, cur_dev->product_id);
        printf("  Path: %s\n", cur_dev->path ? cur_dev->path : "(null)");
        
        // Print strings with null checks
        if (cur_dev->serial_number)
            printf("  Serial: %ls\n", cur_dev->serial_number);
        else
            printf("  Serial: (null)\n");
            
        if (cur_dev->manufacturer_string)
            printf("  Manufacturer: %ls\n", cur_dev->manufacturer_string);
        else
            printf("  Manufacturer: (null)\n");
            
        if (cur_dev->product_string)
            printf("  Product: %ls\n", cur_dev->product_string);
        else
            printf("  Product: (null)\n");
            
        printf("  Interface: %d\n", cur_dev->interface_number);
        
        // Highlight our target device
        if (cur_dev->vendor_id == VENDOR_ID && cur_dev->product_id == PRODUCT_ID) {
            printf("  *** THIS IS OUR TARGET DEVICE ***\n");
        }
        
        printf("\n");
        cur_dev = cur_dev->next;
    }
    
    hid_free_enumeration(devs);
    printf("====== END OF USB DEVICE LIST ======\n\n");
}

bool DirectUSB::TryAlternativeAccess() {
    printf("\n====== TRYING ALTERNATIVE ACCESS METHODS ======\n");
    
    // Try looking for the device by path
    struct hid_device_info *devs = hid_enumerate(VENDOR_ID, PRODUCT_ID);
    if (devs) {
        printf("Found matching device(s) with VID=0x%04X, PID=0x%04X\n", VENDOR_ID, PRODUCT_ID);
        
        struct hid_device_info *cur_dev = devs;
        while (cur_dev) {
            printf("Attempting to open device path: %s\n", cur_dev->path);
            m_DeviceHandle = hid_open_path(cur_dev->path);
            
            if (m_DeviceHandle) {
                printf("Successfully opened device by path\n");
                hid_free_enumeration(devs);
                
                // Set non-blocking mode
                if (hid_set_nonblocking(m_DeviceHandle, 1) != 0) {
                    printf("Warning: Failed to set non-blocking mode\n");
                }
                
                return true;
            }
            
            cur_dev = cur_dev->next;
        }
        
        hid_free_enumeration(devs);
    }
    
    printf("All alternative methods failed\n");
    return false;
}

bool DirectUSB::SendReport(const uint8_t* data, size_t length) {
    if (!m_DeviceHandle) {
        printf("Cannot send report: No active device connection\n");
        return false;
    }
    
    // Log the outgoing data if verbose logging is enabled
    if (m_VerboseLogging) {
        LogPacket("SENDING", data, length);
    }
    
    // Send data using HIDAPI
    int res = hid_write(m_DeviceHandle, data, length);
    
    if (res < 0) {
        printf("Failed to write to device: %ls\n", hid_error(m_DeviceHandle));
        return false;
    }
    
    if (m_VerboseLogging) {
        printf("Successfully sent %d bytes\n", res);
    }
    
    return true;
}

bool DirectUSB::GetReport(std::vector<uint8_t>& report, int timeout_ms) {
    if (!m_DeviceHandle) {
        printf("Cannot get report: No active device connection\n");
        return false;
    }
    
    // Initialize buffer for report
    report.resize(64);  // Common HID report size
    
    // Read with timeout
    int res = hid_read_timeout(m_DeviceHandle, report.data(), report.size(), timeout_ms);
    
    if (res < 0) {
        printf("Error reading from device: %ls\n", hid_error(m_DeviceHandle));
        return false;
    } else if (res == 0) {
        // Timeout - not an error, but no data
        return false;
    }
    
    // Resize to actual data received
    report.resize(res);
    
    // Log the received data if verbose logging is enabled
    if (m_VerboseLogging) {
        LogPacket("RECEIVED", report.data(), report.size());
    }
    
    return true;
}

// GetNextReport - Same as GetReport but can also pull from async queue
bool DirectUSB::GetNextReport(std::vector<uint8_t>& report, int timeout_ms) {
    // Check for data in the queue first (from async thread)
    if (m_Running) {
        std::lock_guard<std::mutex> lock(m_QueueMutex);
        if (!m_DataQueue.empty()) {
            report = std::move(m_DataQueue.front());
            m_DataQueue.pop();
            return true;
        }
    }
    
    // If no data in queue, just call GetReport
    return GetReport(report, timeout_ms);
}

bool DirectUSB::ResetDevice() {
    if (!m_DeviceHandle) {
        printf("Cannot reset device: No active device connection\n");
        return false;
    }
    
    printf("Attempting to reset device...\n");
    
    // Close and reopen to perform a soft reset
    hid_close(m_DeviceHandle);
    m_DeviceHandle = nullptr;
    
    // Short delay to let the device settle
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Try to reopen
    m_DeviceHandle = hid_open(VENDOR_ID, PRODUCT_ID, NULL);
    if (m_DeviceHandle) {
        printf("Device reset successful\n");
        
        // Re-enable non-blocking mode
        hid_set_nonblocking(m_DeviceHandle, 1);
        return true;
    }
    
    // Try one more time with a longer delay
    printf("Initial reset failed, trying again with longer delay...\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    m_DeviceHandle = hid_open(VENDOR_ID, PRODUCT_ID, NULL);
    if (m_DeviceHandle) {
        printf("Device reset successful on second attempt\n");
        hid_set_nonblocking(m_DeviceHandle, 1);
        return true;
    }
    
    printf("Device reset failed\n");
    return false;
}

void DirectUSB::DumpDeviceInfo() {
    if (!m_DeviceHandle) {
        printf("No device handle available\n");
        return;
    }
    
    printf("\n==== DEVICE INFORMATION ====\n");
    
    // Get device information using HIDAPI
    struct hid_device_info *devs = hid_enumerate(VENDOR_ID, PRODUCT_ID);
    if (devs) {
        printf("Device Information:\n");
        printf("  VID: 0x%04hX, PID: 0x%04hX\n", devs->vendor_id, devs->product_id);
        printf("  Path: %s\n", devs->path ? devs->path : "(null)");
        
        if (devs->serial_number) {
            printf("  Serial Number: %ls\n", devs->serial_number);
        } else {
            printf("  Serial Number: (null)\n");
        }
        
        if (devs->manufacturer_string) {
            printf("  Manufacturer: %ls\n", devs->manufacturer_string);
        } else {
            printf("  Manufacturer: (null)\n");
        }
        
        if (devs->product_string) {
            printf("  Product: %ls\n", devs->product_string);
        } else {
            printf("  Product: (null)\n");
        }
        
        printf("  Interface: %d\n", devs->interface_number);
        
        hid_free_enumeration(devs);
    } else {
        printf("Could not get device information\n");
    }
    
    printf("\n");
}

void DirectUSB::PrintEndpointInfo() {
    printf("\n==== USB ENDPOINT INFORMATION ====\n");
    printf("Note: Detailed endpoint information not available through HIDAPI.\n");
    printf("Using standard HIDAPI endpoints for device communication.\n");
    
    if (m_DeviceHandle) {
        printf("Device is connected and active.\n");
    } else {
        printf("No active device handle.\n");
    }
    printf("\n");
}

void DirectUSB::DumpRawDescriptors() {
    printf("\n==== USB RAW DESCRIPTOR INFORMATION ====\n");
    printf("Note: Raw descriptor access is not available through HIDAPI.\n");
    printf("This is a simplified implementation for Linux compatibility.\n");
    
    if (m_DeviceHandle) {
        printf("Device is connected with VID=0x%04X, PID=0x%04X.\n", VENDOR_ID, PRODUCT_ID);
    } else {
        printf("No active device handle.\n");
    }
    printf("\n");
}

void DirectUSB::LogPacket(const char* prefix, const uint8_t* data, size_t length) {
    printf("%s (%zu bytes): ", prefix, length);
    
    // Limit the output to a reasonable number of bytes
    const size_t display_limit = 32;
    size_t display_bytes = length > display_limit ? display_limit : length;
    
    for (size_t i = 0; i < display_bytes; i++) {
        printf("%02X ", data[i]);
        
        // Add a newline every 16 bytes for readability
        if ((i + 1) % 16 == 0 && i < display_bytes - 1) {
            printf("\n                  ");
        }
    }
    
    if (length > display_limit) {
        printf("... (%zu more bytes)", length - display_limit);
    }
    
    printf("\n");
}

void DirectUSB::StartAsyncRead() {
    if (m_Running) return;
    
    printf("Starting asynchronous read thread...\n");
    
    m_Running = true;
    m_ReadThread = std::thread(&DirectUSB::ReadThreadFunc, this);
}

void DirectUSB::StopAsyncRead() {
    if (!m_Running) return;
    
    printf("Stopping asynchronous read thread...\n");
    
    m_Running = false;
    
    if (m_ReadThread.joinable()) {
        m_ReadThread.join();
    }
    
    printf("Asynchronous read thread stopped\n");
}

void DirectUSB::ReadThreadFunc() {
    printf("Async read thread started\n");
    
    std::vector<uint8_t> report;
    
    while (m_Running) {
        // Try to get a report
        if (GetReport(report, 100)) {
            // Add to queue if successful
            std::lock_guard<std::mutex> lock(m_QueueMutex);
            
            // Limit queue size to prevent memory issues
            if (m_DataQueue.size() < 100) {
                m_DataQueue.push(report);
            }
        }
        
        // Don't hog CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    printf("Async read thread exiting\n");
}