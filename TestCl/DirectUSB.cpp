// Copyright 2023, All rights reserved

#include "DirectUSB.h"
#include <cstdio>
#include <cstring>
#include <chrono>
#include <thread>

// Global instance
DirectUSB g_DirectUSB;

DirectUSB::DirectUSB() {
    m_DeviceHandle = nullptr;
    m_VerboseLogging = true;
}

DirectUSB::~DirectUSB() {
    if (m_DeviceHandle) {
        hid_close(m_DeviceHandle);
        m_DeviceHandle = nullptr;
    }
    
    // Clean up HIDAPI
    hid_exit();
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
        
        // Try alternative methods - by path
        struct hid_device_info *devs = hid_enumerate(VENDOR_ID, PRODUCT_ID);
        if (devs) {
            printf("Found device in enumeration, trying to open by path...\n");
            m_DeviceHandle = hid_open_path(devs->path);
            hid_free_enumeration(devs);
            
            if (!m_DeviceHandle) {
                printf("Failed to open device by path\n");
                return false;
            }
        } else {
            printf("Device not found in enumeration\n");
            return false;
        }
    }
    
    printf("Device opened successfully\n");
    
    // Set non-blocking mode
    if (hid_set_nonblocking(m_DeviceHandle, 1) != 0) {
        printf("Warning: Failed to set non-blocking mode\n");
    }
    
    printf("====== DEVICE INITIALIZED ======\n");
    return true;
}

bool DirectUSB::IsConnected() const {
    return m_DeviceHandle != nullptr;
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