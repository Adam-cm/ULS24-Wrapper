// Copyright 2023, All rights reserved

#pragma once

#include <cstdint>
#include <vector>
#include <string>

// Include HIDAPI header
#include <hidapi/hidapi.h>

// Device identification constants
#define VENDOR_ID 0x0483
#define PRODUCT_ID 0x5750

/**
 * DirectUSB - Minimal class for accessing USB HID devices using HIDAPI
 */
class DirectUSB {
public:
    DirectUSB();
    ~DirectUSB();

    // Core functionality
    bool Initialize();
    bool IsConnected() const;
    bool SendReport(const uint8_t* data, size_t length);
    bool GetReport(std::vector<uint8_t>& report, int timeout_ms = 1000);
    bool ResetDevice();
    
private:
    // Device handle
    hid_device* m_DeviceHandle;
};

// Global instance
extern DirectUSB g_DirectUSB;