// Copyright 2023, All rights reserved

#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>

// Device identification constants
#define VENDOR_ID 0x0483
#define PRODUCT_ID 0x5750

// Forward declaration for HIDAPI type
struct hid_device_;
typedef struct hid_device_ hid_device;

/**
 * DirectUSB - Class for accessing USB HID devices
 * 
 * This class provides USB HID access:
 * - On Linux: Full functionality using HIDAPI/libusb
 * - On Windows: Compiles but provides dummy functionality
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
    bool GetNextReport(std::vector<uint8_t>& report, int timeout_ms = 1000);
    bool ResetDevice();
    
    // Device diagnostics
    void ListDevices();
    void DumpDeviceInfo();
    void PrintEndpointInfo();
    void DumpRawDescriptors();
    
    // Alternative access methods
    bool TryAlternativeAccess();
    
    // Asynchronous read functionality 
    void StartAsyncRead();
    void StopAsyncRead();
    
    // Logging control
    void SetVerboseLogging(bool enable) { m_VerboseLogging = enable; }
    void LogPacket(const char* prefix, const uint8_t* data, size_t length);
    
private:
    // Device handle
    hid_device* m_DeviceHandle;
    
    // Logging control
    bool m_VerboseLogging;
    
    // Thread management for async reads
    std::thread m_ReadThread;
    std::atomic<bool> m_Running;
    std::mutex m_QueueMutex;
    std::queue<std::vector<uint8_t>> m_DataQueue;
    
    // Read thread function
    void ReadThreadFunc();
};

// Global instance
extern DirectUSB g_DirectUSB;