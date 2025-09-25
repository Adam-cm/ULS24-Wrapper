// Copyright 2023, All rights reserved

#pragma once

#include <cstdint>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <libusb-1.0/libusb.h>

// Device identification constants
#define VENDOR_ID 0x0483  // STMicroelectronics
#define PRODUCT_ID 0x5750 // Your product ID

class DirectUSB {
public:
    DirectUSB();
    ~DirectUSB();

    // Core functionality
    bool Initialize();
    bool IsConnected() const;
    bool SendReport(const uint8_t* data, size_t length);
    bool GetNextReport(std::vector<uint8_t>& report, int timeout_ms);
    
    // Asynchronous read functionality 
    void StartAsyncRead();
    void StopAsyncRead();

    // Diagnostic functions
    void SetVerboseLogging(bool enable) { m_VerboseLogging = enable; }
    void DumpDeviceInfo();
    void DumpRawDescriptors();
    void PrintEndpointInfo();
    
private:
    // libusb structures
    libusb_context* m_Context = nullptr;
    libusb_device_handle* m_DeviceHandle = nullptr;
    
    // Interface and endpoint information
    int m_Interface = 0;
    int m_InputEndpoint = 0x81;  // Default IN endpoint
    int m_OutputEndpoint = 0x01; // Default OUT endpoint

    // Thread management for async reads
    std::thread m_ReadThread;
    std::atomic<bool> m_Running;
    std::mutex m_QueueMutex;
    std::condition_variable m_DataAvailable;
    std::queue<std::vector<uint8_t>> m_DataQueue;
    
    // Read thread function
    void ReadThreadFunc();
    
    // Helper functions
    bool FindEndpoints();
    void LogPacket(const char* prefix, const uint8_t* data, size_t length);
    
    // Verbose logging flag
    bool m_VerboseLogging = true;
};

// Global instance for direct use
extern DirectUSB g_DirectUSB;