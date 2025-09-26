// Copyright 2023, All rights reserved

#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>

// We'll use the standard Linux include path when on Linux
#ifdef __linux__
    #include <libusb-1.0/libusb.h>
#elif defined(_WIN32)
    // For Windows, we provide forward declarations to avoid dependency issues
    typedef struct libusb_context libusb_context;
    typedef struct libusb_device libusb_device;
    typedef struct libusb_device_handle libusb_device_handle;
    
    // libusb constants needed for DirectUSB
    #define LIBUSB_ENDPOINT_IN 0x80
    #define LIBUSB_REQUEST_TYPE_STANDARD (0x00 << 5)
    #define LIBUSB_RECIPIENT_DEVICE 0x00
    #define LIBUSB_REQUEST_GET_STATUS 0x00
    #define LIBUSB_ERROR_TIMEOUT -7
    #define LIBUSB_TRANSFER_TYPE_MASK 0x03
    #define LIBUSB_TRANSFER_TYPE_CONTROL 0
    #define LIBUSB_TRANSFER_TYPE_ISOCHRONOUS 1
    #define LIBUSB_TRANSFER_TYPE_BULK 2
    #define LIBUSB_TRANSFER_TYPE_INTERRUPT 3
    #define LIBUSB_OPTION_LOG_LEVEL 1
#endif

// Device identification constants
#define VENDOR_ID 0x0483  // STMicroelectronics
#define PRODUCT_ID 0x5750 // Your product ID

/**
 * DirectUSB - Class for directly accessing USB devices using libusb
 * 
 * This class provides a more direct and controllable interface to USB devices
 * compared to using HIDAPI alone. It can be used for diagnostic purposes,
 * low-level operations, and fallback access methods when HIDAPI has issues.
 */
class DirectUSB {
public:
    DirectUSB();
    ~DirectUSB();

    // Core functionality
    bool Initialize();
    bool IsConnected() const;
    bool SendReport(const uint8_t* data, size_t length);
    bool GetNextReport(std::vector<uint8_t>& report, int timeout_ms);
    
    // USB reset and control functionality
    bool ResetDevice();
    bool ResetEndpoints();
    bool ClearHalt(uint8_t endpoint);
    bool ControlTransfer(uint8_t requestType, uint8_t request, uint16_t value, uint16_t index, 
                        unsigned char* data, uint16_t length, unsigned int timeout);
    
    // Asynchronous read functionality 
    void StartAsyncRead();
    void StopAsyncRead();

    // Alternative access methods
    bool TryAlternativeMethods();
    bool TryAlternativeAccess();
    #ifdef __linux__
    bool TryRawAccess();
    #endif

    // Diagnostic functions
    void ListAllUSBDevices();
    void SetVerboseLogging(bool enable) { m_VerboseLogging = enable; }
    bool CheckLibusbAvailability();
    void DumpDeviceInfo();
    void DumpRawDescriptors();
    void PrintEndpointInfo();

    // Returns a string description of a libusb error code
    const char* GetErrorName(int error_code) const;
    
private:
    // libusb structures
    libusb_context* m_Context = nullptr;
    libusb_device_handle* m_DeviceHandle = nullptr;
    
    // Raw device access (Linux)
    int m_RawDevice = -1;
    std::string m_RawDevicePath;
    
    // Interface and endpoint information
    int m_Interface = 0;
    int m_InputEndpoint = 0x81;  // Default IN endpoint
    int m_OutputEndpoint = 0x01; // Default OUT endpoint
    int m_DefaultTimeout = 5000; // Default timeout (5 seconds)

    // Thread management for async reads
    std::thread m_ReadThread;
    std::atomic<bool> m_Running{false};
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