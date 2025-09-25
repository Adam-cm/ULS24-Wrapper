#pragma once

#include <libusb-1.0/libusb.h>
#include <vector>
#include <cstdint>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <queue>

#define VENDOR_ID 0x0483
#define PRODUCT_ID 0x5750

// Interface and endpoint details for our device
#define USB_INTERFACE_NUMBER 0
#define USB_ENDPOINT_IN 0x81
#define USB_ENDPOINT_OUT 0x01
#define USB_TIMEOUT 1000

// DirectUSB handler class
class DirectUSB {
private:
    libusb_context* ctx = nullptr;
    libusb_device_handle* handle = nullptr;
    int active_config = 0;
    bool kernel_detached = false;
    
    // Thread for async reading
    std::atomic<bool> running{false};
    std::thread read_thread;
    
    // Buffer management
    static const size_t BUFFER_SIZE = 1024;
    std::mutex buffer_mutex;
    std::condition_variable buffer_cv;
    std::queue<std::vector<uint8_t>> data_queue;
    
    // Thread function
    void ReadThreadFunc();

public:
    DirectUSB();
    ~DirectUSB();
    
    bool Initialize();
    void Close();
    
    // Synchronous operations
    bool SendReport(const uint8_t* data, size_t length);
    bool ReceiveReport(uint8_t* data, size_t length, int timeout_ms = USB_TIMEOUT);
    
    // Asynchronous operations
    bool StartAsyncRead();
    void StopAsyncRead();
    bool GetNextReport(std::vector<uint8_t>& report, int timeout_ms = 100);
    
    // Utility
    bool IsConnected() const { return handle != nullptr; }
};

// Global instance
extern DirectUSB g_DirectUSB;