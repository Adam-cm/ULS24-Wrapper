#include <hidapi/hidapi.h>
#include <cstdio>
#include <cstring>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>
#include <array>

// Linux-specific headers - must be included outside of functions
#ifndef _WIN32
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <sys/resource.h>
#include <sys/mman.h>
#endif

#include "HidMgr.h"

bool g_DeviceDetected = false;
int Continue_Flag = 0;
bool ee_continue = false;
int chan_num = 0;

// REMOVE THESE - already defined in HidMgr.h
// #define VENDOR_ID  0x0483
// #define PRODUCT_ID 0x5750

// REMOVE THESE - already defined in HidMgr.h
// #define TxNum 64
// #define RxNum 64

// Global device handle (already declared in header)
// hid_device* DeviceHandle = nullptr;
// Just keep the definition:
hid_device* DeviceHandle = nullptr;

// Buffers for communication (1st byte is report ID, usually 0)
uint8_t RxData[RxNum];
uint8_t TxData[TxNum];

// REMOVE THIS - already defined in HidMgr.h
// #define CIRCULAR_BUFFER_SIZE 512

// Implementation of CircularBuffer methods
bool CircularBuffer::push(std::vector<uint8_t>&& report) {
    size_t next_head = (head + 1) % CIRCULAR_BUFFER_SIZE;
    if (next_head == tail) {
        return false; // Buffer full
    }
    buffer[head] = std::move(report);
    head = next_head;
    return true;
}

bool CircularBuffer::pop(std::vector<uint8_t>& report) {
    if (head == tail) {
        return false; // Buffer empty
    }
    report = std::move(buffer[tail]);
    tail = (tail + 1) % CIRCULAR_BUFFER_SIZE;
    return true;
}

bool CircularBuffer::empty() const {
    return head == tail;
}

size_t CircularBuffer::size() const {
    return (head >= tail) ? (head - tail) : (CIRCULAR_BUFFER_SIZE - tail + head);
}

// Create the buffer and sync objects
static CircularBuffer hid_report_buffer;
static std::mutex hid_buffer_mutex;
static std::condition_variable hid_buffer_cv;
static std::atomic<bool> hid_read_thread_running{ false };
static std::thread hid_read_thread;

// Add the check_data_flow implementation (already declared in header)
int check_data_flow() {
    int count = 0;
    while (ReadHIDInputReportFromQueue()) {
        count++;
    }
    return count;
}

// Dedicated read thread function - optimized for maximum throughput
static void HidReadThreadFunc() {
    // Pre-allocate vector to avoid allocation during read
    std::vector<uint8_t> report(RxNum);
    unsigned char InputReport[HIDREPORTNUM];

    while (hid_read_thread_running) {
        // Use non-blocking mode for maximum throughput
        int res = hid_read_timeout(DeviceHandle, InputReport, HIDREPORTNUM, 1);
        if (res > 0) {
            // Reuse pre-allocated vector with copy
            report.assign(InputReport + 1, InputReport + 1 + RxNum);

            // Use lock with minimal scope
            {
                std::lock_guard<std::mutex> lock(hid_buffer_mutex);
                if (!hid_report_buffer.push(std::move(report))) {
                    // Buffer full - very unlikely with 512-entry buffer
                    fprintf(stderr, "Warning: HID buffer overflow!\n");
                }
                // Allocate new vector since we moved the old one
                report = std::vector<uint8_t>(RxNum);
            }
            hid_buffer_cv.notify_one();
        }

        // Minimal sleep to avoid excessive CPU usage
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

// When starting the thread
void StartHidReadThread() {
    if (!hid_read_thread_running) {
        // On Pi, increase the non-blocking poll rate
        hid_set_nonblocking(DeviceHandle, 1);

        // Start the read thread
        hid_read_thread_running = true;
        hid_read_thread = std::thread(HidReadThreadFunc);
    }
}

// Stop the read thread and join
void StopHidReadThread() {
    if (hid_read_thread_running) {
        hid_read_thread_running = false;
        if (hid_read_thread.joinable())
            hid_read_thread.join();
    }
}

// Get current buffer size
size_t GetBufferSize() {
    std::lock_guard<std::mutex> lock(hid_buffer_mutex);
    return hid_report_buffer.size();
}

// Retrieve the next HID report (blocks until available)
bool GetNextHidReport(std::vector<uint8_t>& report) {
    std::unique_lock<std::mutex> lock(hid_buffer_mutex); // Use hid_buffer_mutex instead of hid_queue_mutex
    hid_buffer_cv.wait(lock, [] {
        return !hid_report_buffer.empty() || !hid_read_thread_running;
        });

    if (!hid_report_buffer.empty()) {
        hid_report_buffer.pop(report); // Use circular buffer pop instead of queue operations
        return true;
    }
    return false;
}

// Find and open the HID device
bool FindTheHID()
{
    if (hid_init() != 0) {
        std::printf("hidapi init failed\n");
        return false;
    }

    DeviceHandle = hid_open(VENDOR_ID, PRODUCT_ID, nullptr);
    hid_set_nonblocking(DeviceHandle, 0); // 0 = blocking mode
    if (DeviceHandle) {
        std::printf("Device found!\n");
        hid_set_nonblocking(DeviceHandle, 1); // 1 = non-blocking
        StartHidReadThread();
        return true;
    }
    else {
        std::printf("Device not found.\n");
        return false;
    }
}

// Close the HID device and stop thread
void CloseHandles()
{
    StopHidReadThread();
    if (DeviceHandle) {
        hid_close(DeviceHandle);
        DeviceHandle = nullptr;
    }
    hid_exit();
}

// Write HID output report
bool WriteHIDOutputReport(int length)
{
    unsigned char OutputReport[HIDREPORTNUM] = { 0 };
    OutputReport[0] = 0; // Report ID
    std::memcpy(&OutputReport[1], TxData, TxNum);
    int res = hid_write(DeviceHandle, OutputReport, length);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (res < 0) {
        // std::printf("Write failed: %ls\n", hid_error(DeviceHandle));
        return false;
    }
    return true;
}

// Non-blocking read from circular buffer
bool ReadHIDInputReportFromQueue() {
    std::vector<uint8_t> report;
    {
        std::lock_guard<std::mutex> lock(hid_buffer_mutex);
        if (!hid_report_buffer.pop(report)) {
            return false; // No data
        }
    }

    if (report.size() == RxNum) {
        std::memcpy(RxData, report.data(), RxNum);
        return true;
    }
    return false;
}

// Blocking read with timeout
bool ReadHIDInputReportBlocking(int timeout_ms) {
    std::vector<uint8_t> report;
    {
        std::unique_lock<std::mutex> lock(hid_buffer_mutex);
        bool success = hid_buffer_cv.wait_for(lock,
            std::chrono::milliseconds(timeout_ms),
            [] { return !hid_report_buffer.empty() || !hid_read_thread_running; });

        if (!success || !hid_report_buffer.pop(report)) {
            return false; // Timeout or thread stopped
        }
    }

    if (report.size() == RxNum) {
        std::memcpy(RxData, report.data(), RxNum);
        return true;
    }
    return false;
}

// Add this function to use the timeout feature of hidapi
bool ReadHIDInputReportTimeout(int length, int timeout_ms) {
    unsigned char InputReport[HIDREPORTNUM] = { 0 };
    // Use hid_read_timeout instead of hid_read to avoid indefinite blocking
    int res = hid_read_timeout(DeviceHandle, InputReport, length, timeout_ms);
    if (res > 0) {
        std::memcpy(RxData, &InputReport[1], RxNum);
        return true;
    }
    return false;
}

// C-style wrappers for compatibility
void WriteHIDOutputReport(void) { WriteHIDOutputReport(HIDREPORTNUM); }
void ReadHIDInputReport(void) { ReadHIDInputReportBlocking(); } // This is fine as it uses the default parameter