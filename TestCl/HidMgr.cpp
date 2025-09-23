#include <hidapi/hidapi.h>
#include <cstdio>
#include <cstring>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>
#include "HidMgr.h"

bool g_DeviceDetected = false;
int Continue_Flag = 0;
bool ee_continue = false;
int chan_num = 0;

// Vendor and Product IDs
#define VENDOR_ID  0x0483
#define PRODUCT_ID 0x5750

#define TxNum 64
#define RxNum 64
#define HIDREPORTNUM 65 // 1 byte report ID + 64 bytes data

// Global device handle
hid_device* DeviceHandle = nullptr;

// Buffers for communication (1st byte is report ID, usually 0)
uint8_t RxData[RxNum];
uint8_t TxData[TxNum];

// Threaded read queue and synchronization
static std::queue<std::vector<uint8_t>> hid_report_queue;
static std::mutex hid_queue_mutex;
static std::condition_variable hid_queue_cv;
static std::atomic<bool> hid_read_thread_running{ false };
static std::thread hid_read_thread;

// Dedicated read thread function
static void HidReadThreadFunc() {
    while (hid_read_thread_running) {
        unsigned char InputReport[HIDREPORTNUM] = { 0 };
        int res = hid_read(DeviceHandle, InputReport, HIDREPORTNUM);
        if (res > 0) {
            std::vector<uint8_t> report(InputReport + 1, InputReport + 1 + RxNum);
            {
                std::lock_guard<std::mutex> lock(hid_queue_mutex);
                hid_report_queue.push(std::move(report));
            }
            hid_queue_cv.notify_one();
        }
        // No sleep: read as fast as possible
    }
}

// Start the read thread
void StartHidReadThread() {
    if (!hid_read_thread_running) {
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

// Retrieve the next HID report (blocks until available)
bool GetNextHidReport(std::vector<uint8_t>& report) {
    std::unique_lock<std::mutex> lock(hid_queue_mutex);
    hid_queue_cv.wait(lock, [] { return !hid_report_queue.empty() || !hid_read_thread_running; });
    if (!hid_report_queue.empty()) {
        report = std::move(hid_report_queue.front());
        hid_report_queue.pop();
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

// Read HID input report from the queue (non-blocking)
// Returns true if a report was available, false otherwise
bool ReadHIDInputReportFromQueue()
{
    std::vector<uint8_t> report;
    {
        std::lock_guard<std::mutex> lock(hid_queue_mutex);
        if (hid_report_queue.empty())
            return false;
        report = std::move(hid_report_queue.front());
        hid_report_queue.pop();
    }
    if (report.size() == RxNum) {
        std::memcpy(RxData, report.data(), RxNum);
        return true;
    }
    return false;
}

// Blocking version: waits for a report
bool ReadHIDInputReportBlocking()
{
    std::vector<uint8_t> report;
    if (GetNextHidReport(report) && report.size() == RxNum) {
        std::memcpy(RxData, report.data(), RxNum);
        return true;
    }
    return false;
}

// C-style wrappers for compatibility
void WriteHIDOutputReport(void) { WriteHIDOutputReport(HIDREPORTNUM); }
void ReadHIDInputReport(void) { ReadHIDInputReportBlocking(); }