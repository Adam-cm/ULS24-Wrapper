// Copyright 2023, All rights reserved

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
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/hidraw.h>
#else
#include <windows.h>
#endif

#include "HidMgr.h"

// Global variables
bool g_DeviceDetected = false;
int Continue_Flag = 0;
bool ee_continue = false;
int chan_num = 1;  // Default to channel 1

hid_device* DeviceHandle = nullptr;

// Communication buffers
uint8_t RxData[RxNum] = { 0 };
uint8_t TxData[TxNum] = { 0 };

// Windows-specific globals
#ifdef _WIN32
HANDLE ReadHandle = INVALID_HANDLE_VALUE;
HANDLE hEventObject = NULL;
DWORD NumberOfBytesRead = 0;
OVERLAPPED HIDOverlapped = { 0 };
unsigned char InputReport[HIDREPORTNUM] = { 0 };
struct {
    ULONG InputReportByteLength;
} Capabilities = { 0 };
bool MyDeviceDetected = false;
#else
// Non-Windows placeholders
typedef int DWORD;
#define WAIT_OBJECT_0 0
#define WAIT_TIMEOUT 258
unsigned char InputReport[HIDREPORTNUM] = { 0 };
bool MyDeviceDetected = false;
#endif

// CircularBuffer implementation
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

// Buffer and synchronization objects
static CircularBuffer hid_report_buffer;
static std::mutex hid_buffer_mutex;
static std::condition_variable hid_buffer_cv;
static std::atomic<bool> hid_read_thread_running{ false };
static std::thread hid_read_thread;

// Thread function for continuous HID reading
static void HidReadThreadFunc() {
    // Pre-allocate vector to avoid allocation during read
    std::vector<uint8_t> report(RxNum);
    unsigned char InputReport[HIDREPORTNUM];

#ifdef __linux__
    // Set real-time priority for this thread
    struct sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) == 0) {
        printf("HID thread using real-time scheduling\n");
    }

    // Increase read aggressiveness on Linux
    int missed = 0;
    const int max_consec_missed = 3;
#endif

    while (hid_read_thread_running) {
        // Use minimal timeout for maximum throughput
        int res = hid_read_timeout(DeviceHandle, InputReport, HIDREPORTNUM, 0);
        if (res > 0) {
            // Got data - process it
            report.assign(InputReport + 1, InputReport + 1 + RxNum);

            // Store in buffer with minimal lock time
            {
                std::lock_guard<std::mutex> lock(hid_buffer_mutex);
                if (!hid_report_buffer.push(std::move(report))) {
                    fprintf(stderr, "Warning: HID buffer overflow!\n");
                }
                report = std::vector<uint8_t>(RxNum); // New buffer after move
            }
            hid_buffer_cv.notify_one();

#ifdef __linux__
            missed = 0; // Reset missed counter when we get data
#endif
        }
        else {
#ifdef __linux__
            // On Linux, adjust sleep based on whether we're getting data
            if (++missed > max_consec_missed) {
                // Only sleep if we've had several empty reads in a row
                std::this_thread::sleep_for(std::chrono::microseconds(50));
                missed = max_consec_missed;
            }
#else
            // On other platforms, use a fixed sleep
            std::this_thread::sleep_for(std::chrono::microseconds(50));
#endif
        }
    }
}

// Start the read thread
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

// Simple flow check - for monitoring
int check_data_flow() {
    static int last_size = 0;
    int current_size = 0;

    {
        std::lock_guard<std::mutex> lock(hid_buffer_mutex);
        current_size = static_cast<int>(hid_report_buffer.size());
    }

    int delta = current_size - last_size;
    last_size = current_size;

    return delta;
}

// Retrieve the next HID report (blocks until available)
bool GetNextHidReport(std::vector<uint8_t>& report) {
    std::unique_lock<std::mutex> lock(hid_buffer_mutex);
    hid_buffer_cv.wait(lock, [] {
        return !hid_report_buffer.empty() || !hid_read_thread_running;
        });

    if (!hid_report_buffer.empty()) {
        hid_report_buffer.pop(report);
        return true;
    }
    return false;
}

// Find and open the HID device
bool FindTheHID() {
    if (hid_init() != 0) {
        std::printf("hidapi init failed\n");
        return false;
    }

    DeviceHandle = hid_open(VENDOR_ID, PRODUCT_ID, nullptr);

    if (DeviceHandle) {
        std::printf("Device found!\n");

#ifdef __linux__
        // HIDAPI doesn't expose buffer size control directly
        // Instead, modify our internal buffer and read strategy

        // 1. Increase our internal circular buffer size
        std::printf("Using %d-entry circular buffer (%d KB)\n",
            CIRCULAR_BUFFER_SIZE,
            (CIRCULAR_BUFFER_SIZE * RxNum) / 1024);

        // 2. Find the hidraw path for diagnostics
        struct hid_device_info* devs = hid_enumerate(VENDOR_ID, PRODUCT_ID);
        if (devs) {
            std::string path = devs->path;
            hid_free_enumeration(devs);

            std::printf("Device path: %s\n", path.c_str());

            // Extract hidraw device name
            size_t pos = path.rfind('/');
            if (pos != std::string::npos) {
                std::string hidraw = path.substr(pos + 1);
                std::printf("HID device: %s\n", hidraw.c_str());

                // Print diagnostic info about the device
                std::printf("For maximum performance, you can create a udev rule:\n");
                std::printf("echo 'KERNEL==\"%s\", ATTR{power/control}=\"on\", "
                    "ATTR{device/power/wakeup}=\"enabled\"' > "
                    "/etc/udev/rules.d/99-hidraw-performance.rules\n",
                    hidraw.c_str());
            }
        }
#endif

        // Set non-blocking mode for maximum throughput
        hid_set_nonblocking(DeviceHandle, 1);

        // Start the read thread
        StartHidReadThread();
        g_DeviceDetected = true;
        return true;
    }
    else {
        std::printf("Device not found.\n");
        g_DeviceDetected = false;
        return false;
    }
}

// Close the HID device and stop thread
void CloseHandles() {
    StopHidReadThread();
    if (DeviceHandle) {
        hid_close(DeviceHandle);
        DeviceHandle = nullptr;
    }
    hid_exit();
    g_DeviceDetected = false;
}

// Modify these functions to fix the buffer overflows

// Write HID output report - single implementation to avoid ambiguity
bool WriteHIDOutputReport(int length) {
    if (!DeviceHandle) return false;
    
    // Increase buffer size to HIDREPORTNUM (65 bytes)
    unsigned char OutputReport[HIDREPORTNUM] = { 0 };
    OutputReport[0] = 0; // Report ID
    
    // Make sure we don't overflow the buffer
    std::memcpy(&OutputReport[1], TxData, std::min(static_cast<size_t>(TxNum), static_cast<size_t>(HIDREPORTNUM - 1)));
    
    int res = hid_write(DeviceHandle, OutputReport, length);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    if (res < 0) {
        printf("Write failed with error: %d\n", res);
        return false;
    }
    return true;
}

// Add this function to use the timeout feature of hidapi with proper buffer handling
bool ReadHIDInputReportTimeout(int length, int timeout_ms) {
    // Increase buffer size to HIDREPORTNUM (65 bytes)
    unsigned char InputReport[HIDREPORTNUM] = { 0 };
    
    // Use hid_read_timeout instead of hid_read to avoid indefinite blocking
    int res = hid_read_timeout(DeviceHandle, InputReport, length, timeout_ms);
    if (res > 0) {
        // Make sure we don't overflow the buffer
        std::memcpy(RxData, &InputReport[1], std::min(static_cast<size_t>(RxNum), static_cast<size_t>(HIDREPORTNUM - 1)));
        return true;
    }
    return false;
}

// Fix the ReadHIDInputReport function
void ReadHIDInputReport() {
#ifdef _WIN32
    // Windows implementation - same as before
    // ...
#else
    // Linux/non-Windows implementation - use hidapi directly
    // Increase buffer size to HIDREPORTNUM (65 bytes)
    unsigned char buffer[HIDREPORTNUM] = {0};
    int res = hid_read_timeout(DeviceHandle, buffer, HIDREPORTNUM, 1000);
    
    if (res > 0) {
        // Copy data from buffer to RxData (skipping report ID) with proper bounds checking
        std::memcpy(RxData, &buffer[1], std::min(static_cast<size_t>(RxNum), static_cast<size_t>(HIDREPORTNUM - 1)));
        
        // Process the data as needed
        uint8_t cmd = RxData[2];  // Command type 
        uint8_t type = RxData[4]; // Row type
        uint8_t row = RxData[5];  // Row number

        printf("Received packet: cmd=0x%02x type=0x%02x row=0x%02x\n", cmd, type, row);

        // Process based on command type
        if (cmd == 0x02) {  // Frame data command
            if ((type & 0x0F) == 0x02 || (type & 0x0F) == 0x22) {  // 12x12 frame
                // Get channel from the upper nibble of type
                chan_num = ((type & 0xF0) >> 4) + 1;

                // Check for end signal
                if (row == 0x0b || row == 0xf1) {
                    Continue_Flag = false;
                    printf("End signal detected: 0x%02x\n", row);
                }
                else {
                    Continue_Flag = true;
                }
            }
            else if ((type & 0x0F) == 0x08) {  // 24x24 frame
                if (row == 0x17) {
                    Continue_Flag = false;
                }
                else {
                    Continue_Flag = true;
                }
            }
        }
    }
    else if (res < 0) {
        printf("Error reading from device\n");
        CloseHandles();
        g_DeviceDetected = false;
        MyDeviceDetected = false;
    }
#endif
}