// Copyright 2014-2017, Anitoa Systems, LLC
// All rights reserved

#pragma once

#ifdef _WIN32
#include <wtypes.h>
#include <initguid.h>
extern "C" {
#include "hidsdi.h"
#include <setupapi.h>
#include <dbt.h>
}
bool DeviceNameMatch(LPARAM lParam);
#endif

#include <vector>
#include <array>
#include <atomic>

#define MAX_LOADSTRING 256

extern int Continue_Flag;

#define TxNum 64        // the number of the buffer for sent data to HID
#define RxNum 64        // the number of the buffer for received data from HID

#define HIDREPORTNUM (64+1)       //  HID report num bytes
#define HIDBUFSIZE 12

#define GetCmd      0x02        // return 0x02 command 
#define ReadCmd     0x04        // Read command

// Circular buffer size - 512 reports × 64 bytes = 32KB buffer
#define CIRCULAR_BUFFER_SIZE 512

// Circular buffer structure for HID reports
struct CircularBuffer {
    std::array<std::vector<uint8_t>, CIRCULAR_BUFFER_SIZE> buffer;
    std::atomic<size_t> head{ 0 };
    std::atomic<size_t> tail{ 0 };

    bool push(std::vector<uint8_t>&& report);
    bool pop(std::vector<uint8_t>& report);
    bool empty() const;
    size_t size() const;
};

// HID device management
bool FindTheHID();
void CloseHandles();

// HID threaded read management
void StartHidReadThread();
void StopHidReadThread();
bool GetNextHidReport(std::vector<uint8_t>& report);
bool ReadHIDInputReportFromQueue();
bool ReadHIDInputReportBlocking(int timeout_ms = 1000);
bool ReadHIDInputReportTimeout(int length, int timeout_ms = 1000);

// HID report I/O
bool WriteHIDOutputReport(int length);
void WriteHIDOutputReport(void);
void ReadHIDInputReport(void);

// Buffer management
size_t GetBufferSize();
int reset_usb_endpoints();

// (Legacy/Windows-specific, can be guarded or removed if not used on Linux)
void DisplayInputReport();
void DisplayReceivedData(char ReceivedByte);
void GetDeviceCapabilities();
void PrepareForOverlappedTransfer();
void ReadAndWriteToDevice();
void RegisterForDeviceNotifications();