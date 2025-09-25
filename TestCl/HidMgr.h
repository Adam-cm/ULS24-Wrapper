#pragma once

#include <hidapi/hidapi.h>
#include <vector>   // Missing include for std::vector
#include <cstdint>  // For uint8_t

// Constants for HID communication
#define HIDREPORTNUM 65   // Increase to 65 to avoid buffer overflow (64 + 1 report ID)
#define RxNum 64
#define TxNum 64

// Device identifiers
#define VENDOR_ID 0x04d8
#define PRODUCT_ID 0x003f

// Buffer size for circular queue
#define CIRCULAR_BUFFER_SIZE 256

// Handle to the HID device
extern hid_device* DeviceHandle;

// Data buffers for HID communication
extern uint8_t TxData[TxNum];
extern uint8_t RxData[RxNum];

// State variables
extern bool g_DeviceDetected;
extern int Continue_Flag;
extern bool ee_continue;
extern int chan_num;

// Circular buffer class definition
class CircularBuffer {
private:
    std::vector<uint8_t> buffer[CIRCULAR_BUFFER_SIZE];
    size_t head = 0;
    size_t tail = 0;

public:
    bool push(std::vector<uint8_t>&& report);
    bool pop(std::vector<uint8_t>& report);
    bool empty() const;
    size_t size() const;
};

// Core HID functions
bool FindTheHID();
void CloseHandles();

// SINGLE WriteHIDOutputReport function with default parameter
bool WriteHIDOutputReport(int length = HIDREPORTNUM);

// Read operations
void ReadHIDInputReport();
bool ReadHIDInputReportFromQueue();
bool ReadHIDInputReportBlocking(int timeout_ms);
bool ReadHIDInputReportTimeout(int length, int timeout_ms);

// Thread management functions
void StartHidReadThread();
void StopHidReadThread();
size_t GetBufferSize();
bool GetNextHidReport(std::vector<uint8_t>& report);
int check_data_flow();