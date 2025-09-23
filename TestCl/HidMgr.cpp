#include <hidapi/hidapi.h>
#include <cstdio>
#include <cstring>
#include <thread>
#include <chrono>
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
//#define HIDREPORTNUM 65 // 1 byte report ID + 64 bytes data

// Global device handle
hid_device* DeviceHandle = nullptr;

// Buffers for communication (1st byte is report ID, usually 0)
uint8_t RxData[RxNum];
uint8_t TxData[TxNum];

// Find and open the HID device
bool FindTheHID()
{
    if (hid_init() != 0) {
        std::printf("hidapi init failed\n");
        return false;
    }

    DeviceHandle = hid_open(VENDOR_ID, PRODUCT_ID, nullptr);
    if (DeviceHandle) {
        std::printf("Device found!\n");
        hid_set_nonblocking(DeviceHandle, 1); // 1 = non-blocking
        return true;
    }
    else {
        std::printf("Device not found.\n");
        return false;
    }
}

// Close the HID device
void CloseHandles()
{
    if (DeviceHandle) {
        hid_close(DeviceHandle);
        DeviceHandle = nullptr;
    }
    hid_exit();
}

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

bool ReadHIDInputReport(int length)
{
    unsigned char InputReport[HIDREPORTNUM] = { 0 };
    int res = hid_read(DeviceHandle, InputReport, length);
    if (res > 0) {
        std::memcpy(RxData, &InputReport[1], RxNum);
        return true;
    }
    if (res < 0) {
        std::printf("Read failed: %ls\n", hid_error(DeviceHandle));
        return false;
    }
    return false;
}

// C-style wrappers for compatibility
void WriteHIDOutputReport(void) { WriteHIDOutputReport(HIDREPORTNUM); }
void ReadHIDInputReport(void) { ReadHIDInputReport(HIDREPORTNUM); }