#include "stdafx.h"
#include <hidapi/hidapi.h>
#include <stdio.h>
#include <string.h>
#include "HidMgr.h"

int g_DeviceDetected = 0;
int Continue_Flag = 0;
int ee_continue = 0;
int chan_num = 0;

// Vendor and Product IDs
#define VENDOR_ID  0x0483
#define PRODUCT_ID 0x5750

#define TxNum 64
#define RxNum 64
#define HIDREPORTNUM 65 // 1 byte report ID + 64 bytes data

// Global device handle
hid_device* DeviceHandle = NULL;

// Buffers for communication (1st byte is report ID, usually 0)
BYTE RxData[RxNum];
BYTE TxData[TxNum];

// Find and open the HID device
bool FindTheHID()
{
    if (hid_init() != 0) {
        printf("hidapi init failed\n");
        return false;
    }

    DeviceHandle = hid_open(VENDOR_ID, PRODUCT_ID, NULL);
    if (DeviceHandle) {
        printf("Device found!\n");
        hid_set_nonblocking(DeviceHandle, 1); // 1 = non-blocking
        return true;
    }
    else {
        printf("Device not found.\n");
        return false;
    }
}

// Close the HID device
void CloseHandles()
{
    if (DeviceHandle) {
        hid_close(DeviceHandle);
        DeviceHandle = NULL;
    }
    hid_exit();
}

bool WriteHIDOutputReport(int length)
{
    unsigned char OutputReport[HIDREPORTNUM] = { 0 };
    OutputReport[0] = 0; // Report ID
    memcpy(&OutputReport[1], TxData, TxNum);
    //printf("OutputReport: ");
    //for (int i = 0; i < length; ++i) printf("%02X ", OutputReport[i]);
    //printf("\n");
    int res = hid_write(DeviceHandle, OutputReport, length);
    Sleep(10);
    if (res < 0) {
        //printf("Write failed: %ls\n", hid_error(DeviceHandle));
        return false;
    }
    return true;
}

bool ReadHIDInputReport(int length)
{
    unsigned char InputReport[HIDREPORTNUM] = { 0 };
    int res = hid_read(DeviceHandle, InputReport, length);
    if (res > 0) {
        memcpy(RxData, &InputReport[1], RxNum);
        //printf("RxData: ");
        //for (int i = 0; i < RxNum; ++i) printf("%02X ", RxData[i]);
        //printf("\n");
        return true;
    }
    if (res < 0) {
        printf("Read failed: %ls\n", hid_error(DeviceHandle));
        return false;
    }
    return false;
}

// C-style wrappers for compatibility
void WriteHIDOutputReport(void) { WriteHIDOutputReport(HIDREPORTNUM); }
void ReadHIDInputReport(void) { ReadHIDInputReport(HIDREPORTNUM); }