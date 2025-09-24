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

#define MAX_LOADSTRING 256

extern int Continue_Flag;

#define TxNum 64        // the number of the buffer for sent data to HID
#define RxNum 64        // the number of the buffer for received data from HID

#define HIDREPORTNUM (64+1)       //  HID report num bytes
#define HIDBUFSIZE 12

#define GetCmd      0x02        // return 0x02 command 
#define ReadCmd     0x04        // Read command

// HID device management
bool FindTheHID();
void CloseHandles();

// HID threaded read management
void StartHidReadThread();
void StopHidReadThread();
bool GetNextHidReport(std::vector<uint8_t>& report);
bool ReadHIDInputReportFromQueue();
bool ReadHIDInputReportBlocking();
bool ReadHIDInputReportTimeout(int length, int timeout_ms);

// HID report I/O
bool WriteHIDOutputReport(int length);
void WriteHIDOutputReport(void);
void ReadHIDInputReport(void);

// (Legacy/Windows-specific, can be guarded or removed if not used on Linux)
void DisplayInputReport();
void DisplayReceivedData(char ReceivedByte);
void GetDeviceCapabilities();
void PrepareForOverlappedTransfer();
void ReadAndWriteToDevice();
void RegisterForDeviceNotifications();