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
#endif

#define MAX_LOADSTRING 256

extern int Continue_Flag;

#define TxNum 64        // the number of the buffer for sent data to HID
#define RxNum 64        // the number of the buffer for received data from HID

#define HIDREPORTNUM 64+1       //  HID report num bytes
#define HIDBUFSIZE 12

#define GetCmd      0x02        // return 0x02 command 
#define ReadCmd     0x04        // Read command

// Only declare these if you actually use them on Linux, otherwise guard or remove
bool DeviceNameMatch(LPARAM lParam);
bool FindTheHID();
void CloseHandles();
void DisplayInputReport();
void DisplayReceivedData(char ReceivedByte);
void GetDeviceCapabilities();
void PrepareForOverlappedTransfer();
void ReadAndWriteToDevice();
void ReadHIDInputReport();
void RegisterForDeviceNotifications();
void WriteHIDOutputReport();