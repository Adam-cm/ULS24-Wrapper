// Copyright 2014-2017, Anitoa Systems, LLC
// All rights reserved

#pragma once

#ifdef _WIN32
#include <wtypes.h>
#include <initguid.h>
#else
// Define Windows types for Linux
#include <stdint.h>
typedef uint32_t DWORD;
typedef void* HANDLE;
typedef int BOOL;
typedef unsigned char BYTE;
typedef unsigned short WORD;
typedef long LONG;
typedef unsigned long ULONG;
typedef long long LONGLONG;
typedef unsigned long long ULONGLONG;
typedef void* HWND;
typedef void* HDEVNOTIFY;
typedef void* LPOVERLAPPED;
typedef intptr_t LPARAM;
#define TRUE 1
#define FALSE 0
#endif

#define MAX_LOADSTRING 256

extern "C" {
#ifdef _WIN32
	// This file is in the Windows DDK available from Microsoft.
#include "hidsdi.h"
#include <setupapi.h>
#include <dbt.h>
#endif
}

#define TxNum 64		// the number of the buffer for sent data to HID
#define RxNum 64		// the number of the buffer for received data from HID

#define HIDREPORTNUM 64+1		//	HID report num bytes
#define HIDBUFSIZE 12

#define GetCmd		0x02			// return 0x02 command 
#define ReadCmd		0x04			// Read command

BOOL DeviceNameMatch(LPARAM lParam);
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