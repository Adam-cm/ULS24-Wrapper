// Copyright 2014-2017, Anitoa Systems, LLC
// All rights reserved

#pragma once

#ifdef _WIN32
#include <windows.h>
#else
#include <stdint.h>
typedef uint8_t BYTE;
typedef int BOOL;
#define TRUE 1
#define FALSE 0
typedef char TCHAR;
#endif

#include "TrimReader.h"

#define MAX_IMAGE_SIZE 24

class CInterfaceObject {

protected:

	CTrimReader m_TrimReader;

public:

	int frame_data[MAX_IMAGE_SIZE][MAX_IMAGE_SIZE];				// Captured image frame data
	int cur_chan;

public:

	CInterfaceObject();

	///////////////////////////////////////////////////////
	//  Callable functions for application developers
	///////////////////////////////////////////////////////

	void SetGainMode(int);				// 0: high gain mode; 1: low gain mode
	void SetTXbin(BYTE txbin);			// Tx Binning pattern: 0x0 to 0xf
	void SetIntTime(float);				// Integration time in ms: 1 to 66000
	void SelSensor(BYTE);

	//	BYTE GetGainMode(int);
	//	BYTE GetTXbin();
	//	int  GetIntTime();

	void SetV15(BYTE v15);				// Normally not changed by user. See ULS24 Solution Kit Datasheet
	void SetV20(BYTE v20);				// Normally not changed by user. See ULS24 Solution Kit Datasheet
	void SetRangeTrim(BYTE range);		// Normally not changed by user. See ULS24 Solution Kit Datasheet
	void SetRampgen(BYTE rampgen);		// Normally not changed by user. See ULS24 Solution Kit Datasheet

	void SetLEDConfig(BOOL IndvEn, BOOL Chan1, BOOL Chan2, BOOL Chan3, BOOL Chan4);

	//	BYTE GetV15();
	//	BYTE GetV20();
	//	BYTE GetRangeTrim();
	//	BYTE GetRampgen();

	int CaptureFrame12(/*int (*frame_data)[IMAGE_SIZE]*/BYTE chan);				// Capture a 12X12 image, 0: success; 1: error detected
	int CaptureFrame24(/*int (*frame_data)[IMAGE_SIZE]*/);				// Capture a 24X24 image, 0: success; 1: error detected

	//	void DrawImage(int (*frame_data)[IMAGE_SIZE], int contrast);	// Display image in GUI, contrast range 1-10

	void ProcessRowData();
	int LoadTrimFile();
	void ResetTrim();


	void ReadTrimData();	// From flash

	int IsDeviceDetected();				// 0: Device not detected; 1: device detected. 
#ifdef _WIN32
	CString	GetChipName();				// Get the name of the chip embedded in trim.dat file
#else
	const char* GetChipName();
#endif
};