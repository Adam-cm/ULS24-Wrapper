// Copyright 2014-2017, Anitoa Systems, LLC
// All rights reserved

#include "stdafx.h"


#include "InterfaceObj.h"
#include "HidMgr.h"

extern BYTE TxData[TxNum];		// the buffer of sent data to HID
extern BYTE RxData[RxNum];		// the buffer of received data from HID

extern BOOL g_DeviceDetected;
extern bool	MyDeviceDetected;	// redundant, maybe keep only one?

int gain_mode = 0;					// todo: move to member variable
float int_time = 1;				// integration time
int frame_size = 0;				// 0: 12x12 frame; 1: 24x24 frame

extern BOOL Continue_Flag;
extern BOOL ee_continue;

TCHAR g_CurrentDirectory[MAX_PATH];

CInterfaceObject::CInterfaceObject()
{
	cur_chan = 1;
}

CString CInterfaceObject::GetChipName()
{
	return m_TrimReader.Node[0].name;
}

/////////////////////////////////////////////////////////////////////////////
// Below are interfaces to set ULS24 internal parameters - called trim data. 
/////////////////////////////////////////////////////////////////////////////

void CInterfaceObject::ResetTrim()
{
	SelSensor(1);
	SetRampgen((BYTE)m_TrimReader.Node[0].rampgen);
	SetRangeTrim(0x0f);
	SetV20(m_TrimReader.Node[0].auto_v20[1]);
	SetV15(m_TrimReader.Node[0].auto_v15);
	SetGainMode(1);			// Low gain
	SetTXbin(0x8);
	SetIntTime(1);			// 1 ms

	SelSensor(2);
	SetRampgen((BYTE)m_TrimReader.Node[1].rampgen);
	SetRangeTrim(0x0f);
	SetV20(m_TrimReader.Node[1].auto_v20[1]);
	SetV15(m_TrimReader.Node[1].auto_v15);
	SetGainMode(1);			// Low gain
	SetTXbin(0x8);
	SetIntTime(1);			// 1 ms

	SelSensor(3);
	SetRampgen((BYTE)m_TrimReader.Node[2].rampgen);
	SetRangeTrim(0x0f);
	SetV20(m_TrimReader.Node[2].auto_v20[1]);
	SetV15(m_TrimReader.Node[2].auto_v15);
	SetGainMode(1);			// Low gain
	SetTXbin(0x8);
	SetIntTime(1);			// 1 ms

	SelSensor(4);
	SetRampgen((BYTE)m_TrimReader.Node[3].rampgen);
	SetRangeTrim(0x0f);
	SetV20(m_TrimReader.Node[3].auto_v20[1]);
	SetV15(m_TrimReader.Node[3].auto_v15);
	SetGainMode(1);			// Low gain
	SetTXbin(0x8);
	SetIntTime(1);			// 1 ms

	SetLEDConfig(1, 1, 1, 1, 1);			// Set Multi LED mode, first enable all channels, then disable all channels.
	Sleep(100);								// Why do we need to do this
	SetLEDConfig(1, 0, 0, 0, 0);
}

void CInterfaceObject::SetV15(BYTE v15)
{
	m_TrimReader.SetV15(v15);

	WriteHIDOutputReport();		// 
	memset(TxData, 0, sizeof(TxData));
	ReadHIDInputReport();
}

void CInterfaceObject::SetV20(BYTE v20)
{
	m_TrimReader.SetV20(v20);

	WriteHIDOutputReport();		// 
	memset(TxData, 0, sizeof(TxData));
	ReadHIDInputReport();
}


void CInterfaceObject::SetGainMode(int gain)
{
	m_TrimReader.SetGainMode(gain);

	WriteHIDOutputReport();		// 
	memset(TxData, 0, sizeof(TxData));
	ReadHIDInputReport();

	gain_mode = gain;

	// When gain mode change, V20 needs to change also
	if (!gain) SetV20(m_TrimReader.Node[cur_chan - 1].auto_v20[1]); // auto_v20_hg);
	else SetV20(m_TrimReader.Node[cur_chan - 1].auto_v20[0]); // auto_v20_lg);
}

void  CInterfaceObject::SetRangeTrim(BYTE range)
{
	m_TrimReader.SetRangeTrim(range);

	WriteHIDOutputReport();		// 
	memset(TxData, 0, sizeof(TxData));
	ReadHIDInputReport();
}

void  CInterfaceObject::SetRampgen(BYTE rampgen)
{
	m_TrimReader.SetRampgen(rampgen);

	WriteHIDOutputReport();		// 
	memset(TxData, 0, sizeof(TxData));
	ReadHIDInputReport();
}

void  CInterfaceObject::SetTXbin(BYTE txbin)
{
	m_TrimReader.SetTXbin(txbin);

	WriteHIDOutputReport();		// 
	memset(TxData, 0, sizeof(TxData));
	ReadHIDInputReport();
}

///////////////////////////////////////////////////////
// Below are routines to adjust integration time
////////////////////////////////////////////////////////

void  CInterfaceObject::SetIntTime(float it)
{
	m_TrimReader.SetIntTime(it);

	WriteHIDOutputReport();		// 
	memset(TxData, 0, sizeof(TxData));
	ReadHIDInputReport();

	int_time = it;
}

void  CInterfaceObject::SelSensor(BYTE chan)
{
	m_TrimReader.SelSensor(chan);

	WriteHIDOutputReport();		// 
	memset(TxData, 0, sizeof(TxData));
	ReadHIDInputReport();

	cur_chan = (int)chan;
}

void  CInterfaceObject::SetLEDConfig(BOOL IndvEn, BOOL Chan1, BOOL Chan2, BOOL Chan3, BOOL Chan4)
{
	m_TrimReader.SetLEDConfig(IndvEn, Chan1, Chan2, Chan3, Chan4);

	WriteHIDOutputReport();		// 
	memset(TxData, 0, sizeof(TxData));
	ReadHIDInputReport();
}

void CInterfaceObject::ProcessRowData()
{
	frame_size = m_TrimReader.ProcessRowData(frame_data, gain_mode);
}

int  CInterfaceObject::CaptureFrame12(BYTE chan)
{
	// Issue capture command

	m_TrimReader.Capture12(chan);
	WriteHIDOutputReport();		// 
	memset(TxData, 0, sizeof(TxData));

	// Read and process result
	Continue_Flag = true;

	while (Continue_Flag) {		// Process data row by row
		ReadHIDInputReport();
		ProcessRowData();
		//		((CTestBBDlg*)pDlg)->DrawPattern();
		memset(RxData, 0, sizeof(RxData));
	}

	// Application developer can add code here to further process 
	// the data, that is save in "adc_result[24][24]

	return 0;
}

int  CInterfaceObject::CaptureFrame24()
{
	// Issue capture command
	m_TrimReader.Capture24();
	WriteHIDOutputReport();		// 
	memset(TxData, 0, sizeof(TxData));

	// Read and process result
	Continue_Flag = true;

	while (Continue_Flag) {		// Process data row by row
		ReadHIDInputReport();
		ProcessRowData();
		//		((CTestBBDlg*)pDlg)->DrawPattern();
		memset(RxData, 0, sizeof(RxData));
	}

	// Application developer can add code here to further process 
	// the data, that is save in "adc_result[24][24]

	return 0;
}

int  CInterfaceObject::LoadTrimFile()
{
	GetCurrentDirectory(MAX_PATH, g_CurrentDirectory);

	CString path;
	path = g_CurrentDirectory;
	path += "\\Trim\\trim.dat";

	LPTSTR lpszData = path.GetBuffer(path.GetLength());
	int e = m_TrimReader.Load((TCHAR*)lpszData);
	path.ReleaseBuffer(0);

	if (e) {
		m_TrimReader.Parse();
	}

	return e;
}

void CInterfaceObject::ReadTrimData()	// From flash
{

	m_TrimReader.EEPROMRead();

	WriteHIDOutputReport();		// 
	memset(TxData, 0, sizeof(TxData));

	while (ee_continue) {
		ReadHIDInputReport();
		m_TrimReader.OnEEPROMRead();
		memset(RxData, 0, sizeof(RxData));
	}

	m_TrimReader.ReadTrimData();

	ResetTrim();
}

int CInterfaceObject::IsDeviceDetected()
{
	return g_DeviceDetected;
}