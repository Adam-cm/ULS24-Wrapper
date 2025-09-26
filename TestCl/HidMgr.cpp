// Copyright 2014-2017, Anitoa Systems, LLC
// All rights reserved

#include "HidMgr.h"
#include "TrimReader.h"

//Application global variables 

DWORD								ActualBytesRead;
DWORD								BytesRead;
HIDP_CAPS							Capabilities;
DWORD								cbBytesRead;
PSP_DEVICE_INTERFACE_DETAIL_DATA	detailData;
HANDLE								DeviceHandle;
DWORD								dwError;
char								FeatureReport[256];
HANDLE								hEventObject;
HANDLE								hDevInfo;
GUID								HidGuid;
OVERLAPPED							HIDOverlapped;
char								InputReport[HIDREPORTNUM];
ULONG								Length;
LPOVERLAPPED						lpOverLap;
bool								MyDeviceDetected = FALSE;
CString								MyDevicePathName;
DWORD								NumberOfBytesRead;
char								OutputReport[HIDREPORTNUM];
HANDLE								ReadHandle;
DWORD								ReportType;
ULONG								Required;
CString								ValueToDisplay;
HANDLE								WriteHandle;

extern HWND							hWnd;

//These are the vendor and product IDs to look for.
//Uses Lakeview Research's Vendor ID.

// Original

//int VendorID = 0x0483;
//int ProductID = 0x5750;

int VendorID = 0x0683;
int ProductID = 0x5850;

BYTE TxData[TxNum + 1];		// the buffer of sent data to HID
BYTE RxData[RxNum + 1];		// the buffer of received data from HID

BOOL g_DeviceDetected = false;
BOOL Continue_Flag = false;
BOOL ee_continue = true;

int chan_num = 1;

BOOL DeviceNameMatch(LPARAM lParam)
{

	// Compare the device path name of a device recently attached or removed 
	// with the device path name of the device we want to communicate with.

	PDEV_BROADCAST_HDR lpdb = (PDEV_BROADCAST_HDR)lParam;

	//	DisplayData("MyDevicePathName = " + MyDevicePathName);

	if (lpdb->dbch_devicetype == DBT_DEVTYP_DEVICEINTERFACE)
	{

		PDEV_BROADCAST_DEVICEINTERFACE lpdbi = (PDEV_BROADCAST_DEVICEINTERFACE)lParam;


		CString DeviceNameString;

		//The dbch_devicetype parameter indicates that the event applies to a device interface.
		//So the structure in LParam is actually a DEV_BROADCAST_INTERFACE structure, 
		//which begins with a DEV_BROADCAST_HDR.

		//The dbcc_name parameter of DevBroadcastDeviceInterface contains the device name. 

		//Compare the name of the newly attached device with the name of the device 
		//the application is accessing (myDevicePathName).

		DeviceNameString = lpdbi->dbcc_name;

		//		DisplayData("DeviceNameString = " + DeviceNameString);


		if ((DeviceNameString.CompareNoCase(MyDevicePathName)) == 0)

		{
			//The name matches.

			return true;
		}
		else
		{
			//It's a different device.

			return false;
		}

	}
	else
	{
		return false;
	}
}

bool FindTheHID()
{
	//Use a series of API calls to find a HID with a specified Vendor IF and Product ID.

	HIDD_ATTRIBUTES						Attributes;
	DWORD								DeviceUsage;
	SP_DEVICE_INTERFACE_DATA			devInfoData;
	bool								LastDevice = FALSE;
	int									MemberIndex = 0;
	LONG								Result;
	CString								UsageDescription;

	Length = 0;
	detailData = NULL;
	DeviceHandle = NULL;

	/*
	API function: HidD_GetHidGuid
	Get the GUID for all system HIDs.
	Returns: the GUID in HidGuid.
	*/

	HidD_GetHidGuid(&HidGuid);

	/*
	API function: SetupDiGetClassDevs
	Returns: a handle to a device information set for all installed devices.
	Requires: the GUID returned by GetHidGuid.
	*/

	hDevInfo = SetupDiGetClassDevs
	(&HidGuid,
		NULL,
		NULL,
		DIGCF_PRESENT | DIGCF_INTERFACEDEVICE);

	devInfoData.cbSize = sizeof(devInfoData);

	//Step through the available devices looking for the one we want. 
	//Quit on detecting the desired device or checking all available devices without success.

	MemberIndex = 0;
	LastDevice = FALSE;

	do
	{
		/*
		API function: SetupDiEnumDeviceInterfaces
		On return, MyDeviceInterfaceData contains the handle to a
		SP_DEVICE_INTERFACE_DATA structure for a detected device.
		Requires:
		The DeviceInfoSet returned in SetupDiGetClassDevs.
		The HidGuid returned in GetHidGuid.
		An index to specify a device.
		*/

		Result = SetupDiEnumDeviceInterfaces
		(hDevInfo,
			0,
			&HidGuid,
			MemberIndex,
			&devInfoData);

		if (Result != 0)
		{
			//A device has been detected, so get more information about it.

			/*
			API function: SetupDiGetDeviceInterfaceDetail
			Returns: an SP_DEVICE_INTERFACE_DETAIL_DATA structure
			containing information about a device.
			To retrieve the information, call this function twice.
			The first time returns the size of the structure in Length.
			The second time returns a pointer to the data in DeviceInfoSet.
			Requires:
			A DeviceInfoSet returned by SetupDiGetClassDevs
			The SP_DEVICE_INTERFACE_DATA structure returned by SetupDiEnumDeviceInterfaces.

			The final parameter is an optional pointer to an SP_DEV_INFO_DATA structure.
			This application doesn't retrieve or use the structure.
			If retrieving the structure, set
			MyDeviceInfoData.cbSize = length of MyDeviceInfoData.
			and pass the structure's address.
			*/

			//Get the Length value.
			//The call will return with a "buffer too small" error which can be ignored.

			Result = SetupDiGetDeviceInterfaceDetail
			(hDevInfo,
				&devInfoData,
				NULL,
				0,
				&Length,
				NULL);

			//Allocate memory for the hDevInfo structure, using the returned Length.

			detailData = (PSP_DEVICE_INTERFACE_DETAIL_DATA)malloc(Length);

			//Set cbSize in the detailData structure.

			detailData->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);

			//Call the function again, this time passing it the returned buffer size.

			Result = SetupDiGetDeviceInterfaceDetail
			(hDevInfo,
				&devInfoData,
				detailData,
				Length,
				&Required,
				NULL);

			// Open a handle to the device.
			// To enable retrieving information about a system mouse or keyboard,
			// don't request Read or Write access for this handle.

			/*
			API function: CreateFile
			Returns: a handle that enables reading and writing to the device.
			Requires:
			The DevicePath in the detailData structure
			returned by SetupDiGetDeviceInterfaceDetail.
			*/

			DeviceHandle = CreateFile
			(detailData->DevicePath,
				0,
				FILE_SHARE_READ | FILE_SHARE_WRITE,
				(LPSECURITY_ATTRIBUTES)NULL,
				OPEN_EXISTING,
				0,
				NULL);

			//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
			HidD_SetNumInputBuffers(DeviceHandle, HIDBUFSIZE);
			//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//			DisplayLastError("CreateFile: ");

			/*
			API function: HidD_GetAttributes
			Requests information from the device.
			Requires: the handle returned by CreateFile.
			Returns: a HIDD_ATTRIBUTES structure containing
			the Vendor ID, Product ID, and Product Version Number.
			Use this information to decide if the detected device is
			the one we're looking for.
			*/

			//Set the Size to the number of bytes in the structure.

			Attributes.Size = sizeof(Attributes);

			Result = HidD_GetAttributes
			(DeviceHandle,
				&Attributes);

			//			DisplayLastError("HidD_GetAttributes: ");

						//Is it the desired device?

			MyDeviceDetected = FALSE;


			if (Attributes.VendorID == VendorID)
			{
				if (Attributes.ProductID == ProductID)
				{
					//Both the Vendor ID and Product ID match.

					MyDeviceDetected = TRUE;
					MyDevicePathName = detailData->DevicePath;
					//					DisplayData("Device detected");

										//Register to receive device notifications.

					RegisterForDeviceNotifications();

					//Get the device's capablities.

					GetDeviceCapabilities();

					// Find out if the device is a system mouse or keyboard.

					DeviceUsage = (Capabilities.UsagePage * 256) + Capabilities.Usage;

					if (DeviceUsage == 0x102)
					{
						UsageDescription = "mouse";
					}

					if (DeviceUsage == 0x106)
					{
						UsageDescription = "keyboard";
					}

					if ((DeviceUsage == 0x102) | (DeviceUsage == 0x106))
					{
						//						DisplayData("");
						//						DisplayData("*************************");
						//					 DisplayData("The device is a system " + UsageDescription + ".");
						//					 DisplayData("Windows 2000 and Windows XP don't allow applications");
						//					 DisplayData("to directly request Input reports from or "); 
						//					 DisplayData("write Output reports to these devices.");
						//					 DisplayData("*************************");
						//					 DisplayData("");
					}

					// Get a handle for writing Output reports.

					WriteHandle = CreateFile
					(detailData->DevicePath,
						GENERIC_WRITE,
						FILE_SHARE_READ | FILE_SHARE_WRITE,
						(LPSECURITY_ATTRIBUTES)NULL,
						OPEN_EXISTING,
						0,
						NULL);

					//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
					HidD_SetNumInputBuffers(WriteHandle, HIDBUFSIZE);
					//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//					DisplayLastError("CreateFile: ");

					// Prepare to read reports using Overlapped I/O.

					PrepareForOverlappedTransfer();

					// Set HID driver input buffer
//					HidD_SetNumInputBuffers(DeviceHandle,64);

				} //if (Attributes.ProductID == ProductID)

				else
					//The Product ID doesn't match.

					CloseHandle(DeviceHandle);

			} //if (Attributes.VendorID == VendorID)

			else
				//The Vendor ID doesn't match.

				CloseHandle(DeviceHandle);

			//Free the memory used by the detailData structure (no longer needed).

			free(detailData);

		}  //if (Result != 0)

		else
			//SetupDiEnumDeviceInterfaces returned 0, so there are no more devices to check.

			LastDevice = TRUE;

		//If we haven't found the device yet, and haven't tried every available device,
		//try the next one.

		MemberIndex = MemberIndex + 1;

	} //do

	while ((LastDevice == FALSE) && (MyDeviceDetected == FALSE));

	if (MyDeviceDetected == FALSE) {
		//		DisplayData("Device not detected");
		//		SetDlgItemText(IDC_STATICOpenComm,"Device Not Detected");
		g_DeviceDetected = false;
	}
	else {
		//		DisplayData("Device detected");
		//		SetDlgItemText(IDC_STATICOpenComm,"ULS24 Device Detected");
		g_DeviceDetected = true;
	}

	//Free the memory reserved for hDevInfo by SetupDiClassDevs.

	SetupDiDestroyDeviceInfoList(hDevInfo);
	//	DisplayLastError("SetupDiDestroyDeviceInfoList");

		//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	HidD_SetNumInputBuffers(DeviceHandle, HIDBUFSIZE);
	HidD_SetNumInputBuffers(WriteHandle, HIDBUFSIZE);
	HidD_SetNumInputBuffers(ReadHandle, HIDBUFSIZE);
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	return MyDeviceDetected;
}

void CloseHandles()
{
	//Close open handles.

	if (DeviceHandle != INVALID_HANDLE_VALUE)
	{
		CloseHandle(DeviceHandle);
	}

	if (ReadHandle != INVALID_HANDLE_VALUE)
	{
		CloseHandle(ReadHandle);
	}

	if (WriteHandle != INVALID_HANDLE_VALUE)
	{
		CloseHandle(WriteHandle);
	}
}

void DisplayInputReport()
{
	USHORT	ByteNumber;
	CHAR	ReceivedByte;

	//Display the received data in the log and the Bytes Received List boxes.
	//Start at the top of the List Box.

//	m_BytesReceived.ResetContent();

	//Step through the received bytes and display each.

	for (ByteNumber = 0; ByteNumber < Capabilities.InputReportByteLength; ByteNumber++)
	{
		//Get a byte.

		ReceivedByte = InputReport[ByteNumber];

		//Display it.

		DisplayReceivedData(ReceivedByte);
	}
}

void DisplayReceivedData(char ReceivedByte)
{
	/*
		//Display data received from the device.

		CString	strByteRead;

		//Convert the value to a 2-character Cstring.

		strByteRead.Format("%02X", ReceivedByte);
		strByteRead = strByteRead.Right(2);

		//Display the value in the Bytes Received List Box.

		m_BytesReceived.InsertString(-1, strByteRead);

		//Display the value in the log List Box (optional).
		//MessageToDisplay.Format("%s%s", "Byte 0: ", strByteRead);
		//DisplayData(MessageToDisplay);
		//UpdateData(false);
	*/
}

void GetDeviceCapabilities()
{
	//Get the Capabilities structure for the device.

	PHIDP_PREPARSED_DATA	PreparsedData;

	/*
	API function: HidD_GetPreparsedData
	Returns: a pointer to a buffer containing the information about the device's capabilities.
	Requires: A handle returned by CreateFile.
	There's no need to access the buffer directly,
	but HidP_GetCaps and other API functions require a pointer to the buffer.
	*/

	HidD_GetPreparsedData
	(DeviceHandle,
		&PreparsedData);
	//	DisplayLastError("HidD_GetPreparsedData: ");

		/*
		API function: HidP_GetCaps
		Learn the device's capabilities.
		For standard devices such as joysticks, you can find out the specific
		capabilities of the device.
		For a custom device, the software will probably know what the device is capable of,
		and the call only verifies the information.
		Requires: the pointer to the buffer returned by HidD_GetPreparsedData.
		Returns: a Capabilities structure containing the information.
		*/

	HidP_GetCaps
	(PreparsedData,
		&Capabilities);
	//	DisplayLastError("HidP_GetCaps: ");

		//Display the capabilities
	/*
		ValueToDisplay.Format("%s%X", "Usage Page: ", Capabilities.UsagePage);
	//	DisplayData(ValueToDisplay);
		ValueToDisplay.Format("%s%d", "Input Report Byte Length: ", Capabilities.InputReportByteLength);
	//	DisplayData(ValueToDisplay);
		ValueToDisplay.Format("%s%d", "Output Report Byte Length: ", Capabilities.OutputReportByteLength);
	//	DisplayData(ValueToDisplay);
		ValueToDisplay.Format("%s%d", "Feature Report Byte Length: ", Capabilities.FeatureReportByteLength);
	//	DisplayData(ValueToDisplay);
		ValueToDisplay.Format("%s%d", "Number of Link Collection Nodes: ", Capabilities.NumberLinkCollectionNodes);
	//	DisplayData(ValueToDisplay);
		ValueToDisplay.Format("%s%d", "Number of Input Button Caps: ", Capabilities.NumberInputButtonCaps);
	//	DisplayData(ValueToDisplay);
		ValueToDisplay.Format("%s%d", "Number of InputValue Caps: ", Capabilities.NumberInputValueCaps);
	//	DisplayData(ValueToDisplay);
		ValueToDisplay.Format("%s%d", "Number of InputData Indices: ", Capabilities.NumberInputDataIndices);
	//	DisplayData(ValueToDisplay);
		ValueToDisplay.Format("%s%d", "Number of Output Button Caps: ", Capabilities.NumberOutputButtonCaps);
	//	DisplayData(ValueToDisplay);
		ValueToDisplay.Format("%s%d", "Number of Output Value Caps: ", Capabilities.NumberOutputValueCaps);
	//	DisplayData(ValueToDisplay);
		ValueToDisplay.Format("%s%d", "Number of Output Data Indices: ", Capabilities.NumberOutputDataIndices);
	//	DisplayData(ValueToDisplay);
		ValueToDisplay.Format("%s%d", "Number of Feature Button Caps: ", Capabilities.NumberFeatureButtonCaps);
	//	DisplayData(ValueToDisplay);
		ValueToDisplay.Format("%s%d", "Number of Feature Value Caps: ", Capabilities.NumberFeatureValueCaps);
	//	DisplayData(ValueToDisplay);
		ValueToDisplay.Format("%s%d", "Number of Feature Data Indices: ", Capabilities.NumberFeatureDataIndices);
	///	DisplayData(ValueToDisplay);
	*/
	//No need for PreparsedData any more, so free the memory it's using.

	HidD_FreePreparsedData(PreparsedData);
	//	DisplayLastError("HidD_FreePreparsedData: ") ;
}

void PrepareForOverlappedTransfer()
{
	//Get a handle to the device for the overlapped ReadFiles.

	ReadHandle = CreateFile
	(detailData->DevicePath,
		GENERIC_READ,
		FILE_SHARE_READ | FILE_SHARE_WRITE,
		(LPSECURITY_ATTRIBUTES)NULL,
		OPEN_EXISTING,
		FILE_FLAG_OVERLAPPED,
		NULL);

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	HidD_SetNumInputBuffers(ReadHandle, HIDBUFSIZE);
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//	DisplayLastError("CreateFile (ReadHandle): ");

	//Get an event object for the overlapped structure.

	/*API function: CreateEvent
	Requires:
	  Security attributes or Null
	  Manual reset (true). Use ResetEvent to set the event object's state to non-signaled.
	  Initial state (true = signaled)
	  Event object name (optional)
	Returns: a handle to the event object
	*/

	if (hEventObject == 0)
	{
		hEventObject = CreateEvent
		(NULL,
			TRUE,
			TRUE,
			CString(""));

		//	DisplayLastError("CreateEvent: ") ;

			//Set the members of the overlapped structure.

		HIDOverlapped.hEvent = hEventObject;
		HIDOverlapped.Offset = 0;
		HIDOverlapped.OffsetHigh = 0;
	}
}

void ReadAndWriteToDevice()
{
	//If necessary, find the device and learn its capabilities.
	//Then send a report and request a report.

	//Clear the List Box (optional).
	//m_ResultsList.ResetContent();

//	DisplayData("***HID Test Report***");
//	DisplayCurrentTime();

	//If the device hasn't been detected already, look for it.

	if (MyDeviceDetected == FALSE)
	{
		MyDeviceDetected = FindTheHID();
	}

	// Do nothing if the device isn't detected.

	if (MyDeviceDetected == TRUE)
	{
		//Write a report to the device.

		WriteHIDOutputReport();

		//Active read message

//		SendHIDRead();

		ReadHIDInputReport();
	}
}

// Parsing Received USB data

void ReadHIDInputReport()
{

	// Retrieve an Input report from the device.

	DWORD	Result;

	//The first byte is the report number.
	InputReport[0] = 0;

	/*API call:ReadFile
	'Returns: the report in InputReport.
	'Requires: a device handle returned by CreateFile
	'(for overlapped I/O, CreateFile must be called with FILE_FLAG_OVERLAPPED),
	'the Input report length in bytes returned by HidP_GetCaps,
	'and an overlapped structure whose hEvent member is set to an event object.
	*/

	if (ReadHandle != INVALID_HANDLE_VALUE)
	{
		Result = ReadFile
		(ReadHandle,
			InputReport,
			Capabilities.InputReportByteLength,
			&NumberOfBytesRead,
			(LPOVERLAPPED)&HIDOverlapped
		);
	}

	//	DisplayLastError("ReadFile: ") ;

	//////////////////

		/*API call:WaitForSingleObject
		'Used with overlapped ReadFile.
		'Returns when ReadFile has received the requested amount of data or on timeout.
		'Requires an event object created with CreateEvent
		'and a timeout value in milliseconds.
		*/

	Result = WaitForSingleObject
	(hEventObject,
		264000);	// timer out timer for USB packet, must be longer than longest int time -Zhimin Ding

	long k;
	BYTE rCmd;	//
	BYTE rType;	//

	switch (Result)
	{
	case WAIT_OBJECT_0:
	{
		//
		for (k = 0; k < HIDREPORTNUM - 1; k++)
			RxData[k] = InputReport[k + 1];	//

		rCmd = RxData[2];	//
		rType = RxData[4];	//

		switch (rCmd)
		{
		case GetCmd:
		{
			if ((rType == 0x01) | (rType == 0x02) | (rType == 0x12) | (rType == 0x22) | (rType == 0x32) | (rType == 0x03))		//
			{

				chan_num = (rType & 0xF0) / 16 + 1;

				//=================F1 Code detection

				if ((RxData[5] == 0x0b) || (RxData[5] == 0xf1))		//
				{
					Continue_Flag = false;
					if (RxData[5] == 0xf1)
					{
						//										MessageBox("Error code 0xF1.  Sensor communication time out.", "TestBB", MB_ICONWARNING);
						return;
					}
				}

				//==================	
				//							if (RxData[5]==0x0b)		// HID读到第12行后停止读取
				//								Gra_pageFlag = false;
				else
					Continue_Flag = true;
			}
			else
			{
				if ((rType == 0x07) | (rType == 0x08) | (rType == 0x0b))	//返回24 pixel 数据时
				{
					if (RxData[5] == 0x17)	// HID读到第24行后停止读取
						Continue_Flag = false;
					else
						Continue_Flag = true;
				}
			}

			//						m_GraDlg.SendMessage(UM_GRAPROCESS);				
			break;
		}
		}
		break;
	}
	case WAIT_TIMEOUT:
	{
		//		SetDlgItemText(IDC_STATICOpenComm, "ReadFile timeout");

				/*API call: CancelIo
				Cancels the ReadFile
				Requires the device handle.
				Returns non-zero on success.
				*/

		Result = CancelIo(ReadHandle);

		//A timeout may mean that the device has been removed. 
		//Close the device handles and set MyDeviceDetected = False 
		//so the next access attempt will search for the device.
		CloseHandles();
		MyDeviceDetected = FALSE;
		break;
	}
	default:
	{
		//Close the device handles and set MyDeviceDetected = False 
		//so the next access attempt will search for the device.

		CloseHandles();
		//		SetDlgItemText(IDC_STATICOpenComm,"Can't read from device");
		MyDeviceDetected = FALSE;
		break;
	}
	}

	/*
	API call: ResetEvent
	Sets the event object to non-signaled.
	Requires a handle to the event object.
	Returns non-zero on success.
	*/

	ResetEvent(hEventObject);

	//Display the report data.

	DisplayInputReport();

}

void RegisterForDeviceNotifications()
{

	// Request to receive messages when a device is attached or removed.
	// Also see WM_DEVICECHANGE in BEGIN_MESSAGE_MAP(CPCRProjectDlg, CDialog).

	DEV_BROADCAST_DEVICEINTERFACE DevBroadcastDeviceInterface;
	HDEVNOTIFY DeviceNotificationHandle;

	DevBroadcastDeviceInterface.dbcc_size = sizeof(DevBroadcastDeviceInterface);
	DevBroadcastDeviceInterface.dbcc_devicetype = DBT_DEVTYP_DEVICEINTERFACE;
	DevBroadcastDeviceInterface.dbcc_classguid = HidGuid;

	//	DeviceNotificationHandle =
	//		RegisterDeviceNotification(hWnd, &DevBroadcastDeviceInterface, DEVICE_NOTIFY_WINDOW_HANDLE);

}

void WriteHIDOutputReport()
{
	//Send a report to the device.

	DWORD	BytesWritten = 0;
	INT		Index = 0;
	ULONG	Result;
	CString	strBytesWritten = CString("");

	//	UpdateData(true);

		//The first byte is the report number.

	OutputReport[0] = 0;

	for (int i = 1; i < TxNum + 1; i++)
		OutputReport[i] = TxData[i - 1];

	/*
		API Function: WriteFile
		Sends a report to the device.
		Returns: success or failure.
		Requires:
		A device handle returned by CreateFile.
		A buffer that holds the report.
		The Output Report length returned by HidP_GetCaps,
		A variable to hold the number of bytes written.
	*/

	if (WriteHandle != INVALID_HANDLE_VALUE)
	{
		Result = WriteFile
		(WriteHandle,
			OutputReport,
			Capabilities.OutputReportByteLength,
			&BytesWritten,
			NULL);
	}

	//Display the result of the API call and the report bytes.

	if (!Result)
	{
		//The WriteFile failed, so close the handles, display a message,
		//and set MyDeviceDetected to FALSE so the next attempt will look for the device.

		CloseHandles();
		//			SetDlgItemText(IDC_STATICOpenComm,"Can't write to device");
		MyDeviceDetected = FALSE;
	}

}