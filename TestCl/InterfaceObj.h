// Copyright 2014-2017, Anitoa Systems, LLC
// All rights reserved

#pragma once

#include "TrimReader.h"
#include <string>
#include <cstdint>

#define MAX_IMAGE_SIZE 24

class CInterfaceObject {
protected:
    CTrimReader m_TrimReader;

public:
    int frame_data[MAX_IMAGE_SIZE][MAX_IMAGE_SIZE]; // Captured image frame data
    int cur_chan;

    CInterfaceObject();

    // Callable functions for application developers
    void SetGainMode(int gain); // 0: high gain mode; 1: low gain mode
    void SetTXbin(uint8_t txbin); // Tx Binning pattern: 0x0 to 0xf
    void SetIntTime(float); // Integration time in ms: 1 to 66000
    void SelSensor(uint8_t chan);

    void SetV15(uint8_t v15); // Normally not changed by user
    void SetV20(uint8_t v20); // Normally not changed by user
    void SetRangeTrim(uint8_t range); // Normally not changed by user
    void SetRampgen(uint8_t rampgen); // Normally not changed by user

    void SetLEDConfig(bool IndvEn, bool Chan1, bool Chan2, bool Chan3, bool Chan4);

    int CaptureFrame12(uint8_t chan); // Capture a 12x12 image, 0: success; 1: error detected
    int CaptureFrame24(); // Capture a 24x24 image, 0: success; 1: error detected

    void ProcessRowData();
    int LoadTrimFile();
    void ResetTrim();
    void ReadTrimData(); // From flash
    // Add to the public section of the CInterfaceObject class
    bool ResetUSBEndpoints();

    // Add this to the public section of the CInterfaceObject class
    void CaptureEvenRows(uint8_t chan);

    int IsDeviceDetected(); // 0: Device not detected; 1: device detected
    std::string GetChipName(); // Get the name of the chip embedded in trim.dat file
};