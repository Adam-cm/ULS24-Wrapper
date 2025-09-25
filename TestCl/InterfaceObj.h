#pragma once

#include "TrimReader.h"
#include <string>

// Add this include for DirectUSB functionality
#include "DirectUSB.h"

// Make sure MAX_IMAGE_SIZE is defined and matches TrimReader's expectation
#define MAX_IMAGE_SIZE 12

class CInterfaceObject
{
public:
    CInterfaceObject();
    
    // Add this line to declare the Initialize method
    void Initialize();
    
    // Frame data storage - using uint16_t to match the values we're storing
    uint16_t frame_data[MAX_IMAGE_SIZE][MAX_IMAGE_SIZE];

    // Functions
    std::string GetChipName();
    void SetV15(uint8_t v15);
    void SetV20(uint8_t v20);
    void SetGainMode(int gain);
    void SetRangeTrim(uint8_t range);
    void SetRampgen(uint8_t rampgen);
    void SetTXbin(uint8_t txbin);
    void SetIntTime(float it);
    void SelSensor(uint8_t chan);
    void SetLEDConfig(bool IndvEn, bool Chan1, bool Chan2, bool Chan3, bool Chan4);
    void ResetTrim();
    void ProcessRowData();
    void CaptureEvenRows(uint8_t chan);
    void CompleteCapture12(uint8_t chan);
    int CaptureFrame12(uint8_t chan);
    int CaptureFrame24();
    bool ResetUSBEndpoints();
    int LoadTrimFile();
    void ReadTrimData();
    int IsDeviceDetected();
    int WindowsStyleCapture12(uint8_t chan);

    // Add the new direct USB capture function
    int DirectUSBCapture12(uint8_t chan);
};

extern CInterfaceObject theInterfaceObject;