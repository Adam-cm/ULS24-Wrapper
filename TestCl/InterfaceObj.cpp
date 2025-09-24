// Copyright 2014-2017, Anitoa Systems, LLC
// All rights reserved

#include "InterfaceObj.h"
#include "HidMgr.h"
#include <cstring>
#include <string>
#include <thread>
#include <chrono>
#include <filesystem>

namespace fs = std::filesystem;

extern uint8_t TxData[TxNum];
extern uint8_t RxData[RxNum];

extern bool g_DeviceDetected;
extern bool MyDeviceDetected; // redundant, maybe keep only one?

int gain_mode = 0;
float int_time = 1;
int frame_size = 0;

extern int Continue_Flag;
extern bool ee_continue;

CInterfaceObject theInterfaceObject;

CInterfaceObject::CInterfaceObject()
{
    cur_chan = 1;
}

std::string CInterfaceObject::GetChipName()
{
    return m_TrimReader.Node[0].name;
}

void CInterfaceObject::ResetTrim()
{
    SelSensor(1);
    SetRampgen((uint8_t)m_TrimReader.Node[0].rampgen);
    SetRangeTrim(0x0f);
    SetV20(m_TrimReader.Node[0].auto_v20[1]);
    SetV15(m_TrimReader.Node[0].auto_v15);
    SetGainMode(1);
    SetTXbin(0x8);
    SetIntTime(1);

    SelSensor(2);
    SetRampgen((uint8_t)m_TrimReader.Node[1].rampgen);
    SetRangeTrim(0x0f);
    SetV20(m_TrimReader.Node[1].auto_v20[1]);
    SetV15(m_TrimReader.Node[1].auto_v15);
    SetGainMode(1);
    SetTXbin(0x8);
    SetIntTime(1);

    SelSensor(3);
    SetRampgen((uint8_t)m_TrimReader.Node[2].rampgen);
    SetRangeTrim(0x0f);
    SetV20(m_TrimReader.Node[2].auto_v20[1]);
    SetV15(m_TrimReader.Node[2].auto_v15);
    SetGainMode(1);
    SetTXbin(0x8);
    SetIntTime(1);

    SelSensor(4);
    SetRampgen((uint8_t)m_TrimReader.Node[3].rampgen);
    SetRangeTrim(0x0f);
    SetV20(m_TrimReader.Node[3].auto_v20[1]);
    SetV15(m_TrimReader.Node[3].auto_v15);
    SetGainMode(1);
    SetTXbin(0x8);
    SetIntTime(1);

    SetLEDConfig(true, true, true, true, true);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    SetLEDConfig(true, false, false, false, false);
}

void CInterfaceObject::SetV15(uint8_t v15)
{
    m_TrimReader.SetV15(v15);
    WriteHIDOutputReport();
    std::memset(TxData, 0, sizeof(TxData));
    ReadHIDInputReport();
}

void CInterfaceObject::SetV20(uint8_t v20)
{
    m_TrimReader.SetV20(v20);
    WriteHIDOutputReport();
    std::memset(TxData, 0, sizeof(TxData));
    ReadHIDInputReport();
}

void CInterfaceObject::SetGainMode(int gain)
{
    m_TrimReader.SetGainMode(gain);
    WriteHIDOutputReport();
    std::memset(TxData, 0, sizeof(TxData));
    ReadHIDInputReport();

    gain_mode = gain;
    if (!gain)
        SetV20(m_TrimReader.Node[cur_chan - 1].auto_v20[1]);
    else
        SetV20(m_TrimReader.Node[cur_chan - 1].auto_v20[0]);
}

void CInterfaceObject::SetRangeTrim(uint8_t range)
{
    m_TrimReader.SetRangeTrim(range);
    WriteHIDOutputReport();
    std::memset(TxData, 0, sizeof(TxData));
    ReadHIDInputReport();
}

void CInterfaceObject::SetRampgen(uint8_t rampgen)
{
    m_TrimReader.SetRampgen(rampgen);
    WriteHIDOutputReport();
    std::memset(TxData, 0, sizeof(TxData));
    ReadHIDInputReport();
}

void CInterfaceObject::SetTXbin(uint8_t txbin)
{
    m_TrimReader.SetTXbin(txbin);
    WriteHIDOutputReport();
    std::memset(TxData, 0, sizeof(TxData));
    ReadHIDInputReport();
}

void CInterfaceObject::SetIntTime(float it)
{
    m_TrimReader.SetIntTime(it);
    WriteHIDOutputReport();
    std::memset(TxData, 0, sizeof(TxData));
    ReadHIDInputReport();
    int_time = it;
}

void CInterfaceObject::SelSensor(uint8_t chan)
{
    m_TrimReader.SelSensor(chan);
    WriteHIDOutputReport();
    std::memset(TxData, 0, sizeof(TxData));
    ReadHIDInputReport();
    cur_chan = static_cast<int>(chan);
}

void CInterfaceObject::SetLEDConfig(bool IndvEn, bool Chan1, bool Chan2, bool Chan3, bool Chan4)
{
    m_TrimReader.SetLEDConfig(IndvEn, Chan1, Chan2, Chan3, Chan4);
    WriteHIDOutputReport();
    std::memset(TxData, 0, sizeof(TxData));
    ReadHIDInputReport();
}

void CInterfaceObject::ProcessRowData()
{
    frame_size = m_TrimReader.ProcessRowData(frame_data, gain_mode);
}

int CInterfaceObject::CaptureFrame12(uint8_t chan)
{
    // Define which rows we're explicitly looking for
    bool rows_received[12] = { false };
    int total_rows = 0;

    printf("Starting full capture for channel %d\n", chan);

    // Try to get all rows in first pass
    m_TrimReader.Capture12(chan);
    WriteHIDOutputReport();
    std::memset(TxData, 0, sizeof(TxData));

    Continue_Flag = true;
    int attempts = 0;
    int max_attempts = 50;

    // Drain buffer first
    while (ReadHIDInputReportFromQueue()) { /* Just drain */ }

    // First pass - get whatever rows come naturally
    while (Continue_Flag && attempts < max_attempts && total_rows < 12) {
        if (ReadHIDInputReportFromQueue()) {
            uint8_t row = RxData[4]; // Row index in the data packet
            if (row < 12 && !rows_received[row]) {
                ProcessRowData();
                rows_received[row] = true;
                total_rows++;
                printf("Got row %d (%d/12 rows)\n", row, total_rows);
                attempts = 0; // Reset timeout counter on successful read
            }
        }
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            attempts++;
        }
    }

    // Now specifically target each missing row with individual requests
    for (int row = 0; row < 12; row++) {
        if (!rows_received[row]) {
            printf("Specifically requesting row %d...\n", row);

            // Request a specific row (modify as needed based on your protocol)
            m_TrimReader.SelSensor(chan);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));

            // Add specific row request command - you'll need to implement this
            TxData[0] = 0xaa;     // preamble code
            TxData[1] = 0x01;     // command
            TxData[2] = 0x03;     // data length
            TxData[3] = 0x30;     // data type for row request (MODIFY THIS)
            TxData[4] = row;      // requested row
            TxData[5] = 0x00;
            TxData[6] = TxData[1] + TxData[2] + TxData[3] + TxData[4] + TxData[5];  // checksum
            if (TxData[6] == 0x17)
                TxData[6] = 0x18;

            WriteHIDOutputReport();
            std::memset(TxData, 0, sizeof(TxData));

            // Try to get this specific row
            attempts = 0;
            bool got_row = false;
            while (attempts < 20 && !got_row) {
                if (ReadHIDInputReportFromQueue()) {
                    if (RxData[4] == row) {
                        ProcessRowData();
                        rows_received[row] = true;
                        total_rows++;
                        got_row = true;
                        printf("Successfully got row %d (%d/12 total)\n", row, total_rows);
                    }
                }
                else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    attempts++;
                }
            }
        }
    }

    // Final report
    if (total_rows < 12) {
        printf("Warning: Could only capture %d/12 rows. Missing rows:", total_rows);
        for (int i = 0; i < 12; i++) {
            if (!rows_received[i]) printf(" %d", i);
        }
        printf("\n");
    }
    else {
        printf("Successfully captured all 12 rows!\n");
    }

    Continue_Flag = false;
    return (total_rows > 0) ? 0 : 1;
}

int CInterfaceObject::CaptureFrame24()
{
    m_TrimReader.Capture24();
    WriteHIDOutputReport();
    std::memset(TxData, 0, sizeof(TxData));
    Continue_Flag = true;
    while (Continue_Flag) {
        ReadHIDInputReport();
        ProcessRowData();
        std::memset(RxData, 0, sizeof(RxData));
    }
    return 0;
}

int CInterfaceObject::LoadTrimFile()
{
    // Use C++17 filesystem for cross-platform current directory
    std::string path = fs::current_path().string();
    path += "/Trim/trim.dat";
    int e = m_TrimReader.Load(path);
    if (e == 0) {
        m_TrimReader.Parse();
    }
    return e;
}

void CInterfaceObject::ReadTrimData()
{
    m_TrimReader.EEPROMRead();
    WriteHIDOutputReport();
    std::memset(TxData, 0, sizeof(TxData));
    while (ee_continue) {
        ReadHIDInputReport();
        m_TrimReader.OnEEPROMRead();
        std::memset(RxData, 0, sizeof(RxData));
    }
    m_TrimReader.ReadTrimData();
    ResetTrim();
}

int CInterfaceObject::IsDeviceDetected()
{
    return g_DeviceDetected;
}