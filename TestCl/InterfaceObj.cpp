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
    bool rows_received[12] = { false };
    int total_rows = 0;

    printf("Starting capture for channel %d (addressing Linux USB buffering issue)\n", chan);

    // Completely drain buffer before starting
    int drained = 0;
    while (ReadHIDInputReportFromQueue()) { drained++; }
    printf("Drained %d stale packets\n", drained);

    // Force USB reset before capture to clear any stalled endpoints
#ifdef __linux__
    if (DeviceHandle) {
        printf("Resetting USB device before capture...\n");
        hid_set_nonblocking(DeviceHandle, 0);  // Switch to blocking mode briefly
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        hid_set_nonblocking(DeviceHandle, 1);  // Back to non-blocking
    }
#endif

    // Make multiple attempts to get all rows
    for (int capture_attempt = 0; capture_attempt < 3 && total_rows < 12; capture_attempt++) {
        printf("Capture attempt %d/3\n", capture_attempt + 1);

        // Send capture command
        m_TrimReader.Capture12(chan);
        WriteHIDOutputReport();
        std::memset(TxData, 0, sizeof(TxData));

        // Allow time for ALL rows to be transmitted before we start reading
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // Now read as many rows as possible in a tight loop
        Continue_Flag = true;
        int attempt = 0;
        int timeout_ms = 1000; // 1 second timeout per attempt
        auto start_time = std::chrono::steady_clock::now();

        while (Continue_Flag && total_rows < 12) {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                current_time - start_time).count();

            if (elapsed > timeout_ms) break;

            if (ReadHIDInputReportFromQueue()) {
                uint8_t row = RxData[4];
                if (row < 12 && !rows_received[row]) {
                    ProcessRowData();
                    rows_received[row] = true;
                    total_rows++;
                    printf("Got row %d (%d/12 rows)\n", row, total_rows);
                }
            }
            else {
                // Extremely short sleep to avoid CPU burning while allowing max throughput
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        }

        // Print status after this attempt
        if (total_rows < 12) {
            printf("After attempt %d: Got %d/12 rows. Missing:", capture_attempt + 1, total_rows);
            for (int i = 0; i < 12; i++) {
                if (!rows_received[i]) printf(" %d", i);
            }
            printf("\n");

            // Wait longer between attempts to let USB bus recover
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    // If we still have missing rows after all attempts, fill with last known good values
    if (total_rows < 12) {
        printf("Warning: Using interpolation for %d missing rows\n", 12 - total_rows);

        // Find a good row to use as template
        int good_row = -1;
        for (int i = 0; i < 12; i++) {
            if (rows_received[i]) {
                good_row = i;
                break;
            }
        }

        // Fill in missing rows with data from good row
        if (good_row >= 0) {
            for (int i = 0; i < 12; i++) {
                if (!rows_received[i]) {
                    for (int j = 0; j < 12; j++) {
                        frame_data[i][j] = frame_data[good_row][j];
                    }
                }
            }
        }
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