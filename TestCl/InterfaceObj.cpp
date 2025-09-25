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
    printf("Starting USB protocol-aware capture for channel %d\n", chan);

    // Initialize tracking arrays
    bool rows_received[12] = { false };
    int total_rows = 0;

#ifdef __linux__
    if (DeviceHandle) {
        printf("Taking exclusive USB control with protocol analysis\n");
        hid_set_nonblocking(DeviceHandle, 0);
        StopHidReadThread();

        unsigned char flush_buffer[HIDREPORTNUM];
        while (hid_read_timeout(DeviceHandle, flush_buffer, HIDREPORTNUM, 5) > 0);
    }
#endif

    // First pass: standard command (gets odd-indexed rows)
    m_TrimReader.Capture12(chan);
    WriteHIDOutputReport();
    std::memset(TxData, 0, sizeof(TxData));

    printf("Pass 1: Reading odd-indexed rows\n");
    for (int i = 0; i < 12; i++) {
        unsigned char buffer[HIDREPORTNUM];
        int res = hid_read_timeout(DeviceHandle, buffer, HIDREPORTNUM, 100);

        if (res > 0) {
            std::memcpy(RxData, &buffer[1], RxNum);
            uint8_t row = RxData[4];

            if (row < 12 && !rows_received[row]) {
                ProcessRowData();
                rows_received[row] = true;
                total_rows++;
                printf("Got row %d (%d/12 rows)\n", row, total_rows);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    // Second pass: Get rows 2, 6, 10 with first bit pattern
    if (total_rows < 12) {
        printf("Pass 2: Reading rows 2, 6, 10 with command variant 1\n");
        m_TrimReader.Capture12(chan);
        TxData[4] |= 0x80;  // Set bit 7
        WriteHIDOutputReport();
        std::memset(TxData, 0, sizeof(TxData));

        for (int i = 0; i < 10; i++) {
            unsigned char buffer[HIDREPORTNUM];
            int res = hid_read_timeout(DeviceHandle, buffer, HIDREPORTNUM, 100);

            if (res > 0) {
                std::memcpy(RxData, &buffer[1], RxNum);
                uint8_t row = RxData[4];

                // If row is 1, 3, or 5 - interpret as 2, 6, or 10
                if (row == 1 && !rows_received[2]) { row = 2; }
                else if (row == 3 && !rows_received[6]) { row = 6; }
                else if (row == 5 && !rows_received[10]) { row = 10; }

                if (row < 12 && !rows_received[row]) {
                    ProcessRowData();
                    rows_received[row] = true;
                    total_rows++;
                    printf("Got row %d with command variant 1 (%d/12 rows)\n", row, total_rows);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    }

    // Third pass: Get rows 4, 8 with second bit pattern
    if (total_rows < 12) {
        printf("Pass 3: Reading rows 4, 8 with command variant 2\n");
        m_TrimReader.Capture12(chan);
        TxData[4] |= 0x40;  // Try bit 6 instead
        WriteHIDOutputReport();
        std::memset(TxData, 0, sizeof(TxData));

        for (int i = 0; i < 10; i++) {
            unsigned char buffer[HIDREPORTNUM];
            int res = hid_read_timeout(DeviceHandle, buffer, HIDREPORTNUM, 100);

            if (res > 0) {
                std::memcpy(RxData, &buffer[1], RxNum);
                uint8_t row = RxData[4];
                printf("Raw row: %d\n", row);

                // If row is 2 or 4 - interpret as 4 or 8
                if (row == 2 && !rows_received[4]) { row = 4; }
                else if (row == 4 && !rows_received[8]) { row = 8; }

                if (row < 12 && !rows_received[row]) {
                    ProcessRowData();
                    rows_received[row] = true;
                    total_rows++;
                    printf("Got row %d with command variant 2 (%d/12 rows)\n", row, total_rows);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    }

    // Fourth pass: Try a combination pattern for any remaining rows
    if (total_rows < 12) {
        printf("Pass 4: Final attempt for remaining rows\n");
        m_TrimReader.Capture12(chan);
        TxData[4] |= 0xC0;  // Try both bits 6 and 7
        WriteHIDOutputReport();
        std::memset(TxData, 0, sizeof(TxData));

        for (int i = 0; i < 10; i++) {
            unsigned char buffer[HIDREPORTNUM];
            int res = hid_read_timeout(DeviceHandle, buffer, HIDREPORTNUM, 100);

            if (res > 0) {
                std::memcpy(RxData, &buffer[1], RxNum);
                uint8_t row = RxData[4];

                if (row < 12 && !rows_received[row]) {
                    ProcessRowData();
                    rows_received[row] = true;
                    total_rows++;
                    printf("Got row %d with command variant 3 (%d/12 rows)\n", row, total_rows);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    }

#ifdef __linux__
    // Restore thread-based reader
    if (DeviceHandle) {
        printf("Restoring standard USB access mode\n");
        hid_set_nonblocking(DeviceHandle, 1);
        StartHidReadThread();
    }
#endif

    // Final report
    printf("USB protocol analysis complete - received %d/12 rows\n", total_rows);
    if (total_rows < 12) {
        printf("Missing rows:");
        for (int i = 0; i < 12; i++) {
            if (!rows_received[i]) printf(" %d", i);
        }
        printf("\n");
    }
    else {
        printf("SUCCESS! All 12 rows received\n");
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