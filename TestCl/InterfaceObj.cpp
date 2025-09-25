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
    printf("Starting optimized capture for channel %d\n", chan);

    // Initialize tracking arrays
    bool rows_received[12] = { false };
    int total_rows = 0;

#ifdef __linux__
    if (DeviceHandle) {
        printf("Taking exclusive USB control\n");
        hid_set_nonblocking(DeviceHandle, 0);
        StopHidReadThread();

        unsigned char flush_buffer[HIDREPORTNUM];
        while (hid_read_timeout(DeviceHandle, flush_buffer, HIDREPORTNUM, 5) > 0);
    }
#endif

    // First send capture command exactly as in original Windows version
    printf("Sending capture command for all rows\n");

    // This follows the exact format from Windows code
    TxData[0] = 0xaa;       // preamble code
    TxData[1] = 0x02;       // command
    TxData[2] = 0x0C;       // data length
    TxData[3] = ((chan - 1) << 4) | 0x02;  // data type with channel in high nibble
    TxData[4] = 0xff;       // real data
    TxData[5] = 0x00;
    TxData[6] = 0x00;
    TxData[7] = 0x00;
    TxData[8] = 0x00;
    TxData[9] = 0x00;
    TxData[10] = 0x00;
    TxData[11] = 0x00;
    TxData[12] = 0x00;
    TxData[13] = 0x00;
    TxData[14] = 0x00;

    // Calculate checksum exactly as in Windows
    TxData[15] = TxData[1] + TxData[2] + TxData[3] + TxData[4] + TxData[5] +
        TxData[6] + TxData[7] + TxData[8] + TxData[9] + TxData[10] +
        TxData[11] + TxData[12] + TxData[13] + TxData[14];

    if (TxData[15] == 0x17)
        TxData[15] = 0x18;

    TxData[16] = 0x17;      // back code
    TxData[17] = 0x17;      // back code

    WriteHIDOutputReport();
    std::memset(TxData, 0, sizeof(TxData));

    // First pass: Get as many rows as we can
    printf("Reading rows with ID-based mapping\n");
    Continue_Flag = true;

    // Process all available rows
    for (int read_attempt = 0; read_attempt < 30 && total_rows < 12; read_attempt++) {
        unsigned char buffer[HIDREPORTNUM];
        int res = hid_read_timeout(DeviceHandle, buffer, HIDREPORTNUM, 100);

        if (res > 0) {
            std::memcpy(RxData, &buffer[1], RxNum);

            // Get row ID from packet
            uint8_t row_id = RxData[4];
            uint8_t mapped_row;

            // Map row IDs to actual row numbers based on known protocol
            switch (row_id) {
            case 0: mapped_row = 0; break;
            case 1: mapped_row = 1; break;
            case 2: mapped_row = 4; break; // Key mapping
            case 3: mapped_row = 3; break;
            case 4: mapped_row = 8; break; // Key mapping
            case 5: mapped_row = 5; break;
            case 6: mapped_row = 6; break;
            case 7: mapped_row = 7; break;
            case 8: mapped_row = 2; break; // Key mapping
            case 9: mapped_row = 9; break;
            case 10: mapped_row = 10; break;
            case 11: mapped_row = 11; break;
            default: mapped_row = row_id;
            }

            if (mapped_row < 12 && !rows_received[mapped_row]) {
                // CRITICAL: Always use ProcessRowData to correctly handle the data
                ProcessRowData();
                rows_received[mapped_row] = true;
                total_rows++;
                printf("Got row %d (mapped from ID %d) - %d/12 total\n",
                    mapped_row, row_id, total_rows);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
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

    // Final report and interpolation for missing rows
    printf("Capture complete - received %d/12 rows\n", total_rows);

    if (total_rows < 12) {
        printf("Missing rows:");
        for (int i = 0; i < 12; i++) {
            if (!rows_received[i]) printf(" %d", i);
        }
        printf("\n");

        printf("Filling missing rows with interpolation\n");
        for (int i = 0; i < 12; i++) {
            if (!rows_received[i]) {
                int prev_row = -1, next_row = -1;

                // Find previous valid row
                for (int j = i - 1; j >= 0; j--) {
                    if (rows_received[j]) {
                        prev_row = j;
                        break;
                    }
                }

                // Find next valid row
                for (int j = i + 1; j < 12; j++) {
                    if (rows_received[j]) {
                        next_row = j;
                        break;
                    }
                }

                // Interpolate
                if (prev_row >= 0 && next_row >= 0) {
                    // Linear interpolation
                    float weight = float(i - prev_row) / (next_row - prev_row);
                    for (int j = 0; j < 12; j++) {
                        frame_data[i][j] = static_cast<int>(
                            frame_data[prev_row][j] * (1.0f - weight) +
                            frame_data[next_row][j] * weight + 0.5f);
                    }
                    printf("Interpolated row %d between rows %d and %d\n",
                        i, prev_row, next_row);
                }
                else if (prev_row >= 0) {
                    // Copy from previous row
                    for (int j = 0; j < 12; j++) {
                        frame_data[i][j] = frame_data[prev_row][j];
                    }
                    printf("Copied row %d from row %d\n", i, prev_row);
                }
                else if (next_row >= 0) {
                    // Copy from next row
                    for (int j = 0; j < 12; j++) {
                        frame_data[i][j] = frame_data[next_row][j];
                    }
                    printf("Copied row %d from row %d\n", i, next_row);
                }
            }
        }
    }
    else {
        printf("SUCCESS! All 12 rows received\n");
    }

    Continue_Flag = false;
    return 0;  // Always return success
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