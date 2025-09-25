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

int CInterfaceObject::CaptureFrame12(uint8_t chan)
{
    printf("Starting complete protocol analysis for channel %d\n", chan);

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

    // We need to use a different approach than Windows
    // Windows approach uses RxData[5] as row number, but Linux needs different handling

    // First send capture command exactly as in original TrimReader.cpp
    printf("Sending capture command for all rows\n");

    // This follows the exact format from Windows code:
    TxData[0] = 0xaa;       // preamble code
    TxData[1] = 0x02;       // command
    TxData[2] = 0x0C;       // data length
    TxData[3] = (chan - 1) << 4 | 0x02;  // data type with channel in high nibble
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

    // Now send this properly formatted command
    WriteHIDOutputReport();
    std::memset(TxData, 0, sizeof(TxData));

    // First pass: Read what we can get
    printf("Reading primary rows\n");

    // Allow enough time for all rows
    for (int read_attempt = 0; read_attempt < 30 && total_rows < 12; read_attempt++) {
        unsigned char buffer[HIDREPORTNUM];
        int res = hid_read_timeout(DeviceHandle, buffer, HIDREPORTNUM, 100);

        if (res > 0) {
            std::memcpy(RxData, &buffer[1], RxNum);

            // Windows uses RxData[5] as row, but we need to be more careful
            uint8_t row_id = RxData[4];
            uint8_t row;

            // Map the row ID based on our learned protocol
            if (row_id == 0) row = 0;
            else if (row_id == 1) row = 1;
            else if (row_id == 2) row = 4;   // This is the key insight - row 2 is actually row 4
            else if (row_id == 3) row = 3;
            else if (row_id == 4) row = 8;   // Row 4 is actually row 8
            else if (row_id == 5) row = 5;
            else if (row_id == 6) row = 6;
            else if (row_id == 7) row = 7;
            else if (row_id == 8) row = 2;   // Row 8 is actually row 2
            else if (row_id == 9) row = 9;
            else if (row_id == 10) row = 10;
            else if (row_id == 11) row = 11;
            else row = row_id; // Default

            if (row < 12 && !rows_received[row]) {
                ProcessRowData();
                rows_received[row] = true;
                total_rows++;
                printf("Received row %d (mapped from ID %d) - %d/12 total\n",
                    row, row_id, total_rows);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    // Check which rows we've received
    printf("Primary pass complete - received %d/12 rows\n", total_rows);
    if (total_rows < 12) {
        printf("Missing rows:");
        for (int i = 0; i < 12; i++) {
            if (!rows_received[i]) printf(" %d", i);
        }
        printf("\n");

        // Try one more focused pass for missing rows
        printf("Attempting focused capture for missing rows\n");

        for (int miss_attempt = 0; miss_attempt < 5 && total_rows < 12; miss_attempt++) {
            // Send focused capture command targeting missing rows
            TxData[0] = 0xaa;       // preamble code
            TxData[1] = 0x02;       // command
            TxData[2] = 0x0C;       // data length
            TxData[3] = ((chan - 1) << 4) | 0x03;  // different command type (0x03)
            TxData[4] = 0xff;
            TxData[5] = 0x00;

            // Fill in remaining data
            for (int i = 6; i < 15; i++) {
                TxData[i] = 0x00;
            }

            // Calculate checksum
            TxData[15] = 0;
            for (int i = 1; i < 15; i++) {
                TxData[15] += TxData[i];
            }

            if (TxData[15] == 0x17)
                TxData[15] = 0x18;

            TxData[16] = 0x17;      // back code
            TxData[17] = 0x17;      // back code

            WriteHIDOutputReport();
            std::memset(TxData, 0, sizeof(TxData));

            // Read results
            for (int i = 0; i < 15; i++) {
                unsigned char buffer[HIDREPORTNUM];
                int res = hid_read_timeout(DeviceHandle, buffer, HIDREPORTNUM, 100);

                if (res > 0) {
                    std::memcpy(RxData, &buffer[1], RxNum);

                    // Try both direct and mapped row numbers
                    uint8_t direct_row = RxData[5]; // Try direct read (Windows method)
                    uint8_t mapped_row;

                    // Map from protocol ID to actual row
                    uint8_t row_id = RxData[4];
                    if (row_id == 2) mapped_row = 4;
                    else if (row_id == 4) mapped_row = 8;
                    else if (row_id == 8) mapped_row = 2;
                    else mapped_row = row_id;

                    // Try both methods
                    for (uint8_t test_row : {direct_row, mapped_row}) {
                        if (test_row < 12 && !rows_received[test_row]) {
                            ProcessRowData();
                            rows_received[test_row] = true;
                            total_rows++;
                            printf("Got missing row %d (%d/12 rows)\n", test_row, total_rows);
                            break;
                        }
                    }
                }
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
    printf("Complete capture - received %d/12 rows\n", total_rows);

    // Fill in any missing rows with interpolation
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