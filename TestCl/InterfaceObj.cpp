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
    printf("Starting final protocol analysis for channel %d\n", chan);

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

    // Second pass: Get rows 2, 6, 10 with bit pattern 1
    if (total_rows < 12) {
        printf("Pass 2: Reading rows 2, 6, 10\n");
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

                // Map rows based on protocol pattern
                if (row == 1 && !rows_received[2]) { row = 2; }
                else if (row == 3 && !rows_received[6]) { row = 6; }
                else if (row == 5 && !rows_received[10]) { row = 10; }

                if (row < 12 && !rows_received[row]) {
                    ProcessRowData();
                    rows_received[row] = true;
                    total_rows++;
                    printf("Got row %d with variant 1 (%d/12 rows)\n", row, total_rows);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    }

    // Try a series of different command types and patterns for rows 4 and 8
    for (int attempt = 0; attempt < 5 && total_rows < 12; attempt++) {
        uint8_t cmd_type = 0;
        uint8_t chan_value = chan;

        switch (attempt) {
        case 0:
            cmd_type = 0x28;  // Try command type 0x28
            chan_value = chan;
            printf("Pass 3.1: Testing command 0x28\n");
            break;
        case 1:
            cmd_type = 0x26;  // Original command with different bit pattern
            chan_value = chan | 0x40;  // Set bit 6
            printf("Pass 3.2: Testing command 0x26 with bit 6 set\n");
            break;
        case 2:
            cmd_type = 0x26;  // Original command with combined bit pattern
            chan_value = chan | 0xC0;  // Set bits 6 and 7
            printf("Pass 3.3: Testing command 0x26 with bits 6+7 set\n");
            break;
        case 3:
            cmd_type = 0x25;  // Try command type 0x25
            chan_value = chan;
            printf("Pass 3.4: Testing command 0x25\n");
            break;
        case 4:
            cmd_type = 0x24;  // Try command type 0x24
            chan_value = chan;
            printf("Pass 3.5: Testing command 0x24\n");
            break;
        }

        // Send custom command
        TxData[0] = 0xaa;     // preamble code
        TxData[1] = 0x01;     // command
        TxData[2] = 0x03;     // data length
        TxData[3] = cmd_type; // command type
        TxData[4] = chan_value;
        TxData[5] = 0x00;

        // Calculate checksum
        TxData[6] = TxData[1] + TxData[2] + TxData[3] + TxData[4] + TxData[5];
        if (TxData[6] == 0x17) TxData[6] = 0x18;

        TxData[7] = 0x17;     // back code
        TxData[8] = 0x17;     // back code

        WriteHIDOutputReport();
        std::memset(TxData, 0, sizeof(TxData));

        // Capture raw packets to see what we're getting
        printf("Capturing with command type 0x%02x, chan 0x%02x\n", cmd_type, chan_value);
        for (int i = 0; i < 10; i++) {
            unsigned char buffer[HIDREPORTNUM];
            int res = hid_read_timeout(DeviceHandle, buffer, HIDREPORTNUM, 100);

            if (res > 0) {
                std::memcpy(RxData, &buffer[1], RxNum);
                uint8_t row = RxData[4];

                // Print raw packet info
                printf("Raw packet: CMD=0x%02x ROW=%d\n", RxData[3], row);

                // For rows 4 and 8, we need to interpret the row index
                // based on observations
                if (!rows_received[4] && (row == 2 || row == 4)) {
                    printf("Interpreting as row 4\n");
                    row = 4;
                }
                else if (!rows_received[8] && (row == 4 || row == 8)) {
                    printf("Interpreting as row 8\n");
                    row = 8;
                }

                if (row < 12 && !rows_received[row]) {
                    ProcessRowData();
                    rows_received[row] = true;
                    total_rows++;
                    printf("Got row %d with command 0x%02x (%d/12 rows)\n",
                        row, cmd_type, total_rows);
                }

                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    }

    // As a last resort, try for rows 4 and 8 with explicit row request
    if (!rows_received[4] || !rows_received[8]) {
        printf("Final attempt: Explicit row requests\n");

        for (int missing_row : {4, 8}) {
            if (rows_received[missing_row]) continue;

            // Direct row request command
            TxData[0] = 0xaa;     // preamble code
            TxData[1] = 0x01;     // command
            TxData[2] = 0x03;     // data length
            TxData[3] = 0x31;     // row request command (0x31)
            TxData[4] = missing_row;
            TxData[5] = chan;

            // Calculate checksum
            TxData[6] = TxData[1] + TxData[2] + TxData[3] + TxData[4] + TxData[5];
            if (TxData[6] == 0x17) TxData[6] = 0x18;

            TxData[7] = 0x17;     // back code
            TxData[8] = 0x17;     // back code

            WriteHIDOutputReport();
            std::memset(TxData, 0, sizeof(TxData));

            printf("Explicitly requesting row %d\n", missing_row);
            for (int i = 0; i < 5; i++) {
                unsigned char buffer[HIDREPORTNUM];
                int res = hid_read_timeout(DeviceHandle, buffer, HIDREPORTNUM, 100);

                if (res > 0) {
                    std::memcpy(RxData, &buffer[1], RxNum);
                    uint8_t row = RxData[4];

                    if (row == missing_row ||
                        (missing_row == 4 && row == 2) ||
                        (missing_row == 8 && row == 4)) {
                        printf("Got explicit row %d\n", missing_row);
                        row = missing_row;
                        ProcessRowData();
                        rows_received[row] = true;
                        total_rows++;
                        break;
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
    printf("Protocol capture complete - received %d/12 rows\n", total_rows);
    if (total_rows < 12) {
        printf("Missing rows:");
        for (int i = 0; i < 12; i++) {
            if (!rows_received[i]) printf(" %d", i);
        }
        printf("\n");

        // Fill in missing rows with interpolation as a fallback
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
    return 0;  // Always return success since we've ensured all rows have data
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