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
    printf("Starting comprehensive multi-pass capture for channel %d\n", chan);

    // Initialize tracking arrays
    bool rows_received[12] = { false };
    int total_rows = 0;
    int attempts_per_approach = 0;

#ifdef __linux__
    if (DeviceHandle) {
        printf("Taking exclusive USB control\n");
        hid_set_nonblocking(DeviceHandle, 0);
        StopHidReadThread();

        unsigned char flush_buffer[HIDREPORTNUM];
        while (hid_read_timeout(DeviceHandle, flush_buffer, HIDREPORTNUM, 5) > 0);
    }
#endif

    // We'll use multiple different approaches until we get all rows
    const int MAX_APPROACHES = 8;

    for (int approach = 0; approach < MAX_APPROACHES && total_rows < 12; approach++) {
        // Configure the command based on the current approach
        TxData[0] = 0xaa;       // preamble code
        TxData[1] = 0x02;       // command
        TxData[2] = 0x0C;       // data length

        // Vary the command parameters based on the approach
        switch (approach) {
        case 0:
            // Standard approach - gets odd-indexed rows typically
            printf("\nApproach 1: Standard capture command\n");
            TxData[3] = ((chan - 1) << 4) | 0x02;
            attempts_per_approach = 20;
            break;

        case 1:
            // Modified command to get rows 2, 6, 10
            printf("\nApproach 2: Targeting rows 2, 6, 10\n");
            TxData[3] = ((chan - 1) << 4) | 0x02;
            TxData[4] = 0x80;  // Set high bit
            attempts_per_approach = 20;
            break;

        case 2:
            // Specific command for rows 4, 8
            printf("\nApproach 3: Targeting rows 4, 8\n");
            TxData[3] = ((chan - 1) << 4) | 0x03;  // Different command type
            attempts_per_approach = 20;
            break;

        case 3:
            // Try with bit 7 in channel
            printf("\nApproach 4: Alternate channel bitmask\n");
            TxData[3] = ((chan - 1) << 4) | 0x02;
            TxData[4] = chan | 0x80;
            attempts_per_approach = 15;
            break;

        case 4:
            // Try a reset + standard command
            printf("\nApproach 5: Reset device + standard capture\n");
            // Send a reset command first
            unsigned char reset_cmd[9] = { 0xaa, 0x10, 0x01, 0x00, 0x00, 0x11, 0x17, 0x17 };
            unsigned char reset_buffer[HIDREPORTNUM] = { 0 };
            reset_buffer[0] = 0; // Report ID
            std::memcpy(&reset_buffer[1], reset_cmd, 8);
            hid_write(DeviceHandle, reset_buffer, HIDREPORTNUM);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));

            // Then standard command
            TxData[3] = ((chan - 1) << 4) | 0x02;
            attempts_per_approach = 25;
            break;

        case 5:
            // Try with a longer timeout and specific command type
            printf("\nApproach 6: Longer timeout with specific command\n");
            TxData[3] = ((chan - 1) << 4) | 0x04;  // Different command type
            attempts_per_approach = 30;
            break;

        case 6:
            // Try 0x26 type command directly (from windows code)
            printf("\nApproach 7: Using 0x26 type command\n");
            TxData[0] = 0xaa;       // preamble code
            TxData[1] = 0x01;       // command
            TxData[2] = 0x03;       // data length
            TxData[3] = 0x26;       // data type from windows
            TxData[4] = chan - 1;   // channel index
            TxData[5] = 0x00;
            attempts_per_approach = 25;
            break;

        case 7:
            // Try 0x28 type command for specific rows
            printf("\nApproach 8: Using 0x28 type command\n");
            TxData[0] = 0xaa;       // preamble code
            TxData[1] = 0x01;       // command
            TxData[2] = 0x03;       // data length
            TxData[3] = 0x28;       // different data type
            TxData[4] = chan;       // channel
            TxData[5] = 0x00;
            attempts_per_approach = 25;
            break;
        }

        // Fill in the rest of the data field with zeros if not explicitly set
        for (int i = 4; i < 15 && approach < 6; i++) {
            if (i == 4 && (approach == 1 || approach == 3)) {
                // Skip if we explicitly set this byte
            }
            else {
                TxData[i] = 0x00;
            }
        }

        // Calculate checksum
        if (approach < 6) {
            TxData[15] = 0;
            for (int i = 1; i < 15; i++) {
                TxData[15] += TxData[i];
            }

            if (TxData[15] == 0x17)
                TxData[15] = 0x18;

            TxData[16] = 0x17;      // back code
            TxData[17] = 0x17;      // back code
        }
        else {
            // For approaches 6-7 (custom commands)
            TxData[6] = TxData[1] + TxData[2] + TxData[3] + TxData[4] + TxData[5];
            if (TxData[6] == 0x17)
                TxData[6] = 0x18;

            TxData[7] = 0x17;      // back code
            TxData[8] = 0x17;      // back code
        }

        // Send command
        WriteHIDOutputReport();
        std::memset(TxData, 0, sizeof(TxData));

        // Read responses
        printf("Reading with %d attempts...\n", attempts_per_approach);
        for (int attempt = 0; attempt < attempts_per_approach && total_rows < 12; attempt++) {
            unsigned char buffer[HIDREPORTNUM];
            int res = hid_read_timeout(DeviceHandle, buffer, HIDREPORTNUM, 100);

            if (res > 0) {
                std::memcpy(RxData, &buffer[1], RxNum);

                // Map row IDs to actual row numbers based on approach
                uint8_t row_id = RxData[4];
                uint8_t mapped_row;

                if (approach == 1) {
                    // For approach 1, interpret differently
                    switch (row_id) {
                    case 1: mapped_row = 2; break;
                    case 3: mapped_row = 6; break;
                    case 5: mapped_row = 10; break;
                    default: mapped_row = row_id; break;
                    }
                }
                else if (approach == 2) {
                    // For approach 2, interpret differently
                    switch (row_id) {
                    case 2: mapped_row = 4; break;
                    case 4: mapped_row = 8; break;
                    default: mapped_row = row_id; break;
                    }
                }
                else {
                    // Standard row mapping based on observed behavior
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
                    default: mapped_row = row_id; break;
                    }
                }

                if (mapped_row < 12 && !rows_received[mapped_row]) {
                    ProcessRowData();
                    rows_received[mapped_row] = true;
                    total_rows++;
                    printf("Got row %d (mapped from ID %d) - %d/12 total\n",
                        mapped_row, row_id, total_rows);
                }
            }

            // Short delay between reads
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        // Check if we have all rows
        printf("After approach %d: %d/12 rows received\n", approach + 1, total_rows);
    }

    // Print final status
    printf("\nFinal capture status: %d/12 rows received\n", total_rows);
    if (total_rows < 12) {
        printf("Missing rows:");
        for (int i = 0; i < 12; i++) {
            if (!rows_received[i]) printf(" %d", i);
        }
        printf("\n");
    }
    else {
        printf("SUCCESS! All rows received directly from device - no interpolation needed.\n");
    }

#ifdef __linux__
    // Restore thread-based reader
    if (DeviceHandle) {
        printf("Restoring standard USB access mode\n");
        hid_set_nonblocking(DeviceHandle, 1);
        StartHidReadThread();
    }
#endif

    Continue_Flag = false;
    return (total_rows == 12) ? 0 : 1;  // Return success only if we got all rows
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