// Copyright 2014-2023, Anitoa Systems, LLC
// All rights reserved

// Add at the top of the file
#include "InterfaceObj.h"
#include "HidMgr.h"
#include <cstring>
#include <string>
#include <thread>
#include <chrono>
#include <filesystem>
#include <vector>   // Make sure this is included

namespace fs = std::filesystem;

// External variables
extern uint8_t TxData[TxNum];
extern uint8_t RxData[RxNum];
extern bool g_DeviceDetected;
extern bool MyDeviceDetected; // redundant, maybe keep only one?
extern int Continue_Flag;
extern bool ee_continue;
extern int chan_num;

// Global state variables
int gain_mode = 0;
float int_time = 1;
int frame_size = 0;

// Global interface object instance
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
    // Configure all 4 sensors with their proper settings
    for (int i = 1; i <= 4; i++) {
        SelSensor(i);
        SetRampgen((uint8_t)m_TrimReader.Node[i - 1].rampgen);
        SetRangeTrim(0x0f);
        SetV20(m_TrimReader.Node[i - 1].auto_v20[1]);
        SetV15(m_TrimReader.Node[i - 1].auto_v15);
        SetGainMode(1);
        SetTXbin(0x8);
        SetIntTime(1);
    }

    // Flash all LEDs briefly then turn off
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

    // Adjust V20 based on gain mode
    if (!gain)
        SetV20(m_TrimReader.Node[cur_chan - 1].auto_v20[1]);  // Low gain
    else
        SetV20(m_TrimReader.Node[cur_chan - 1].auto_v20[0]);  // High gain
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
    printf("Starting capture for channel %d with improved reliability\n", chan);

    // Initialize tracking for each row
    bool rows_received[12] = { false };
    int total_rows = 0;
    const int MAX_RETRIES = 5;  // Increased from 3 to 5
    int retry_count = 0;
    int consecutive_timeouts = 0;
    const int MAX_CONSECUTIVE_TIMEOUTS = 10;

#ifdef __linux__
    if (DeviceHandle) {
        printf("Taking exclusive USB control\n");
        hid_set_nonblocking(DeviceHandle, 0);  // Switch to blocking mode for reliability
        StopHidReadThread();

        // Flush any pending data more aggressively
        unsigned char flush_buffer[HIDREPORTNUM];
        int flush_count = 0;
        while (hid_read_timeout(DeviceHandle, flush_buffer, HIDREPORTNUM, 5) > 0) {
            flush_count++;
        }
        if (flush_count > 0) {
            printf("Flushed %d packets of stale data\n", flush_count);
        }

        // Give device a moment to stabilize after flush
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
#endif

    while (retry_count < MAX_RETRIES && total_rows < 12) {
        // Reset row tracking for this attempt if it's a retry
        if (retry_count > 0) {
            printf("\nAttempt %d of %d to capture missing rows...\n", retry_count + 1, MAX_RETRIES);
            printf("Missing rows:");
            for (int i = 0; i < 12; i++) {
                if (!rows_received[i]) printf(" %d", i);
            }
            printf("\n");

            // Don't reset rows_received array on retries to accumulate rows
            // across attempts, but reset consecutive timeouts
            consecutive_timeouts = 0;
        }

        // Set channel and issue capture command
        chan_num = chan;
        printf("Sending capture command for channel %d\n", chan);
        m_TrimReader.Capture12(chan);
        WriteHIDOutputReport();
        std::memset(TxData, 0, sizeof(TxData));

        // Set continue flag for capture loop
        Continue_Flag = true;

        // Main capture loop
        printf("Reading rows...\n");

        const int MAX_READS = 200;  // Increased from 100
        int reads = 0;

        while (Continue_Flag && reads < MAX_READS && total_rows < 12) {
            // Use timeout-based read for reliability with shorter timeouts but more attempts
            bool success = ReadHIDInputReportTimeout(HIDREPORTNUM, 250);  // 250ms timeout

            if (!success) {
                consecutive_timeouts++;
                if (consecutive_timeouts > MAX_CONSECUTIVE_TIMEOUTS) {
                    printf("Too many consecutive timeouts (%d), breaking capture loop\n",
                        consecutive_timeouts);
                    break;
                }
                printf("No data received (timeout %d/%d)\n",
                    consecutive_timeouts, MAX_CONSECUTIVE_TIMEOUTS);
                continue;
            }

            // Reset timeout counter when we get data
            consecutive_timeouts = 0;

            // Extract and debug packet information
            uint8_t cmd_type = RxData[2];  // Command type
            uint8_t row_type = RxData[4];  // Row type
            uint8_t row = RxData[5];       // Row number

            // Enhanced packet debugging
            printf("Received packet: cmd=0x%02x type=0x%02x row=0x%02x\n", cmd_type, row_type, row);

            // More comprehensive packet type analysis
            if (cmd_type == 0x02) {  // Frame data command
                // Check lower nibble for frame format
                uint8_t frameFormat = row_type & 0x0F;

                // More detailed logging of frame types
                if (frameFormat == 0x02 || frameFormat == 0x22) {
                    printf("Valid 12x12 frame data packet\n");

                    // Get channel from the upper nibble of type (if available)
                    if (row_type & 0xF0) {
                        chan_num = ((row_type & 0xF0) >> 4) + 1;
                        printf("Channel detected in packet: %d\n", chan_num);
                    }

                    // Process the row data
                    ProcessRowData();

                    // Check for end signal
                    if (row == 0x0b || row == 0xf1) {
                        printf("End signal detected: 0x%02x\n", row);
                        Continue_Flag = false;
                    }
                    // Check for valid row number and track it
                    else if (row < 12) {
                        if (!rows_received[row]) {
                            rows_received[row] = true;
                            total_rows++;
                            printf("Got row %d (%d/12 total)\n", row, total_rows);
                        }
                        else {
                            printf("Duplicate row %d received\n", row);
                        }
                    }
                    else {
                        printf("Warning: Invalid row number: %d\n", row);
                    }
                }
                else if (frameFormat == 0x08) {
                    printf("24x24 frame data packet (not expected for 12x12 capture)\n");
                    // Process anyway in case it contains useful data
                    ProcessRowData();

                    if (row < 12) {
                        if (!rows_received[row]) {
                            rows_received[row] = true;
                            total_rows++;
                            printf("Got row %d from 24x24 frame (%d/12 total)\n", row, total_rows);
                        }
                    }
                }
                else {
                    // Try to handle unknown frame types more gracefully
                    printf("Warning: Unknown frame type: %02x - trying to process anyway\n", frameFormat);

                    // Process the data anyway - it might contain valid row information
                    ProcessRowData();

                    // Still try to extract row information
                    if (row < 12) {
                        if (!rows_received[row]) {
                            rows_received[row] = true;
                            total_rows++;
                            printf("Got row %d from unknown frame (%d/12 total)\n", row, total_rows);
                        }
                    }
                }
            }
            else {
                printf("Non-frame data command: 0x%02x\n", cmd_type);
            }

            // Clear RxData for next read
            std::memset(RxData, 0, sizeof(RxData));
            reads++;

            // Break early if we got all rows
            if (total_rows >= 12) {
                printf("All rows received, breaking capture loop\n");
                break;
            }
        }

        // If we didn't get all rows, try again
        if (total_rows < 12) {
            retry_count++;
            // Small delay between retries with exponential backoff
            int delay_ms = 50 * (1 << retry_count);  // 50, 100, 200, 400, 800ms
            printf("Waiting %d ms before retry...\n", delay_ms);
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        }
    }

#ifdef __linux__
    // Restore thread-based reader on Linux
    if (DeviceHandle) {
        printf("Restoring standard USB access mode\n");
        hid_set_nonblocking(DeviceHandle, 1);
        StartHidReadThread();
    }
#endif

    // Report final capture results
    printf("\nCapture complete - received %d/12 rows\n", total_rows);

    if (total_rows < 12) {
        printf("Missing rows:");
        for (int i = 0; i < 12; i++) {
            if (!rows_received[i]) printf(" %d", i);
        }
        printf("\n");

        // Fill missing rows with data from adjacent rows
        if (total_rows > 0) {
            printf("Attempting to fill missing rows with interpolated data\n");
            for (int i = 0; i < 12; i++) {
                if (!rows_received[i]) {
                    // Find closest available rows
                    int prev = i - 1;
                    while (prev >= 0 && !rows_received[prev]) prev--;

                    int next = i + 1;
                    while (next < 12 && !rows_received[next]) next++;

                    if (prev >= 0 && next < 12) {
                        // Interpolate between two rows
                        for (int j = 0; j < 12; j++) {
                            float weight = (float)(i - prev) / (next - prev);
                            frame_data[i][j] = (int)((1 - weight) * frame_data[prev][j] +
                                weight * frame_data[next][j]);
                        }
                        printf("Row %d filled by interpolation\n", i);
                    }
                    else if (prev >= 0) {
                        // Copy from previous
                        for (int j = 0; j < 12; j++) {
                            frame_data[i][j] = frame_data[prev][j];
                        }
                        printf("Row %d filled from row %d\n", i, prev);
                    }
                    else if (next < 12) {
                        // Copy from next
                        for (int j = 0; j < 12; j++) {
                            frame_data[i][j] = frame_data[next][j];
                        }
                        printf("Row %d filled from row %d\n", i, next);
                    }
                }
            }
        }
    }

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