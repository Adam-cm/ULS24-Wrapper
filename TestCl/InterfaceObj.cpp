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

// Add this function after the existing ProcessRowData function
void CInterfaceObject::ProcessRowData()
{
    uint8_t cmd_type = RxData[2];  // Command type
    uint8_t row_type = RxData[4];  // Row type/index

    if (cmd_type == 0x1c) {
        // For 0x1c command, row_type is the row index (0-11)
        int row_idx = row_type;

        // Ensure the row index is valid
        if (row_idx >= 0 && row_idx < 12) {
            // Process the 12 data values in this row (16-bit values)
            for (int i = 0; i < 12; i++) {
                // In 0x1c format, each pixel is a 16-bit value
                // First byte is the high byte, second byte is the low byte
                uint8_t high_byte = RxData[6 + i * 2];
                uint8_t low_byte = RxData[7 + i * 2];

                // Combine into a 16-bit value
                uint16_t value = (high_byte << 8) | low_byte;

                // Store directly in the frame data array
                frame_data[row_idx][i] = value;
            }

            // Print some diagnostics
            printf("Processed row %d with values: ", row_idx);
            for (int i = 0; i < 3; i++) {  // Just show first 3 values
                printf("%d ", frame_data[row_idx][i]);
            }
            printf("...\n");
        }
    }
    else {
        // Use the original ProcessRowData for other command types
        frame_size = m_TrimReader.ProcessRowData(frame_data, gain_mode);
    }
}

int CInterfaceObject::CaptureFrame12(uint8_t chan)
{
    printf("Starting capture for channel %d with improved reliability\n", chan);

    // Initialize tracking for each row
    bool rows_received[12] = { false };
    int total_rows = 0;
    const int MAX_RETRIES = 5;
    int retry_count = 0;
    int consecutive_timeouts = 0;
    const int MAX_CONSECUTIVE_TIMEOUTS = 10;

#ifdef __linux__
    if (DeviceHandle) {
        printf("Taking exclusive USB control\n");
        hid_set_nonblocking(DeviceHandle, 0);  // Switch to blocking mode for reliability
        StopHidReadThread();

        // Flush any pending data
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

        const int MAX_READS = 200;
        int reads = 0;

        while (Continue_Flag && reads < MAX_READS && total_rows < 12) {
            // Use timeout-based read for reliability
            bool success = ReadHIDInputReportTimeout(HIDREPORTNUM, 250);

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

            // Extract packet information
            uint8_t cmd_type = RxData[2];  // Command type
            uint8_t row_type = RxData[4];  // Row type
            uint8_t row = RxData[5];       // Row number

            // Debug output
            printf("Received packet: cmd=0x%02x type=0x%02x row=0x%02x\n", cmd_type, row_type, row);

            // Process the data based on command type
            // IMPORTANT: This section now handles both 0x02 and 0x1c command types
            if (cmd_type == 0x1c) {
                // For 0x1c command, the type field (byte 4) contains the row index (0-11)
                int actual_row = row_type;

                // Ensure row index is within valid range
                if (actual_row >= 0 && actual_row < 12) {
                    // Process the data and update our frame
                    ProcessRowData();

                    // Mark this row as received
                    if (!rows_received[actual_row]) {
                        rows_received[actual_row] = true;
                        total_rows++;
                        printf("Got row %d from 0x1c command (%d/12 total)\n", actual_row, total_rows);
                    }
                    else {
                        printf("Duplicate row %d received\n", actual_row);
                    }
                }
                else {
                    printf("Warning: Invalid row index in 0x1c command: %d\n", actual_row);
                }
            }
            else if (cmd_type == 0x02) {
                // Original handling for 0x02 command
                uint8_t frameFormat = row_type & 0x0F;

                if (frameFormat == 0x02 || frameFormat == 0x22) {
                    printf("Valid 12x12 frame data packet\n");

                    // Get channel from upper nibble if present
                    if (row_type & 0xF0) {
                        chan_num = ((row_type & 0xF0) >> 4) + 1;
                    }

                    // Process the row data
                    ProcessRowData();

                    // Check for end signal
                    if (row == 0x0b || row == 0xf1) {
                        printf("End signal detected: 0x%02x\n", row);
                        Continue_Flag = false;
                    }
                    // Track valid rows
                    else if (row < 12) {
                        if (!rows_received[row]) {
                            rows_received[row] = true;
                            total_rows++;
                            printf("Got row %d from 0x02 command (%d/12 total)\n", row, total_rows);
                        }
                        else {
                            printf("Duplicate row %d received\n", row);
                        }
                    }
                }
            }
            else if (cmd_type == 0x01) {
                // Command acknowledgment
                printf("Command acknowledgement received\n");
            }
            else {
                printf("Unknown command: 0x%02x\n", cmd_type);
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

        // If we got all rows, break out
        if (total_rows == 12) {
            printf("Successfully received all 12 rows!\n");
            break;
        }

        // Prepare for retry
        retry_count++;

        // Use exponential backoff for retries
        int delay_ms = 100 * retry_count;
        printf("Waiting %d ms before retry...\n", delay_ms);
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
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

// Add this function to CInterfaceObject class
void CInterfaceObject::CaptureEvenRows(uint8_t chan)
{
    if (chan < 1 || chan > 4)
        return;

    chan -= 1; // Convert to 0-based for internal use

    // Modified command specifically for even rows
    TxData[0] = 0xaa;      // preamble code
    TxData[1] = 0x02;      // command
    TxData[2] = 0x0C;      // data length
    TxData[3] = (chan << 4) | 0x22;  // modified type for even rows
    TxData[4] = 0xff;      // real data
    TxData[5] = 0x02;      // special flag for even rows
    TxData[6] = 0x00;
    TxData[7] = 0x00;
    TxData[8] = 0x00;
    TxData[9] = 0x00;
    TxData[10] = 0x00;
    TxData[11] = 0x00;
    TxData[12] = 0x00;
    TxData[13] = 0x00;
    TxData[14] = 0x00;
    TxData[15] = TxData[1] + TxData[2] + TxData[3] + TxData[4] + TxData[5] + TxData[6] + TxData[7] +
        TxData[8] + TxData[9] + TxData[10] + TxData[11] + TxData[12] + TxData[13] + TxData[14];

    if (TxData[15] == 0x17)
        TxData[15] = 0x18;

    TxData[16] = 0x17;     // back code
    TxData[17] = 0x17;     // back code
}

// This should replace the existing CaptureFrame12 function
int CInterfaceObject::CaptureFrame12(uint8_t chan)
{
    printf("Starting two-phase capture for channel %d\n", chan);

    // Initialize tracking for each row
    bool rows_received[12] = { false };
    int total_rows = 0;
    int phase = 0;  // 0 = odd rows, 1 = even rows
    const int MAX_PHASES = 2;
    int consecutive_timeouts = 0;
    const int MAX_CONSECUTIVE_TIMEOUTS = 5;  // Reduced for faster phase switching

#ifdef __linux__
    if (DeviceHandle) {
        printf("Taking exclusive USB control\n");
        hid_set_nonblocking(DeviceHandle, 0);  // Switch to blocking mode
        StopHidReadThread();

        // Flush any pending data
        unsigned char flush_buffer[HIDREPORTNUM];
        int flush_count = 0;
        while (hid_read_timeout(DeviceHandle, flush_buffer, HIDREPORTNUM, 5) > 0) {
            flush_count++;
            if (flush_count > 20) break;  // Prevent excessive flushing
        }
        if (flush_count > 0) {
            printf("Flushed %d packets of stale data\n", flush_count);
        }

        // Give device time to stabilize
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
#endif

    // Two-phase capture process - first odd rows, then even rows
    while (phase < MAX_PHASES && total_rows < 12) {
        printf("\nPhase %d: %s rows\n", phase + 1, (phase == 0) ? "odd" : "even");

        // Reset timeout counter for this phase
        consecutive_timeouts = 0;

        // Set channel and issue appropriate capture command
        chan_num = chan;

        if (phase == 0) {
            // First phase - standard capture (primarily gets odd rows)
            printf("Sending command for odd rows\n");
            m_TrimReader.Capture12(chan);
        }
        else {
            // Second phase - specialized capture for even rows
            printf("Sending command for even rows\n");
            CaptureEvenRows(chan);
        }

        WriteHIDOutputReport();
        std::memset(TxData, 0, sizeof(TxData));

        // Set continue flag for capture loop
        Continue_Flag = true;

        // Row capture loop for this phase
        printf("Reading rows...\n");
        const int MAX_READS_PER_PHASE = 50;
        int reads = 0;
        int new_rows_this_phase = 0;

        while (Continue_Flag && reads < MAX_READS_PER_PHASE && total_rows < 12) {
            bool success = ReadHIDInputReportTimeout(HIDREPORTNUM, 150);  // Shorter timeout

            if (!success) {
                consecutive_timeouts++;
                if (consecutive_timeouts > MAX_CONSECUTIVE_TIMEOUTS) {
                    printf("Breaking after %d consecutive timeouts\n", consecutive_timeouts);
                    break;
                }
                continue;
            }

            // Reset timeout counter when we get data
            consecutive_timeouts = 0;

            // Extract packet information
            uint8_t cmd_type = RxData[2];
            uint8_t row_type = RxData[4];
            uint8_t row = RxData[5];

            printf("Received packet: cmd=0x%02x type=0x%02x row=0x%02x\n", cmd_type, row_type, row);

            // Process the data based on command type
            if (cmd_type == 0x1c) {
                int actual_row = row_type;

                if (actual_row >= 0 && actual_row < 12) {
                    ProcessRowData();

                    // Track new rows
                    if (!rows_received[actual_row]) {
                        rows_received[actual_row] = true;
                        total_rows++;
                        new_rows_this_phase++;
                        printf("Got row %d (phase %d) (%d/12 total)\n",
                            actual_row, phase + 1, total_rows);
                    }
                    else {
                        printf("Duplicate row %d\n", actual_row);
                    }
                }
            }
            else if (cmd_type == 0x01) {
                printf("Command acknowledgement received\n");
            }

            // Clear RxData for next read
            std::memset(RxData, 0, sizeof(RxData));
            reads++;
        }

        // Move to next phase if we:
        // 1. Have timeouts (indicating end of data for this phase)
        // 2. Have read enough packets for this phase
        // 3. Haven't received any new rows in this phase
        if (consecutive_timeouts > 0 || reads >= MAX_READS_PER_PHASE || new_rows_this_phase == 0) {
            phase++;

            // Show which rows we have so far
            printf("\nAfter phase %d: %d/12 rows received\n", phase, total_rows);
            printf("Rows received: ");
            for (int i = 0; i < 12; i++) {
                if (rows_received[i]) printf("%d ", i);
            }
            printf("\n");

            // Wait between phases to let the device reset
            if (phase < MAX_PHASES) {
                printf("Waiting between phases...\n");
                std::this_thread::sleep_for(std::chrono::milliseconds(150));
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

    // Report final results
    printf("\nCapture complete - received %d/12 rows\n", total_rows);

    if (total_rows < 12) {
        printf("Missing rows:");
        for (int i = 0; i < 12; i++) {
            if (!rows_received[i]) printf(" %d", i);
        }
        printf("\n");

        // Fill missing rows with interpolated data
        if (total_rows > 0) {
            printf("Filling missing rows with interpolated data\n");

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
                        // Copy from previous row
                        for (int j = 0; j < 12; j++) {
                            frame_data[i][j] = frame_data[prev][j];
                        }
                        printf("Row %d copied from row %d\n", i, prev);
                    }
                    else if (next < 12) {
                        // Copy from next row
                        for (int j = 0; j < 12; j++) {
                            frame_data[i][j] = frame_data[next][j];
                        }
                        printf("Row %d copied from row %d\n", i, next);
                    }
                }
            }
        }
    }

    return (total_rows == 12) ? 0 : 1;
}