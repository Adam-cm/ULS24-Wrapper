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

// Improved function to request even rows specifically
void CInterfaceObject::CaptureEvenRows(uint8_t chan)
{
    if (chan < 1 || chan > 4)
        return;

    chan -= 1; // Convert to 0-based for internal use

    // Try a different approach for even rows - using direct row addressing
    // This approach sends a separate command for each even row
    for (uint8_t row = 0; row < 12; row += 2) {
        // Skip row 0 as we already have it from the odd capture
        if (row == 0) continue;

        // Create a command that targets a specific row
        TxData[0] = 0xaa;      // preamble code
        TxData[1] = 0x02;      // command
        TxData[2] = 0x0C;      // data length
        TxData[3] = (chan << 4) | 0x42;  // special type for direct row request
        TxData[4] = row;       // Request this specific row
        TxData[5] = 0x01;      // Flag for specific row
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

        // Send the command
        WriteHIDOutputReport();
        std::memset(TxData, 0, sizeof(TxData));

        // Wait for a short time between row requests
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

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
    printf("Starting multi-mode capture for channel %d\n", chan);
    
    // DEBUG: Print capture parameters
    printf("Capture parameters: Channel=%d, Gain=%d, IntTime=%.2f\n", 
           chan, gain_mode, int_time);

    // Initialize tracking for each row
    bool rows_received[12] = { false };
    int total_rows = 0;

#ifdef __linux__
    if (DeviceHandle) {
        printf("Taking exclusive USB control\n");
        hid_set_nonblocking(DeviceHandle, 0);  // Switch to blocking mode
        StopHidReadThread();

        // More aggressive flush of pending data
        unsigned char flush_buffer[HIDREPORTNUM];
        int flush_count = 0;
        while (hid_read_timeout(DeviceHandle, flush_buffer, HIDREPORTNUM, 1) > 0) {
            flush_count++;
            if (flush_count > 100) break;  // Prevent infinite loop
        }
        if (flush_count > 0) {
            printf("Flushed %d packets of stale data\n", flush_count);
        }

        // Give device time to stabilize with longer delay
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
#endif

    // Initialize frame data to zero
    for (int i = 0; i < MAX_IMAGE_SIZE; i++) {
        for (int j = 0; j < MAX_IMAGE_SIZE; j++) {
            frame_data[i][j] = 0;
        }
    }

    // We'll try multiple capture approaches
    const int NUM_APPROACHES = 4;
    uint8_t packet_types[NUM_APPROACHES] = { 0x02, 0x12, 0x22, 0x32 };
    
    for (int approach = 0; approach < NUM_APPROACHES; approach++) {
        printf("\n==== TRYING CAPTURE APPROACH %d (PACKET TYPE 0x%02X) ====\n", 
               approach + 1, packet_types[approach]);
        
        // Clear any pending input before sending command
        std::memset(RxData, 0, sizeof(RxData));

        // Send capture command with the current packet type
        TxData[0] = 0xaa;      // preamble code
        TxData[1] = 0x02;      // command
        TxData[2] = 0x0C;      // data length
        TxData[3] = ((chan-1) << 4) | packet_types[approach];  // Try different packet types
        TxData[4] = 0xff;      // request all rows
        TxData[5] = 0x00;
        
        // Zero out the rest
        for (int i = 6; i < 15; i++) TxData[i] = 0x00;
        
        // Calculate checksum
        TxData[15] = 0;
        for (int i = 1; i <= 14; i++) {
            TxData[15] += TxData[i];
        }
        if (TxData[15] == 0x17) 
            TxData[15] = 0x18;
        
        TxData[16] = 0x17;     // back code
        TxData[17] = 0x17;     // back code
        
        // Send command
        WriteHIDOutputReport();
        std::memset(TxData, 0, sizeof(TxData));

        // Wait for device to start processing the command
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Set continue flag for capture loop
        Continue_Flag = true;
        int consecutive_timeouts = 0;
        const int MAX_CONSECUTIVE_TIMEOUTS = 8;
        
        // Row capture loop 
        const int MAX_READS_PER_APPROACH = 50;
        int reads = 0;
        int new_rows_this_approach = 0;
        
        printf("Reading rows using packet type 0x%02X...\n", packet_types[approach]);

        while (Continue_Flag && reads < MAX_READS_PER_APPROACH && total_rows < 12) {
            // Enhanced read with better error reporting
            bool success = ReadHIDInputReportTimeout(HIDREPORTNUM, 150);

            if (!success) {
                consecutive_timeouts++;
                if (consecutive_timeouts > MAX_CONSECUTIVE_TIMEOUTS) {
                    printf("Breaking after %d consecutive timeouts\n", consecutive_timeouts);
                    break;
                }
                printf("No data received (timeout %d/%d)\n", consecutive_timeouts, MAX_CONSECUTIVE_TIMEOUTS);
                continue;
            }

            // Reset timeout counter when we get data
            consecutive_timeouts = 0;

            // Extract packet information 
            uint8_t cmd_type = RxData[2];
            uint8_t row_type = RxData[4];
            uint8_t row = RxData[5];

            // Process data
            if (cmd_type == 0x1c) {
                int actual_row = row_type;

                if (actual_row >= 0 && actual_row < 12) {
                    ProcessRowData();

                    // Track new rows
                    if (!rows_received[actual_row]) {
                        rows_received[actual_row] = true;
                        total_rows++;
                        new_rows_this_approach++;
                        printf("Got row %d (%d/12 total) - %s row [approach %d]\n", 
                            actual_row, total_rows, 
                            (actual_row % 2 == 0) ? "EVEN" : "ODD",
                            approach + 1);
                    }
                    else {
                        printf("Duplicate row %d\n", actual_row);
                    }
                }
            }
            else if (cmd_type == 0x02) {
                // Legacy format - extract row from a different field
                uint8_t frameFormat = row_type & 0x0F;

                if ((frameFormat == 0x02 || frameFormat == 0x22) && row < 12) {
                    printf("Processing legacy format data packet\n");
                    ProcessRowData();

                    if (!rows_received[row]) {
                        rows_received[row] = true;
                        total_rows++;
                        new_rows_this_approach++;
                        printf("Got row %d from legacy format (%d/12 total) - %s row\n", 
                            row, total_rows, 
                            (row % 2 == 0) ? "EVEN" : "ODD");
                    }
                }
            }
            else if (cmd_type == 0x01) {
                printf("Command acknowledgement received\n");
            }

            // Clear RxData after processing
            std::memset(RxData, 0, sizeof(RxData));
            reads++;
        }
        
        printf("\nApproach %d results: %d new rows received (total: %d/12)\n", 
               approach + 1, new_rows_this_approach, total_rows);
        
        // Show which rows we have so far
        printf("Rows received so far: ");
        for (int i = 0; i < 12; i++) {
            if (rows_received[i]) printf("%d ", i);
        }
        printf("\n");
        
        // If we have all rows, no need to try other approaches
        if (total_rows == 12) {
            printf("All rows received, no need to try additional capture approaches\n");
            break;
        }
        
        // Wait between approaches
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // Final report
    printf("\n==== CAPTURE SUMMARY ====\n");
    printf("Final result: received %d/12 rows\n", total_rows);
    printf("Rows received: ");
    for (int i = 0; i < 12; i++) {
        if (rows_received[i]) printf("%d ", i);
    }
    printf("\n");

    // Show missing rows
    printf("Missing rows: ");
    for (int i = 0; i < 12; i++) {
        if (!rows_received[i]) printf("%d ", i);
    }
    printf("\n");

    // Check for pattern in missing rows
    int even_count = 0, odd_count = 0;
    for (int i = 0; i < 12; i += 2) {
        if (rows_received[i]) even_count++;
        if (i+1 < 12 && rows_received[i+1]) odd_count++;
    }
    printf("Received %d/6 even rows and %d/6 odd rows\n", even_count, odd_count);

    // Fill in missing rows if needed
    if (total_rows < 12) {
        printf("\n==== FILLING MISSING ROWS ====\n");
        for (int i = 0; i < 12; i++) {
            if (!rows_received[i]) {
                // Find adjacent rows
                int prev = i - 1;
                while (prev >= 0 && !rows_received[prev]) prev--;

                int next = i + 1;
                while (next < 12 && !rows_received[next]) next++;

                if (prev >= 0 && next < 12 && rows_received[prev] && rows_received[next]) {
                    printf("Interpolating row %d between rows %d and %d\n", i, prev, next);
                    // Weighted average interpolation
                    float prev_weight = (float)(next - i) / (next - prev);
                    float next_weight = (float)(i - prev) / (next - prev);

                    for (int j = 0; j < 12; j++) {
                        frame_data[i][j] = (int)(prev_weight * frame_data[prev][j] +
                            next_weight * frame_data[next][j]);
                    }
                }
                else if (prev >= 0 && rows_received[prev]) {
                    printf("Copying from previous row %d to %d\n", prev, i);
                    for (int j = 0; j < 12; j++) {
                        frame_data[i][j] = frame_data[prev][j];
                    }
                }
                else if (next < 12 && rows_received[next]) {
                    printf("Copying from next row %d to %d\n", next, i);
                    for (int j = 0; j < 12; j++) {
                        frame_data[i][j] = frame_data[next][j];
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

    return 0;  // Return success
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

bool CInterfaceObject::ResetUSBEndpoints() {
#ifdef __linux__
    if (!DeviceHandle) return false;

    printf("Attempting to reset USB endpoints...\n");

    // First close and reopen the device
    hid_close(DeviceHandle);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    DeviceHandle = hid_open(VENDOR_ID, PRODUCT_ID, nullptr);

    if (!DeviceHandle) {
        printf("Failed to reopen device after reset\n");
        g_DeviceDetected = false;
        return false;
    }

    // Set non-blocking mode
    hid_set_nonblocking(DeviceHandle, 0);

    // Flush any pending data
    unsigned char flush_buffer[HIDREPORTNUM];
    int flush_count = 0;
    while (hid_read_timeout(DeviceHandle, flush_buffer, HIDREPORTNUM, 5) > 0) {
        flush_count++;
        if (flush_count > 50) break;
    }

    printf("Reset complete, flushed %d packets\n", flush_count);
    return true;
#else
    return true;  // Nothing to do on Windows
#endif
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

// New function to request complete frame data with all rows
void CInterfaceObject::CompleteCapture12(uint8_t chan)
{
    if (chan < 1 || chan > 4)
        return;

    chan -= 1; // Convert to 0-based for internal use

    // Format a command packet that requests ALL rows (not just odd rows)
    TxData[0] = 0xaa;      // preamble code
    TxData[1] = 0x02;      // command
    TxData[2] = 0x0C;      // data length
    TxData[3] = (chan << 4) | 0x12;  // Try different type (0x12 instead of 0x02)
    TxData[4] = 0xff;      // request all rows
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
    
    // Calculate checksum
    TxData[15] = 0;
    for (int i = 1; i <= 14; i++) {
        TxData[15] += TxData[i];
    }
    if (TxData[15] == 0x17) 
        TxData[15] = 0x18;
    
    TxData[16] = 0x17;     // back code
    TxData[17] = 0x17;     // back code

    printf("\n==== SENDING COMPLETE FRAME REQUEST (ALL ROWS) ====\n");
    printf("Using packet type 0x12 instead of 0x02\n");
}