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
    // Simply call the Windows-compatible version
    return WindowsStyleCapture12(chan);
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

// Add this function to directly copy the Windows implementation approach
int CInterfaceObject::WindowsStyleCapture12(uint8_t chan)
{
    printf("Starting Windows-compatible capture for channel %d\n", chan);
    
    // Print capture parameters
    printf("Capture parameters: Channel=%d, Gain=%d, IntTime=%.2f\n", 
           chan, gain_mode, int_time);

    // Initialize tracking for each row
    bool rows_received[12] = { false };
    int total_rows = 0;
    
    // Initialize frame data to zero
    for (int i = 0; i < MAX_IMAGE_SIZE; i++) {
        for (int j = 0; j < MAX_IMAGE_SIZE; j++) {
            frame_data[i][j] = 0;
        }
    }

#ifdef __linux__
    if (DeviceHandle) {
        printf("Taking exclusive USB control with Windows-compatible mode\n");
        
        // Stop the read thread to ensure direct control
        StopHidReadThread();
        
        // Set to blocking mode with longer timeout
        hid_set_nonblocking(DeviceHandle, 0);

        // More aggressive flush of pending data
        unsigned char flush_buffer[HIDREPORTNUM];
        int flush_count = 0;
        while (hid_read_timeout(DeviceHandle, flush_buffer, HIDREPORTNUM, 1) > 0) {
            flush_count++;
            if (flush_count > 100) break;
        }
        if (flush_count > 0) {
            printf("Flushed %d packets of stale data\n", flush_count);
        }
        
        // Wait for device to stabilize
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
#endif

    // Clear RxData before starting
    std::memset(RxData, 0, sizeof(RxData));

    // Prepare the exact same command as Windows version
    printf("\n==== SENDING WINDOWS-COMPATIBLE CAPTURE COMMAND ====\n");
    
    // Format command exactly like Windows (verified from logs)
    TxData[0] = 0xaa;      // preamble code
    TxData[1] = 0x02;      // command
    TxData[2] = 0x0C;      // data length
    TxData[3] = ((chan-1) << 4) | 0x02;  // Standard format used in Windows
    TxData[4] = 0xff;      // request all rows (Windows uses this value)
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
    
    // Calculate checksum (same as Windows)
    TxData[15] = 0;
    for (int i = 1; i <= 14; i++) {
        TxData[15] += TxData[i];
    }
    if (TxData[15] == 0x17) 
        TxData[15] = 0x18;
    
    TxData[16] = 0x17;     // back code
    TxData[17] = 0x17;     // back code

    // Print the command for verification
    printf("Sending command: ");
    for (int i = 0; i < 18; i++) {
        printf("%02X ", TxData[i]);
    }
    printf("\n");
    
    // Send command with forced flush and timing
    bool sent = WriteHIDOutputReport();
    if (!sent) {
        printf("Failed to send capture command\n");
    }
    std::memset(TxData, 0, sizeof(TxData));

    // Wait for device to process - longer wait like Windows does
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Row capture loop with Windows-compatible timing
    printf("Reading rows with Windows-compatible timing...\n");
    
    // In Windows, rows come in without any special requests
    // Wait for each row with appropriate timeout
    int expected_rows = 12;
    int max_attempts = 50; // Higher than needed to ensure we don't miss data
    
    for (int attempt = 0; attempt < max_attempts && total_rows < expected_rows; attempt++) {
        // Each read has its own timeout like Windows
        unsigned char buffer[HIDREPORTNUM] = { 0 };
        int res = hid_read_timeout(DeviceHandle, buffer, HIDREPORTNUM, 250); // longer timeout
        
        if (res > 0) {
            // Copy the data from buffer to RxData
            std::memcpy(RxData, &buffer[1], std::min(static_cast<size_t>(RxNum), static_cast<size_t>(HIDREPORTNUM - 1)));
            
            // Print the received data
            printf("Received packet: ");
            for (int i = 0; i < 10; i++) { // Just print first 10 bytes
                printf("%02X ", buffer[i]);
            }
            printf("...\n");
            
            // Process the data based on the format in Windows logs
            uint8_t cmd_type = RxData[2];
            uint8_t row_type = RxData[4];
            uint8_t row_idx = RxData[5]; // In Windows format, row index is here
            
            if (cmd_type == 0x1c || cmd_type == 0x02) {
                // Windows format rows
                int row = row_idx;
                
                if (row >= 0 && row < 12) {
                    // Process the data like Windows does
                    for (int i = 0; i < 12; i++) {
                        // Windows format - data starts at offset 6
                        uint8_t high_byte = RxData[6 + i * 2];
                        uint8_t low_byte = RxData[7 + i * 2];
                        
                        // Combine into a 16-bit value
                        uint16_t value = (high_byte << 8) | low_byte;
                        
                        // Store in frame data
                        frame_data[row][i] = value;
                    }
                    
                    // Track new row
                    if (!rows_received[row]) {
                        rows_received[row] = true;
                        total_rows++;
                        printf("Got row %d (%d/12 total) with values: %d %d %d ...\n", 
                               row, total_rows, 
                               frame_data[row][0], frame_data[row][1], frame_data[row][2]);
                    } else {
                        printf("Duplicate row %d\n", row);
                    }
                }
            }
            
            // Clear RxData for next read
            std::memset(RxData, 0, sizeof(RxData));
            
            // Reset attempt counter when we get data
            attempt = 0;
        } else {
            // No data received - wait a bit like Windows does
            printf("No data on attempt %d\n", attempt);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    // Report results
    printf("\n==== WINDOWS-COMPATIBLE CAPTURE COMPLETE ====\n");
    printf("Received %d/12 rows\n", total_rows);
    
    // Print which rows we got
    printf("Rows received: ");
    for (int i = 0; i < 12; i++) {
        if (rows_received[i]) printf("%d ", i);
    }
    printf("\n");
    
    // Fill in any missing rows
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

    return 0;
}