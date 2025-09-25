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
        // Create a temporary buffer of the correct type for TrimReader
        int temp_frame_data[MAX_IMAGE_SIZE][24] = {0};
        
        // Copy our frame data to the temporary buffer
        for (int i = 0; i < MAX_IMAGE_SIZE; i++) {
            for (int j = 0; j < MAX_IMAGE_SIZE; j++) {
                temp_frame_data[i][j] = frame_data[i][j];
            }
        }
        
        // Call the TrimReader's ProcessRowData with the correct type
        frame_size = m_TrimReader.ProcessRowData(temp_frame_data, gain_mode);
        
        // Copy the processed data back to our frame data array
        for (int i = 0; i < MAX_IMAGE_SIZE; i++) {
            for (int j = 0; j < MAX_IMAGE_SIZE; j++) {
                frame_data[i][j] = temp_frame_data[i][j];
            }
        }
    }
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
    printf("Starting dual-pass capture for channel %d (kernel buffer workaround)\n", chan);
    
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
        printf("Taking exclusive USB control for kernel buffer workaround\n");
        
        // Stop the read thread to ensure direct control
        StopHidReadThread();
        
        // Set to blocking mode 
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

    // PASS 1: Get odd rows
    printf("\n==== PASS 1: CAPTURING ODD ROWS ====\n");
    
    // Format command for first pass
    TxData[0] = 0xaa;      // preamble code
    TxData[1] = 0x02;      // command
    TxData[2] = 0x0C;      // data length
    TxData[3] = ((chan-1) << 4) | 0x02;  // Standard format used in Windows
    TxData[4] = 0xff;      // request all rows (Windows uses this value)
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
    bool sent = WriteHIDOutputReport();
    if (!sent) {
        printf("Failed to send capture command\n");
    }
    std::memset(TxData, 0, sizeof(TxData));

    // Wait for device to process
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Read whatever rows we can get (typically odd rows + row 0)
    int max_reads_pass1 = 20;
    int timeouts = 0;
    
    for (int read_count = 0; read_count < max_reads_pass1 && timeouts < 3; read_count++) {
        unsigned char buffer[HIDREPORTNUM] = { 0 };
        int res = hid_read_timeout(DeviceHandle, buffer, HIDREPORTNUM, 200);
        
        if (res > 0) {
            timeouts = 0; // Reset timeout counter
            
            // Copy to RxData
            std::memcpy(RxData, &buffer[1], std::min(static_cast<size_t>(RxNum), static_cast<size_t>(HIDREPORTNUM - 1)));
            
            // Extract packet info
            uint8_t cmd_type = RxData[2];
            uint8_t row_type = RxData[4];
            uint8_t row_idx = RxData[5];
            
            // Process row data if valid
            if ((cmd_type == 0x1c || cmd_type == 0x02) && row_idx < 12) {
                // Process the data
                for (int i = 0; i < 12; i++) {
                    uint8_t high_byte = RxData[6 + i * 2];
                    uint8_t low_byte = RxData[7 + i * 2];
                    uint16_t value = (high_byte << 8) | low_byte;
                    frame_data[row_idx][i] = value;
                }
                
                // Mark row as received
                if (!rows_received[row_idx]) {
                    rows_received[row_idx] = true;
                    total_rows++;
                    printf("Got row %d (%d/12 total) - %s row\n", 
                           row_idx, total_rows,
                           (row_idx % 2 == 0) ? "EVEN" : "ODD");
                }
            }
            
            // Clear RxData for next read
            std::memset(RxData, 0, sizeof(RxData));
        }
        else {
            // No data received
            timeouts++;
            printf("No data received (timeout %d/3)\n", timeouts);
        }
    }
    
    // Print which rows we've received so far
    printf("\nPass 1 complete. Received %d/12 rows\n", total_rows);
    printf("Rows received: ");
    for (int i = 0; i < 12; i++) {
        if (rows_received[i]) printf("%d ", i);
    }
    printf("\n");

    // PASS 2: Request each missing row individually, working around kernel buffer limits
    printf("\n==== PASS 2: REQUESTING MISSING ROWS INDIVIDUALLY ====\n");
    printf("Working around kernel buffer limitation by requesting one row at a time\n");
    
    // Create a list of missing rows
    std::vector<int> missing_rows;
    for (int i = 0; i < 12; i++) {
        if (!rows_received[i]) {
            missing_rows.push_back(i);
        }
    }
    
    if (!missing_rows.empty()) {
        printf("Missing rows to request individually: ");
        for (int row : missing_rows) {
            printf("%d ", row);
        }
        printf("\n");
        
        // For each missing row, perform a complete cycle:
        // 1. Reset USB to clear buffers
        // 2. Send targeted request
        // 3. Read with timeout
        // 4. Process response
        for (int missing_row : missing_rows) {
            printf("\n==== REQUESTING ROW %d ====\n", missing_row);
            
            // Reset USB endpoints to clear any stale buffers
            #ifdef __linux__
            ResetUSBEndpoints();
            #endif
            
            // Short delay
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            
            // Create a command specifically for this row
            TxData[0] = 0xaa;      // preamble code
            TxData[1] = 0x02;      // command
            TxData[2] = 0x0C;      // data length
            TxData[3] = ((chan-1) << 4) | 0x42;  // Special type for direct row request
            TxData[4] = missing_row;  // Request specific row
            TxData[5] = 0x01;      // Flag for specific row
            
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
            printf("Sending targeted request for row %d\n", missing_row);
            WriteHIDOutputReport();
            std::memset(TxData, 0, sizeof(TxData));
            
            // Wait for response
            bool got_row = false;
            int attempts = 0;
            const int MAX_ATTEMPTS = 5;
            
            while (!got_row && attempts < MAX_ATTEMPTS) {
                unsigned char buffer[HIDREPORTNUM] = { 0 };
                int res = hid_read_timeout(DeviceHandle, buffer, HIDREPORTNUM, 200);
                
                if (res > 0) {
                    // Copy to RxData
                    std::memcpy(RxData, &buffer[1], std::min(static_cast<size_t>(RxNum), static_cast<size_t>(HIDREPORTNUM - 1)));
                    
                    // Extract packet info
                    uint8_t cmd_type = RxData[2];
                    uint8_t row_idx = RxData[5];  // In direct request response
                    
                    printf("Got response with cmd=0x%02X, row=0x%02X (attempt %d)\n", 
                           cmd_type, row_idx, attempts+1);
                    
                    // Check if this is the row we requested
                    if ((cmd_type == 0x1c || cmd_type == 0x02) && row_idx == missing_row) {
                        // Process the data
                        for (int i = 0; i < 12; i++) {
                            uint8_t high_byte = RxData[6 + i * 2];
                            uint8_t low_byte = RxData[7 + i * 2];
                            uint16_t value = (high_byte << 8) | low_byte;
                            frame_data[missing_row][i] = value;
                        }
                        
                        rows_received[missing_row] = true;
                        total_rows++;
                        printf("Successfully received row %d (%d/12 total)\n", missing_row, total_rows);
                        
                        // Print first few values for verification
                        printf("Row %d values: ", missing_row);
                        for (int i = 0; i < 3; i++) {
                            printf("%d ", frame_data[missing_row][i]);
                        }
                        printf("...\n");
                        
                        got_row = true;
                    }
                    else {
                        printf("Received unexpected row or packet, continuing to next attempt\n");
                    }
                    
                    // Clear RxData for next read
                    std::memset(RxData, 0, sizeof(RxData));
                }
                else {
                    printf("No response on attempt %d\n", attempts+1);
                }
                
                attempts++;
            }
            
            if (!got_row) {
                printf("Failed to receive row %d after %d attempts\n", missing_row, MAX_ATTEMPTS);
            }
            
            // Wait between rows to let device recover
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    // Final report
    printf("\n==== KERNEL BUFFER WORKAROUND COMPLETE ====\n");
    printf("Final result: received %d/12 rows\n", total_rows);
    printf("Rows received: ");
    for (int i = 0; i < 12; i++) {
        if (rows_received[i]) printf("%d ", i);
    }
    printf("\n");
    
    // Fill in any missing rows if needed
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

// Add a new capture function using direct USB access
int CInterfaceObject::DirectUSBCapture12(uint8_t chan)
{
    printf("\n============================================================\n");
    printf("Starting LIBUSB direct capture for channel %d\n", chan);
    printf("============================================================\n");
    
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

    // Initialize direct USB if not already done
    if (!g_DirectUSB.IsConnected()) {
        printf("Initializing libusb connection...\n");
        bool success = g_DirectUSB.Initialize();
        
        if (!success) {
            printf("First initialization attempt failed, trying alternative methods...\n");
            success = g_DirectUSB.TryAlternativeAccess();
        }
        
        if (!success) {
            printf("Failed to initialize libusb connection after all attempts\n");
            return -1;
        }
        
        printf("Libusb connection established\n");
        
        // Dump detailed USB information
        g_DirectUSB.DumpDeviceInfo();
        g_DirectUSB.PrintEndpointInfo();
        g_DirectUSB.DumpRawDescriptors();
    }

    // Enable verbose logging
    g_DirectUSB.SetVerboseLogging(true);

    // Reset sequence
    printf("\n==== DEVICE RESET SEQUENCE ====\n");
    uint8_t resetCmd[64] = {0};
    resetCmd[0] = 0xaa;  // preamble
    resetCmd[1] = 0x01;  // command type (control command)
    resetCmd[2] = 0x02;  // data length
    resetCmd[3] = 0x01;  // reset command
    resetCmd[4] = 0x00;  // parameter
    resetCmd[15] = resetCmd[1] + resetCmd[2] + resetCmd[3] + resetCmd[4]; // checksum
    if (resetCmd[15] == 0x17) resetCmd[15] = 0x18;
    resetCmd[16] = 0x17;  // back code
    resetCmd[17] = 0x17;  // back code
    
    g_DirectUSB.SendReport(resetCmd, sizeof(resetCmd));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // Flush any responses
    std::vector<uint8_t> report;
    while(g_DirectUSB.GetNextReport(report, 50)) {
        // Responses are automatically logged by the DirectUSB class
    }
    
    // Start async read thread for high-speed capture
    g_DirectUSB.StartAsyncRead();

    // Try the Windows-style capture first
    printf("\n==== ATTEMPTING CAPTURE WITH WINDOWS PROTOCOL ====\n");
    
    // Format command for capturing all rows
    uint8_t frameCmd[64] = {0};
    frameCmd[0] = 0xaa;      // preamble code
    frameCmd[1] = 0x02;      // command
    frameCmd[2] = 0x0C;      // data length
    frameCmd[3] = ((chan-1) << 4) | 0x02;  // Standard Windows format
    frameCmd[4] = 0xff;      // request all rows
    frameCmd[5] = 0x00;
    
    // Calculate checksum
    frameCmd[15] = 0;
    for (int i = 1; i <= 14; i++) {
        frameCmd[15] += frameCmd[i];
    }
    if (frameCmd[15] == 0x17) 
        frameCmd[15] = 0x18;
    
    frameCmd[16] = 0x17;     // back code
    frameCmd[17] = 0x17;     // back code
    
    // Send command
    g_DirectUSB.SendReport(frameCmd, sizeof(frameCmd));
    
    // Wait for responses with a 10 second timeout
    printf("\nWaiting for responses (10 second timeout)...\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Wait for device to process
    
    auto start_time = std::chrono::high_resolution_clock::now();
    int responses = 0;
    
    while (true) {
        // Check timeout
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time);
        if (elapsed.count() >= 10) {
            printf("10 second timeout reached after %d responses\n", responses);
            break;
        }
        
        // Try to get a response
        if (g_DirectUSB.GetNextReport(report, 500)) {
            responses++;
            
            // Basic packet analysis - DirectUSB already logs the raw data
            if (report.size() > 5) {
                if (report[0] == 0xAA && (report[2] == 0x02 || report[2] == 0x1C)) {
                    // Check if this is a row data packet
                    if (report.size() >= 30 && report[5] < 12) {
                        // Process the row data
                        uint8_t row_idx = report[5];
                        
                        for (int i = 0; i < 12; i++) {
                            uint8_t high_byte = report[6 + i * 2];
                            uint8_t low_byte = report[7 + i * 2];
                            uint16_t value = (high_byte << 8) | low_byte;
                            frame_data[row_idx][i] = value;
                        }
                        
                        // Mark row as received
                        if (!rows_received[row_idx]) {
                            rows_received[row_idx] = true;
                            total_rows++;
                            printf("Successfully processed row %d data (%d/12 total)\n", 
                                   row_idx, total_rows);
                        }
                    }
                }
            }
        }
        else {
            // Break if we've received responses but nothing new is coming
            if (responses > 0) {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                if (!g_DirectUSB.GetNextReport(report, 500)) {
                    printf("No further responses after 1.5 seconds, continuing\n");
                    break;
                }
            }
        }
    }

    // Try to get any missing rows directly
    for (int row = 0; row < 12; row++) {
        if (rows_received[row]) continue; // Skip rows we already have
        
        printf("\n==== REQUESTING ROW %d DIRECTLY ====\n", row);
        
        uint8_t directCmd[64] = {0};
        directCmd[0] = 0xaa;      // preamble code
        directCmd[1] = 0x02;      // command
        directCmd[2] = 0x0C;      // data length
        directCmd[3] = ((chan-1) << 4) | 0x42;  // Special type for direct row request
        directCmd[4] = row;       // Request specific row
        directCmd[5] = 0x01;      // Flag for specific row
        
        // Calculate checksum
        directCmd[15] = 0;
        for (int i = 1; i <= 14; i++) {
            directCmd[15] += directCmd[i];
        }
        if (directCmd[15] == 0x17) 
            directCmd[15] = 0x18;
        
        directCmd[16] = 0x17;     // back code
        directCmd[17] = 0x17;     // back code
        
        // Send command
        g_DirectUSB.SendReport(directCmd, sizeof(directCmd));
        
        // Wait for response with timeout
        int row_responses = 0;
        bool got_row = false;
        auto row_start_time = std::chrono::high_resolution_clock::now();
        
        while (!got_row) {
            // Check timeout (3 seconds per row)
            auto now = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - row_start_time);
            if (elapsed.count() >= 3) {
                printf("3 second timeout reached for row %d\n", row);
                break;
            }
            
            if (g_DirectUSB.GetNextReport(report, 300)) {
                row_responses++;
                
                // Check if this is our row data
                if (report.size() > 5 && report[0] == 0xAA && 
                   (report[2] == 0x02 || report[2] == 0x1C) && report[5] == row) {
                    
                    // Process the row data
                    for (int i = 0; i < 12; i++) {
                        uint8_t high_byte = report[6 + i * 2];
                        uint8_t low_byte = report[7 + i * 2];
                        uint16_t value = (high_byte << 8) | low_byte;
                        frame_data[row][i] = value;
                    }
                    
                    rows_received[row] = true;
                    total_rows++;
                    printf("Successfully received row %d (%d/12 total)\n", row, total_rows);
                    got_row = true;
                }
            }
            else if (row_responses > 0) {
                // If we got some responses but not our row, move on
                break;
            }
        }
        
        // Wait between row requests
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // Stop the async read thread
    g_DirectUSB.StopAsyncRead();
    
    // Report results
    printf("\n==== LIBUSB DIRECT CAPTURE COMPLETE ====\n");
    printf("Received %d/12 rows\n", total_rows);
    printf("Rows received: ");
    for (int i = 0; i < 12; i++) {
        if (rows_received[i]) printf("%d ", i);
    }
    printf("\n");
    
    // Fill in any missing rows with interpolation
    if (total_rows < 12 && total_rows > 0) {
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
    
    return total_rows;
}

// Update CaptureFrame12 to use the new detailed USB capture
int CInterfaceObject::CaptureFrame12(uint8_t chan)
{
    // Try the detailed direct USB capture first
    int direct_result = DirectUSBCapture12(chan);
    
    // If direct USB failed completely, fall back to the Windows style
    if (direct_result <= 0) {
        printf("\nDirect USB capture failed, falling back to Windows style capture...\n");
        return WindowsStyleCapture12(chan);
    }
    
    return direct_result;
}