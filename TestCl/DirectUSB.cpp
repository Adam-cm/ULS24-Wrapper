// Copyright 2023, All rights reserved

#include "DirectUSB.h"
#include <cstdio>
#include <cstring>
#include <chrono>
#include <iomanip>
#include <sstream>

#ifdef __linux__
#include <dlfcn.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#endif

// Global instance
DirectUSB g_DirectUSB;

DirectUSB::DirectUSB() : m_Running(false) {
    // Default to a higher level of debug/diagnostics
    m_VerboseLogging = true;
}

DirectUSB::~DirectUSB() {
    StopAsyncRead();
    
    if (m_RawDevice != -1) {
        #ifdef __linux__
        close(m_RawDevice);
        m_RawDevice = -1;
        #endif
    }
    
    if (m_DeviceHandle) {
        // Release interface and close device
        try {
            libusb_release_interface(m_DeviceHandle, m_Interface);
            libusb_close(m_DeviceHandle);
        } catch (...) {
            printf("Warning: Exception during device cleanup\n");
        }
        m_DeviceHandle = nullptr;
    }
    
    if (m_Context) {
        libusb_exit(m_Context);
        m_Context = nullptr;
    }
}

// Print detailed info about all USB devices for diagnostics
void DirectUSB::ListAllUSBDevices() {
    printf("\n====== LISTING ALL USB DEVICES ======\n");
    
    if (!m_Context) {
        if (libusb_init(&m_Context) != 0) {
            printf("Failed to initialize libusb for device listing\n");
            return;
        }
    }
    
    libusb_device **devs;
    ssize_t cnt = libusb_get_device_list(m_Context, &devs);
    if (cnt < 0) {
        printf("Failed to get USB device list\n");
        return;
    }
    
    printf("Found %ld USB devices:\n\n", cnt);
    
    for (ssize_t i = 0; i < cnt; i++) {
        libusb_device *dev = devs[i];
        libusb_device_descriptor desc;
        
        if (libusb_get_device_descriptor(dev, &desc) != 0) {
            printf("  %ld: Failed to get device descriptor\n", i);
            continue;
        }
        
        printf("  Device %ld:\n", i);
        printf("    Bus: %03d Device: %03d\n", 
               libusb_get_bus_number(dev), 
               libusb_get_device_address(dev));
        printf("    VID: 0x%04X, PID: 0x%04X\n", desc.idVendor, desc.idProduct);
        printf("    Class: 0x%02X, SubClass: 0x%02X, Protocol: 0x%02X\n", 
               desc.bDeviceClass, desc.bDeviceSubClass, desc.bDeviceProtocol);
        printf("    USB Version: %d.%d\n", desc.bcdUSB >> 8, desc.bcdUSB & 0xFF);
        
        // If this might be our device, mark it
        if (desc.idVendor == VENDOR_ID) {
            if (desc.idProduct == PRODUCT_ID) {
                printf("    *** THIS IS OUR TARGET DEVICE! ***\n");
            } else {
                printf("    *** SAME VENDOR AS OUR TARGET DEVICE ***\n");
            }
        }
        
        // Try to open the device to get more info
        libusb_device_handle *handle;
        if (libusb_open(dev, &handle) == 0) {
            unsigned char string_data[256];
            
            if (desc.iManufacturer > 0) {
                if (libusb_get_string_descriptor_ascii(handle, desc.iManufacturer, 
                                                     string_data, sizeof(string_data)) > 0) {
                    printf("    Manufacturer: %s\n", string_data);
                }
            }
            
            if (desc.iProduct > 0) {
                if (libusb_get_string_descriptor_ascii(handle, desc.iProduct, 
                                                     string_data, sizeof(string_data)) > 0) {
                    printf("    Product: %s\n", string_data);
                }
            }
            
            if (desc.iSerialNumber > 0) {
                if (libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, 
                                                     string_data, sizeof(string_data)) > 0) {
                    printf("    Serial: %s\n", string_data);
                }
            }
            
            libusb_close(handle);
        }
        
        printf("\n");
    }
    
    libusb_free_device_list(devs, 1);
    printf("====== END OF USB DEVICE LIST ======\n\n");
}

bool DirectUSB::Initialize() {
    printf("\n====== INITIALIZING LIBUSB ======\n");

    // Check for available devices first
    ListAllUSBDevices();
    
    // Attempt to check permissions on Linux
    #ifdef __linux__
    printf("\nChecking system permissions...\n");
    
    // Check if we're running as root
    if (geteuid() != 0) {
        printf("WARNING: Not running as root. USB access may be restricted.\n");
        
        // Check USB device node permissions
        struct stat st;
        if (stat("/dev/bus/usb", &st) == 0) {
            printf("USB bus directory exists. Permissions: %o\n", st.st_mode & 0777);
            if ((st.st_mode & S_IROTH) == 0 || (st.st_mode & S_IWOTH) == 0) {
                printf("WARNING: /dev/bus/usb may not be readable/writable by non-root users.\n");
                printf("Consider running with sudo or setting udev rules.\n");
            }
        } else {
            printf("Cannot access /dev/bus/usb - permissions may be restricted\n");
        }
    } else {
        printf("Running as root. Full USB permissions available.\n");
    }
    #endif

    // Initialize libusb
    int result = libusb_init(&m_Context);
    if (result != 0) {
        printf("Failed to initialize libusb: %s\n", libusb_error_name(result));
        return false;
    }
    
    // Set maximum debug level for troubleshooting
    libusb_set_option(m_Context, LIBUSB_OPTION_LOG_LEVEL, 3);
    
    // First, try standard method
    printf("\nAttempting standard device open with VID=0x%04X, PID=0x%04X\n", VENDOR_ID, PRODUCT_ID);
    
    m_DeviceHandle = libusb_open_device_with_vid_pid(m_Context, VENDOR_ID, PRODUCT_ID);
    if (!m_DeviceHandle) {
        printf("Standard device open failed\n");
        return TryAlternativeMethods();
    }
    
    printf("Successfully opened device using standard method\n");
    
    // Print device info
    DumpDeviceInfo();
    
    // Automatically detach kernel driver if needed
    libusb_set_auto_detach_kernel_driver(m_DeviceHandle, 1);
    
    // Get and print device configuration
    int config = -1;
    if (libusb_get_configuration(m_DeviceHandle, &config) == 0) {
        printf("Device is using configuration: %d\n", config);
        
        if (config == 0) {
            printf("Device is unconfigured. Setting configuration 1...\n");
            if (libusb_set_configuration(m_DeviceHandle, 1) != 0) {
                printf("Failed to set configuration, but continuing anyway\n");
            }
        }
    }

    // Try claiming interface
    printf("Attempting to claim interface %d...\n", m_Interface);
    result = libusb_claim_interface(m_DeviceHandle, m_Interface);
    if (result != 0) {
        printf("Failed to claim interface %d: %s\n", m_Interface, libusb_error_name(result));
        printf("Attempting alternate interfaces...\n");
        
        bool claimed = false;
        for (int i = 0; i < 4 && !claimed; i++) {
            if (i == m_Interface) continue;
            
            printf("Trying interface %d...\n", i);
            if (libusb_kernel_driver_active(m_DeviceHandle, i)) {
                printf("Detaching kernel driver from interface %d\n", i);
                libusb_detach_kernel_driver(m_DeviceHandle, i);
            }
            
            if (libusb_claim_interface(m_DeviceHandle, i) == 0) {
                printf("Successfully claimed interface %d\n", i);
                m_Interface = i;
                claimed = true;
            }
        }
        
        if (!claimed) {
            printf("Failed to claim any interface\n");
            libusb_close(m_DeviceHandle);
            m_DeviceHandle = nullptr;
            return TryAlternativeMethods();
        }
    } else {
        printf("Successfully claimed interface %d\n", m_Interface);
    }
    
    // Find endpoints
    if (!FindEndpoints()) {
        printf("Failed to find suitable endpoints\n");
        printf("Trying with default endpoint addresses...\n");
        
        m_InputEndpoint = 0x81;
        m_OutputEndpoint = 0x01;
    }
    
    printf("Using IN endpoint: 0x%02X, OUT endpoint: 0x%02X\n", 
           m_InputEndpoint, m_OutputEndpoint);

    // Try a device reset for good measure (can help with confused devices)
    printf("Resetting device to ensure clean state...\n");
    if (libusb_reset_device(m_DeviceHandle) == 0) {
        printf("Device reset successful\n");
    } else {
        printf("Device reset failed, but continuing anyway\n");
    }
    
    // Try sending a control transfer to further wake up the device
    printf("Sending control transfer to wake up device...\n");
    unsigned char buffer[8] = {0};
    int ret = libusb_control_transfer(
        m_DeviceHandle,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_STANDARD | LIBUSB_RECIPIENT_DEVICE,
        LIBUSB_REQUEST_GET_STATUS,
        0, // value
        0, // index
        buffer,
        sizeof(buffer),
        1000 // timeout
    );
    if (ret >= 0) {
        printf("Control transfer successful\n");
    } else {
        printf("Control transfer failed, but continuing anyway\n");
    }
    
    printf("\n====== LIBUSB INITIALIZATION COMPLETE ======\n");
    return true;
}

bool DirectUSB::TryAlternativeMethods() {
    printf("\n====== TRYING ALTERNATIVE USB ACCESS METHODS ======\n");
    
    // First, try bulk endpoints instead of interrupt
    if (TryAlternativeAccess()) {
        printf("Alternative access method succeeded\n");
        return true;
    }
    
    // Next, try raw device access on Linux
    #ifdef __linux__
    if (TryRawAccess()) {
        printf("Raw device access succeeded\n");
        return true;
    }
    #endif
    
    printf("All standard and alternative methods failed\n");
    return false;
}

#ifdef __linux__
bool DirectUSB::TryRawAccess() {
    printf("\n==== ATTEMPTING RAW DEVICE ACCESS ====\n");
    
    // Look for potential device files
    const char* potential_paths[] = {
        "/dev/hidraw0", "/dev/hidraw1", "/dev/hidraw2", "/dev/hidraw3",
        "/dev/usb/hiddev0", "/dev/usb/hiddev1",
        "/dev/ttyUSB0", "/dev/ttyUSB1",
        nullptr
    };
    
    for (int i = 0; potential_paths[i] != nullptr; i++) {
        printf("Trying to open %s...\n", potential_paths[i]);
        
        int fd = open(potential_paths[i], O_RDWR);
        if (fd >= 0) {
            printf("Successfully opened %s as raw device\n", potential_paths[i]);
            m_RawDevice = fd;
            m_RawDevicePath = potential_paths[i];
            return true;
        }
    }
    
    printf("Failed to open any raw device\n");
    return false;
}
#endif

bool DirectUSB::IsConnected() const {
    return m_DeviceHandle != nullptr || m_RawDevice >= 0;
}

bool DirectUSB::SendReport(const uint8_t* data, size_t length) {
    if (m_RawDevice >= 0) {
        #ifdef __linux__
        // Use raw device file
        LogPacket("RAW SENDING", data, length);
        
        ssize_t written = write(m_RawDevice, data, length);
        if (written < 0) {
            printf("Failed to write to raw device: %s\n", strerror(errno));
            return false;
        }
        
        printf("Successfully sent %zd bytes to raw device\n", written);
        return true;
        #else
        return false;
        #endif
    }
    
    if (!m_DeviceHandle) return false;
    
    // Log the outgoing data
    LogPacket("SENDING", data, length);
    
    // Try interrupt transfer first
    int transferred = 0;
    int result = libusb_interrupt_transfer(
        m_DeviceHandle,
        m_OutputEndpoint,
        const_cast<unsigned char*>(data),
        static_cast<int>(length),
        &transferred,
        m_DefaultTimeout
    );
    
    if (result != 0) {
        printf("Interrupt transfer failed: %s, trying bulk transfer...\n", 
               libusb_error_name(result));
        
        // Fallback to bulk transfer
        result = libusb_bulk_transfer(
            m_DeviceHandle,
            m_OutputEndpoint,
            const_cast<unsigned char*>(data),
            static_cast<int>(length),
            &transferred,
            m_DefaultTimeout
        );
        
        if (result != 0) {
            printf("Bulk transfer also failed: %s\n", libusb_error_name(result));
            return false;
        }
    }
    
    printf("Successfully sent %d bytes\n", transferred);
    return true;
}

bool DirectUSB::GetNextReport(std::vector<uint8_t>& report, int timeout_ms) {
    if (m_RawDevice >= 0) {
        #ifdef __linux__
        // Use raw device file
        report.resize(64);  // Default report size
        
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(m_RawDevice, &readfds);
        
        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        
        int ready = select(m_RawDevice + 1, &readfds, NULL, NULL, &tv);
        if (ready <= 0) {
            if (ready == 0) {
                // Timeout
                return false;
            } else {
                printf("Select error: %s\n", strerror(errno));
                return false;
            }
        }
        
        ssize_t bytes_read = read(m_RawDevice, report.data(), report.size());
        if (bytes_read < 0) {
            printf("Failed to read from raw device: %s\n", strerror(errno));
            return false;
        }
        
        // Resize to actual data received
        report.resize(bytes_read);
        
        // Log the received data
        LogPacket("RAW RECEIVED", report.data(), report.size());
        
        return bytes_read > 0;
        #else
        return false;
        #endif
    }
    
    // Check for data in the queue first (from async thread)
    {
        std::unique_lock<std::mutex> lock(m_QueueMutex);
        if (!m_DataQueue.empty()) {
            report = std::move(m_DataQueue.front());
            m_DataQueue.pop();
            return true;
        }
        
        // If async thread is running, wait for data
        if (m_Running) {
            auto result = m_DataAvailable.wait_for(lock, 
                std::chrono::milliseconds(timeout_ms));
            if (result == std::cv_status::timeout) {
                return false;
            }
            
            if (!m_DataQueue.empty()) {
                report = std::move(m_DataQueue.front());
                m_DataQueue.pop();
                return true;
            }
            return false;
        }
    }
    
    // Direct synchronous read if no async thread
    if (!m_DeviceHandle) return false;
    
    // Create a buffer for the report
    report.resize(64);  // Common HID report size
    
    // Try interrupt transfer first
    int transferred = 0;
    int result = libusb_interrupt_transfer(
        m_DeviceHandle,
        m_InputEndpoint,
        report.data(),
        static_cast<int>(report.size()),
        &transferred,
        timeout_ms > 0 ? timeout_ms : m_DefaultTimeout
    );
    
    if (result != 0) {
        if (result == LIBUSB_ERROR_TIMEOUT) {
            // Timeout is not an error
            return false;
        }
        
        printf("Interrupt read failed: %s, trying bulk read...\n", 
               libusb_error_name(result));
        
        // Fallback to bulk transfer
        result = libusb_bulk_transfer(
            m_DeviceHandle,
            m_InputEndpoint,
            report.data(),
            static_cast<int>(report.size()),
            &transferred,
            timeout_ms > 0 ? timeout_ms : m_DefaultTimeout
        );
        
        if (result != 0) {
            if (result == LIBUSB_ERROR_TIMEOUT) {
                return false;
            }
            
            printf("Bulk read also failed: %s\n", libusb_error_name(result));
            return false;
        }
    }
    
    // Resize to actual data received
    report.resize(transferred);
    
    // Log the received data
    LogPacket("RECEIVED", report.data(), report.size());
    
    return true;
}

bool DirectUSB::ResetDevice() {
    if (!IsConnected() || !m_DeviceHandle) {
        printf("Cannot reset device: No active device connection\n");
        return false;
    }
    
    printf("Attempting USB device reset using libusb...\n");
    
    // Try multiple reset strategies:
    bool success = false;
    
    // Strategy 1: Full device reset (most thorough but can disconnect device)
    printf("Strategy 1: Full device reset\n");
    int result = libusb_reset_device(m_DeviceHandle);
    if (result == 0) {
        printf("Full device reset successful\n");
        
        // Need to reclaim interface after reset
        if (libusb_claim_interface(m_DeviceHandle, m_Interface) == 0) {
            printf("Interface reclaimed successfully\n");
            success = true;
        } else {
            printf("Failed to reclaim interface after reset\n");
        }
    } else {
        printf("Full device reset failed: %s\n", libusb_error_name(result));
        
        // Strategy 2: Try to clear halt on endpoints
        printf("Strategy 2: Clearing halt on endpoints\n");
        bool in_cleared = false;
        bool out_cleared = false;
        
        result = libusb_clear_halt(m_DeviceHandle, m_InputEndpoint);
        if (result == 0) {
            printf("Successfully cleared halt on IN endpoint 0x%02X\n", m_InputEndpoint);
            in_cleared = true;
        } else {
            printf("Failed to clear halt on IN endpoint: %s\n", libusb_error_name(result));
        }
        
        result = libusb_clear_halt(m_DeviceHandle, m_OutputEndpoint);
        if (result == 0) {
            printf("Successfully cleared halt on OUT endpoint 0x%02X\n", m_OutputEndpoint);
            out_cleared = true;
        } else {
            printf("Failed to clear halt on OUT endpoint: %s\n", libusb_error_name(result));
        }
        
        if (in_cleared || out_cleared) {
            success = true;
        } else {
            // Strategy 3: Release and reclaim interface
            printf("Strategy 3: Release and reclaim interface\n");
            libusb_release_interface(m_DeviceHandle, m_Interface);
            
            // Small delay to allow the system to process the release
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            
            result = libusb_claim_interface(m_DeviceHandle, m_Interface);
            if (result == 0) {
                printf("Successfully released and reclaimed interface\n");
                success = true;
            } else {
                printf("Failed to reclaim interface: %s\n", libusb_error_name(result));
            }
        }
    }
    
    return success;
}

bool DirectUSB::FindEndpoints() {
    if (!m_DeviceHandle) return false;
    
    libusb_device* dev = libusb_get_device(m_DeviceHandle);
    if (!dev) return false;
    
    struct libusb_config_descriptor* config = nullptr;
    int r = libusb_get_active_config_descriptor(dev, &config);
    if (r != 0) return false;
    
    bool found_endpoints = false;
    
    // Loop through all interfaces
    for (uint8_t i = 0; i < config->bNumInterfaces; i++) {
        const struct libusb_interface* interface = &config->interface[i];
        
        // Loop through all alternate settings
        for (int j = 0; j < interface->num_altsetting; j++) {
            const struct libusb_interface_descriptor* iface = &interface->altsetting[j];
            
            // Check if this is our claimed interface
            if (iface->bInterfaceNumber == m_Interface) {
                printf("Found our interface %d\n", m_Interface);
                
                // Loop through all endpoints
                for (uint8_t k = 0; k < iface->bNumEndpoints; k++) {
                    const struct libusb_endpoint_descriptor* ep = &iface->endpoint[k];
                    
                    // Check endpoint direction
                    if (ep->bEndpointAddress & LIBUSB_ENDPOINT_IN) {
                        printf("Found IN endpoint: 0x%02X\n", ep->bEndpointAddress);
                        m_InputEndpoint = ep->bEndpointAddress;
                    } else {
                        printf("Found OUT endpoint: 0x%02X\n", ep->bEndpointAddress);
                        m_OutputEndpoint = ep->bEndpointAddress;
                    }
                    
                    found_endpoints = true;
                }
            }
        }
    }
    
    libusb_free_config_descriptor(config);
    return found_endpoints;
}

void DirectUSB::LogPacket(const char* prefix, const uint8_t* data, size_t length) {
    if (!m_VerboseLogging) return;
    
    printf("%s (%zu bytes): ", prefix, length);
    
    // Limit the output to a reasonable number of bytes
    const size_t display_limit = 32;
    size_t display_bytes = length > display_limit ? display_limit : length;
    
    for (size_t i = 0; i < display_bytes; i++) {
        printf("%02X ", data[i]);
        
        // Add a newline every 16 bytes for readability
        if ((i + 1) % 16 == 0 && i < display_bytes - 1) {
            printf("\n                  ");
        }
    }
    
    if (length > display_limit) {
        printf("... (%zu more bytes)", length - display_limit);
    }
    
    printf("\n");
}

void DirectUSB::StartAsyncRead() {
    if (m_Running) return;
    
    m_Running = true;
    m_ReadThread = std::thread(&DirectUSB::ReadThreadFunc, this);
    printf("Async read thread started\n");
}

void DirectUSB::StopAsyncRead() {
    if (!m_Running) return;
    
    printf("Stopping async read thread...\n");
    m_Running = false;
    
    if (m_ReadThread.joinable()) {
        m_ReadThread.join();
    }
    
    printf("Async read thread stopped\n");
}

void DirectUSB::ReadThreadFunc() {
    std::vector<uint8_t> report;
    
    printf("Read thread started\n");
    
    while (m_Running) {
        // Get the next report with a short timeout
        if (GetNextReport(report, 100)) {
            // Add to the queue
            std::unique_lock<std::mutex> lock(m_QueueMutex);
            
            // Limit queue size to prevent memory issues
            if (m_DataQueue.size() < 100) {
                m_DataQueue.push(report);
                m_DataAvailable.notify_one();
            }
        }
        
        // Don't hog the CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    printf("Read thread exiting\n");
}

bool DirectUSB::TryAlternativeAccess() {
    printf("\n==== ATTEMPTING ALTERNATIVE ACCESS METHODS ====\n");
    
    // If we don't have a device handle, try to get one
    if (!m_DeviceHandle) {
        printf("No device handle available for alternative access\n");
        return false;
    }
    
    // Try different interface numbers
    for (int iface = 0; iface < 4; iface++) {
        if (iface == m_Interface) continue;  // Skip our current interface
        
        printf("Trying interface %d...\n", iface);
        
        // If kernel driver is active, detach it
        if (libusb_kernel_driver_active(m_DeviceHandle, iface)) {
            printf("Detaching kernel driver from interface %d\n", iface);
            libusb_detach_kernel_driver(m_DeviceHandle, iface);
        }
        
        // Try to claim the interface
        if (libusb_claim_interface(m_DeviceHandle, iface) == 0) {
            // Release our previous interface
            libusb_release_interface(m_DeviceHandle, m_Interface);
            m_Interface = iface;
            printf("Successfully claimed alternative interface %d\n", iface);
            
            // Try various endpoint addresses
            const int ep_addresses[][2] = {
                {0x81, 0x01},  // Standard HID endpoints
                {0x82, 0x02},  // Alternative endpoints
                {0x83, 0x03},  // More alternatives
                {0x84, 0x04}   // Even more alternatives
            };
            
            for (int i = 0; i < 4; i++) {
                m_InputEndpoint = ep_addresses[i][0];
                m_OutputEndpoint = ep_addresses[i][1];
                
                printf("Trying IN=0x%02X, OUT=0x%02X...\n", m_InputEndpoint, m_OutputEndpoint);
                
                // Try a test transfer to see if it works
                unsigned char test_data[8] = {0};
                int transferred = 0;
                int result = libusb_interrupt_transfer(
                    m_DeviceHandle,
                    m_InputEndpoint,
                    test_data,
                    sizeof(test_data),
                    &transferred,
                    100  // Short timeout for test
                );
                
                // LIBUSB_ERROR_TIMEOUT is actually a "success" for this test
                // It means the endpoint exists but no data is available yet
                if (result == 0 || result == LIBUSB_ERROR_TIMEOUT) {
                    printf("Endpoint test successful!\n");
                    return true;
                }
                
                printf("Endpoint test failed: %s\n", libusb_error_name(result));
            }
            
            // If we couldn't find working endpoints, release this interface
            libusb_release_interface(m_DeviceHandle, m_Interface);
        }
    }
    
    printf("All alternative access methods failed\n");
    return false;
}

void DirectUSB::DumpDeviceInfo() {
    if (!m_DeviceHandle) {
        printf("No device handle available for info dump\n");
        return;
    }
    
    printf("\n==== USB DEVICE INFORMATION ====\n");
    
    libusb_device* dev = libusb_get_device(m_DeviceHandle);
    if (!dev) {
        printf("Could not get device from handle\n");
        return;
    }
    
    // Get device descriptor
    struct libusb_device_descriptor desc;
    if (libusb_get_device_descriptor(dev, &desc) != 0) {
        printf("Failed to get device descriptor\n");
        return;
    }
    
    printf("Device Information:\n");
    printf("  Bus: %03d Device: %03d\n", 
           libusb_get_bus_number(dev), 
           libusb_get_device_address(dev));
    printf("  VID: 0x%04X, PID: 0x%04X\n", desc.idVendor, desc.idProduct);
    printf("  USB Version: %d.%d\n", desc.bcdUSB >> 8, desc.bcdUSB & 0xFF);
    printf("  Device Class: 0x%02X, SubClass: 0x%02X, Protocol: 0x%02X\n", 
           desc.bDeviceClass, desc.bDeviceSubClass, desc.bDeviceProtocol);
    printf("  Max Packet Size: %d\n", desc.bMaxPacketSize0);
    
    // Get string descriptors
    unsigned char string_data[256];
    
    if (desc.iManufacturer > 0) {
        if (libusb_get_string_descriptor_ascii(m_DeviceHandle, desc.iManufacturer, 
                                             string_data, sizeof(string_data)) > 0) {
            printf("  Manufacturer: %s\n", string_data);
        }
    }
    
    if (desc.iProduct > 0) {
        if (libusb_get_string_descriptor_ascii(m_DeviceHandle, desc.iProduct, 
                                             string_data, sizeof(string_data)) > 0) {
            printf("  Product: %s\n", string_data);
        }
    }
    
    if (desc.iSerialNumber > 0) {
        if (libusb_get_string_descriptor_ascii(m_DeviceHandle, desc.iSerialNumber, 
                                             string_data, sizeof(string_data)) > 0) {
            printf("  Serial: %s\n", string_data);
        }
    }
    
    printf("\n");
}

void DirectUSB::PrintEndpointInfo() {
    if (!m_DeviceHandle) {
        printf("No device handle available for endpoint info\n");
        return;
    }
    
    printf("\n==== USB ENDPOINT INFORMATION ====\n");
    
    libusb_device* dev = libusb_get_device(m_DeviceHandle);
    if (!dev) {
        printf("Could not get device from handle\n");
        return;
    }
    
    struct libusb_config_descriptor* config = nullptr;
    int r = libusb_get_active_config_descriptor(dev, &config);
    if (r != 0) {
        printf("Failed to get config descriptor\n");
        return;
    }
    
    printf("Active Configuration:\n");
    printf("  bConfigurationValue: %d\n", config->bConfigurationValue);
    printf("  bNumInterfaces: %d\n", config->bNumInterfaces);
    printf("  bmAttributes: 0x%02X\n", config->bmAttributes);
    
    // Fix for Linux: Use MaxPower instead of bMaxPower
    // The name differs between Windows and Linux versions of libusb
#ifdef __linux__
    printf("  MaxPower: %dmA\n", config->MaxPower * 2);
#else
    printf("  MaxPower: %dmA\n", config->bMaxPower * 2);
#endif
    
    // Loop through all interfaces
    for (uint8_t i = 0; i < config->bNumInterfaces; i++) {
        const struct libusb_interface* interface = &config->interface[i];
        
        // Loop through all alternate settings
        for (int j = 0; j < interface->num_altsetting; j++) {
            const struct libusb_interface_descriptor* iface = &interface->altsetting[j];
            
            printf("\n  Interface %d, Alt Setting %d:\n", 
                   iface->bInterfaceNumber, iface->bAlternateSetting);
            printf("    bInterfaceClass: 0x%02X\n", iface->bInterfaceClass);
            printf("    bInterfaceSubClass: 0x%02X\n", iface->bInterfaceSubClass);
            printf("    bInterfaceProtocol: 0x%02X\n", iface->bInterfaceProtocol);
            printf("    bNumEndpoints: %d\n", iface->bNumEndpoints);
            
            // Check if this is our claimed interface
            if (iface->bInterfaceNumber == m_Interface) {
                printf("    *** THIS IS OUR CLAIMED INTERFACE ***\n");
            }
            
            // Loop through all endpoints
            for (uint8_t k = 0; k < iface->bNumEndpoints; k++) {
                const struct libusb_endpoint_descriptor* ep = &iface->endpoint[k];
                
                printf("      Endpoint 0x%02X:\n", ep->bEndpointAddress);
                printf("        Type: %s\n", 
                       (ep->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_INTERRUPT ? "Interrupt" :
                       (ep->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_BULK ? "Bulk" :
                       (ep->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_CONTROL ? "Control" :
                       (ep->bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_ISOCHRONOUS ? "Isochronous" : "Unknown");
                printf("        Direction: %s\n", (ep->bEndpointAddress & LIBUSB_ENDPOINT_IN) ? "IN" : "OUT");
                printf("        Max Packet Size: %d\n", ep->wMaxPacketSize);
                printf("        Interval: %d\n", ep->bInterval);
                
                // Check if this is one of our endpoints
                if (ep->bEndpointAddress == m_InputEndpoint) {
                    printf("        *** THIS IS OUR IN ENDPOINT ***\n");
                } else if (ep->bEndpointAddress == m_OutputEndpoint) {
                    printf("        *** THIS IS OUR OUT ENDPOINT ***\n");
                }
            }
        }
    }
    
    libusb_free_config_descriptor(config);
}

void DirectUSB::DumpRawDescriptors() {
    if (!m_DeviceHandle) {
        printf("No device handle available for raw descriptor dump\n");
        return;
    }
    
    printf("\n==== RAW USB DESCRIPTORS ====\n");
    
    libusb_device* dev = libusb_get_device(m_DeviceHandle);
    if (!dev) {
        printf("Could not get device from handle\n");
        return;
    }
    
    // Get and print device descriptor
    struct libusb_device_descriptor desc;
    if (libusb_get_device_descriptor(dev, &desc) != 0) {
        printf("Failed to get device descriptor\n");
        return;
    }
    
    printf("Device Descriptor Raw Data:\n");
    unsigned char* ptr = (unsigned char*)&desc;
    for (size_t i = 0; i < sizeof(desc); i++) {
        printf("%02X ", ptr[i]);
        if ((i + 1) % 16 == 0) printf("\n");
    }
    printf("\n\n");
    
    // Get and print configuration descriptor
    struct libusb_config_descriptor* config = nullptr;
    int r = libusb_get_active_config_descriptor(dev, &config);
    if (r != 0) {
        printf("Failed to get config descriptor\n");
        return;
    }
    
    printf("Configuration Descriptor Raw Data:\n");
    ptr = (unsigned char*)config;
    for (size_t i = 0; i < sizeof(struct libusb_config_descriptor); i++) {
        printf("%02X ", ptr[i]);
        if ((i + 1) % 16 == 0) printf("\n");
    }
    printf("\n");
    
    libusb_free_config_descriptor(config);
}