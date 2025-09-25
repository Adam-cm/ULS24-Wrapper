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

// Rest of your implementation remains unchanged...