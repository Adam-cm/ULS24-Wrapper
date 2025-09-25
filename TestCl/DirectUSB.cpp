// Copyright 2023, All rights reserved

#include "DirectUSB.h"
#include <cstdio>
#include <cstring>
#include <chrono>
#include <iomanip>
#include <sstream>

// Global instance
DirectUSB g_DirectUSB;

DirectUSB::DirectUSB() : m_Running(false) {
}

DirectUSB::~DirectUSB() {
    StopAsyncRead();
    
    if (m_DeviceHandle) {
        // Release interface and close device
        libusb_release_interface(m_DeviceHandle, m_Interface);
        libusb_close(m_DeviceHandle);
        m_DeviceHandle = nullptr;
    }
    
    if (m_Context) {
        libusb_exit(m_Context);
        m_Context = nullptr;
    }
}

bool DirectUSB::Initialize() {
    // Initialize libusb
    int result = libusb_init(&m_Context);
    if (result != 0) {
        printf("Failed to initialize libusb: %s\n", libusb_error_name(result));
        return false;
    }
    
    // Set debug level (3 = verbose)
    libusb_set_option(m_Context, LIBUSB_OPTION_LOG_LEVEL, 3);
    
    // Find the device
    m_DeviceHandle = libusb_open_device_with_vid_pid(m_Context, VENDOR_ID, PRODUCT_ID);
    if (!m_DeviceHandle) {
        printf("Failed to find or open the device (VID=0x%04X, PID=0x%04X)\n", 
               VENDOR_ID, PRODUCT_ID);
        return false;
    }
    
    printf("Successfully opened device (VID=0x%04X, PID=0x%04X)\n", 
           VENDOR_ID, PRODUCT_ID);
    
    // Print device info
    DumpDeviceInfo();
    
    // Automatically detach kernel driver if needed
    libusb_set_auto_detach_kernel_driver(m_DeviceHandle, 1);
    
    // Claim the interface
    result = libusb_claim_interface(m_DeviceHandle, m_Interface);
    if (result != 0) {
        printf("Failed to claim interface: %s\n", libusb_error_name(result));
        libusb_close(m_DeviceHandle);
        m_DeviceHandle = nullptr;
        return false;
    }
    
    // Find endpoints
    if (!FindEndpoints()) {
        printf("Failed to find suitable endpoints\n");
        libusb_release_interface(m_DeviceHandle, m_Interface);
        libusb_close(m_DeviceHandle);
        m_DeviceHandle = nullptr;
        return false;
    }
    
    printf("Successfully claimed interface %d\n", m_Interface);
    printf("Using IN endpoint: 0x%02X, OUT endpoint: 0x%02X\n", 
           m_InputEndpoint, m_OutputEndpoint);
    
    return true;
}

bool DirectUSB::IsConnected() const {
    return m_DeviceHandle != nullptr;
}

bool DirectUSB::SendReport(const uint8_t* data, size_t length) {
    if (!m_DeviceHandle) return false;
    
    // Log the outgoing data
    LogPacket("SENDING", data, length);
    
    int transferred = 0;
    int result = libusb_interrupt_transfer(
        m_DeviceHandle,
        m_OutputEndpoint,
        const_cast<unsigned char*>(data), // libusb expects non-const pointer
        static_cast<int>(length),
        &transferred,
        1000 // 1 second timeout
    );
    
    if (result != 0) {
        printf("Failed to send report: %s\n", libusb_error_name(result));
        return false;
    }
    
    printf("Successfully sent %d bytes\n", transferred);
    return true;
}

bool DirectUSB::GetNextReport(std::vector<uint8_t>& report, int timeout_ms) {
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
    
    int transferred = 0;
    int result = libusb_interrupt_transfer(
        m_DeviceHandle,
        m_InputEndpoint,
        report.data(),
        static_cast<int>(report.size()),
        &transferred,
        timeout_ms
    );
    
    if (result != 0) {
        if (result == LIBUSB_ERROR_TIMEOUT) {
            // Timeout is not an error
            return false;
        }
        
        printf("Failed to read report: %s\n", libusb_error_name(result));
        return false;
    }
    
    // Resize to actual data received
    report.resize(transferred);
    
    // Log the received data
    LogPacket("RECEIVED", report.data(), report.size());
    
    return true;
}

void DirectUSB::StartAsyncRead() {
    if (m_Running) return;
    
    m_Running = true;
    m_ReadThread = std::thread(&DirectUSB::ReadThreadFunc, this);
    printf("Async read thread started\n");
}

void DirectUSB::StopAsyncRead() {
    if (!m_Running) return;
    
    m_Running = false;
    
    if (m_ReadThread.joinable()) {
        m_ReadThread.join();
    }
    
    printf("Async read thread stopped\n");
}

void DirectUSB::ReadThreadFunc() {
    while (m_Running) {
        std::vector<uint8_t> report;
        report.resize(64);  // Common HID report size
        
        int transferred = 0;
        int result = libusb_interrupt_transfer(
            m_DeviceHandle,
            m_InputEndpoint,
            report.data(),
            static_cast<int>(report.size()),
            &transferred,
            100  // Short timeout for responsive thread termination
        );
        
        if (result == 0 && transferred > 0) {
            // Resize to actual data received
            report.resize(transferred);
            
            // Log the received data
            LogPacket("ASYNC RECEIVED", report.data(), report.size());
            
            // Add to queue
            {
                std::lock_guard<std::mutex> lock(m_QueueMutex);
                m_DataQueue.push(std::move(report));
            }
            
            // Notify waiting threads
            m_DataAvailable.notify_one();
        }
        else if (result != LIBUSB_ERROR_TIMEOUT) {
            // Print non-timeout errors
            printf("Async read error: %s\n", libusb_error_name(result));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

bool DirectUSB::FindEndpoints() {
    libusb_device* device = libusb_get_device(m_DeviceHandle);
    if (!device) return false;
    
    libusb_config_descriptor* config = nullptr;
    int result = libusb_get_active_config_descriptor(device, &config);
    if (result != 0) {
        printf("Failed to get config descriptor: %s\n", libusb_error_name(result));
        return false;
    }
    
    // Print all available endpoints for diagnostic purposes
    PrintEndpointInfo();
    
    // Find the first interface with IN and OUT endpoints
    bool found = false;
    for (int i = 0; i < config->bNumInterfaces && !found; i++) {
        const libusb_interface& interface = config->interface[i];
        
        for (int j = 0; j < interface.num_altsetting && !found; j++) {
            const libusb_interface_descriptor& iface = interface.altsetting[j];
            
            // Look for HID interface or use the first one
            if (iface.bInterfaceClass == LIBUSB_CLASS_HID || i == 0) {
                int in_ep = -1;
                int out_ep = -1;
                
                // Find IN and OUT endpoints
                for (int k = 0; k < iface.bNumEndpoints; k++) {
                    const libusb_endpoint_descriptor& endpoint = iface.endpoint[k];
                    
                    if ((endpoint.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) == LIBUSB_TRANSFER_TYPE_INTERRUPT) {
                        if (endpoint.bEndpointAddress & LIBUSB_ENDPOINT_IN) {
                            in_ep = endpoint.bEndpointAddress;
                        } else {
                            out_ep = endpoint.bEndpointAddress;
                        }
                    }
                }
                
                if (in_ep != -1 && out_ep != -1) {
                    m_Interface = iface.bInterfaceNumber;
                    m_InputEndpoint = in_ep;
                    m_OutputEndpoint = out_ep;
                    found = true;
                }
            }
        }
    }
    
    libusb_free_config_descriptor(config);
    return found;
}

void DirectUSB::LogPacket(const char* prefix, const uint8_t* data, size_t length) {
    if (!m_VerboseLogging) return;
    
    printf("\n%s %zu bytes:\n", prefix, length);
    
    // Print in hex format, 16 bytes per line
    for (size_t i = 0; i < length; i += 16) {
        printf("%04zX: ", i);
        
        // Print hex values
        for (size_t j = 0; j < 16; j++) {
            if (i + j < length) {
                printf("%02X ", data[i + j]);
            } else {
                printf("   ");
            }
            
            // Extra space after 8 bytes for readability
            if (j == 7) printf(" ");
        }
        
        // Print ASCII representation
        printf(" | ");
        for (size_t j = 0; j < 16; j++) {
            if (i + j < length) {
                uint8_t c = data[i + j];
                if (c >= 32 && c <= 126) {
                    printf("%c", c);
                } else {
                    printf(".");
                }
            } else {
                printf(" ");
            }
        }
        printf(" |\n");
    }
    
    // Look for potential protocol patterns
    if (length > 0) {
        printf("Analysis: ");
        if (data[0] == 0xAA) {
            printf("Valid preamble detected\n");
            
            if (length > 2) {
                printf("  Command type: 0x%02X\n", data[2]);
                
                // Add more protocol-specific analysis here
                if (data[2] == 0x01) {
                    printf("  Looks like a control command\n");
                } else if (data[2] == 0x02) {
                    printf("  Looks like a data command\n");
                    if (length > 5) {
                        printf("  Row index: %d\n", data[5]);
                    }
                }
            }
        }
    }
    
    printf("\n");
}

void DirectUSB::DumpDeviceInfo() {
    if (!m_DeviceHandle) return;
    
    libusb_device* device = libusb_get_device(m_DeviceHandle);
    if (!device) return;
    
    // Get device descriptor
    libusb_device_descriptor desc;
    if (libusb_get_device_descriptor(device, &desc) != 0) {
        printf("Failed to get device descriptor\n");
        return;
    }
    
    printf("\n====== USB DEVICE INFORMATION ======\n");
    printf("Bus: %03d Device: %03d\n", 
           libusb_get_bus_number(device), 
           libusb_get_device_address(device));
    printf("VID: 0x%04X, PID: 0x%04X\n", desc.idVendor, desc.idProduct);
    printf("USB Version: %d.%d\n", 
           desc.bcdUSB >> 8, desc.bcdUSB & 0xFF);
    printf("Device Class: 0x%02X, SubClass: 0x%02X, Protocol: 0x%02X\n", 
           desc.bDeviceClass, desc.bDeviceSubClass, desc.bDeviceProtocol);
    printf("Manufacturer ID: %d, Product ID: %d, Serial Number ID: %d\n", 
           desc.iManufacturer, desc.iProduct, desc.iSerialNumber);
    
    // Try to get string descriptors
    unsigned char string_data[256];
    
    if (desc.iManufacturer > 0) {
        int result = libusb_get_string_descriptor_ascii(
            m_DeviceHandle, desc.iManufacturer, string_data, sizeof(string_data));
        if (result > 0) {
            printf("Manufacturer: %s\n", string_data);
        }
    }
    
    if (desc.iProduct > 0) {
        int result = libusb_get_string_descriptor_ascii(
            m_DeviceHandle, desc.iProduct, string_data, sizeof(string_data));
        if (result > 0) {
            printf("Product: %s\n", string_data);
        }
    }
    
    if (desc.iSerialNumber > 0) {
        int result = libusb_get_string_descriptor_ascii(
            m_DeviceHandle, desc.iSerialNumber, string_data, sizeof(string_data));
        if (result > 0) {
            printf("Serial Number: %s\n", string_data);
        }
    }
    
    printf("==============================\n\n");
}

void DirectUSB::PrintEndpointInfo() {
    if (!m_DeviceHandle) return;
    
    libusb_device* device = libusb_get_device(m_DeviceHandle);
    if (!device) return;
    
    libusb_config_descriptor* config = nullptr;
    int result = libusb_get_active_config_descriptor(device, &config);
    if (result != 0) {
        printf("Failed to get config descriptor\n");
        return;
    }
    
    printf("\n====== USB ENDPOINT INFORMATION ======\n");
    
    for (int i = 0; i < config->bNumInterfaces; i++) {
        const libusb_interface& interface = config->interface[i];
        
        for (int j = 0; j < interface.num_altsetting; j++) {
            const libusb_interface_descriptor& iface = interface.altsetting[j];
            
            printf("Interface %d, Alt Setting %d: Class=%02X, SubClass=%02X, Protocol=%02X\n",
                   iface.bInterfaceNumber, j, 
                   iface.bInterfaceClass,
                   iface.bInterfaceSubClass,
                   iface.bInterfaceProtocol);
            
            for (int k = 0; k < iface.bNumEndpoints; k++) {
                const libusb_endpoint_descriptor& endpoint = iface.endpoint[k];
                
                printf("  Endpoint 0x%02X: ", endpoint.bEndpointAddress);
                
                // Direction
                if (endpoint.bEndpointAddress & LIBUSB_ENDPOINT_IN) {
                    printf("IN, ");
                } else {
                    printf("OUT, ");
                }
                
                // Type
                switch (endpoint.bmAttributes & LIBUSB_TRANSFER_TYPE_MASK) {
                    case LIBUSB_TRANSFER_TYPE_CONTROL:
                        printf("Control, ");
                        break;
                    case LIBUSB_TRANSFER_TYPE_ISOCHRONOUS:
                        printf("Isochronous, ");
                        break;
                    case LIBUSB_TRANSFER_TYPE_BULK:
                        printf("Bulk, ");
                        break;
                    case LIBUSB_TRANSFER_TYPE_INTERRUPT:
                        printf("Interrupt, ");
                        break;
                    default:
                        printf("Unknown, ");
                }
                
                printf("Max Packet Size: %d\n", endpoint.wMaxPacketSize);
            }
            
            printf("\n");
        }
    }
    
    printf("============================\n\n");
    
    libusb_free_config_descriptor(config);
}

void DirectUSB::DumpRawDescriptors() {
    if (!m_DeviceHandle) return;
    
    libusb_device* device = libusb_get_device(m_DeviceHandle);
    if (!device) return;
    
    // Get all descriptors for this device
    printf("\n====== RAW USB DESCRIPTORS ======\n");
    
    // Device descriptor
    libusb_device_descriptor desc;
    if (libusb_get_device_descriptor(device, &desc) == 0) {
        printf("Device Descriptor Raw Data:\n");
        uint8_t* raw = (uint8_t*)&desc;
        for (int i = 0; i < sizeof(desc); i++) {
            printf("%02X ", raw[i]);
            if ((i + 1) % 16 == 0) printf("\n");
        }
        printf("\n\n");
    }
    
    // Configuration descriptor
    libusb_config_descriptor* config = nullptr;
    int result = libusb_get_active_config_descriptor(device, &config);
    if (result == 0) {
        printf("Configuration Descriptor Raw Data:\n");
        uint8_t* raw = (uint8_t*)config;
        for (int i = 0; i < config->wTotalLength && i < 256; i++) {
            printf("%02X ", raw[i]);
            if ((i + 1) % 16 == 0) printf("\n");
        }
        printf("\n");
        
        libusb_free_config_descriptor(config);
    }
    
    printf("============================\n\n");
}