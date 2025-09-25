#include "DirectUSB.h"
#include <cstdio>
#include <chrono>

// Global instance
DirectUSB g_DirectUSB;

DirectUSB::DirectUSB() : ctx(nullptr), handle(nullptr) {
}

DirectUSB::~DirectUSB() {
    Close();
}

bool DirectUSB::Initialize() {
    // Initialize libusb
    int result = libusb_init(&ctx);
    if (result < 0) {
        fprintf(stderr, "Error: Failed to initialize libusb: %s\n", libusb_error_name(result));
        return false;
    }
    
    // Set debug level
    #ifdef _DEBUG
    libusb_set_option(ctx, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
    #endif
    
    // Open the device
    handle = libusb_open_device_with_vid_pid(ctx, VENDOR_ID, PRODUCT_ID);
    if (!handle) {
        fprintf(stderr, "Error: Could not find or open device 0x%04x:0x%04x\n", VENDOR_ID, PRODUCT_ID);
        libusb_exit(ctx);
        ctx = nullptr;
        return false;
    }
    
    // Get current configuration
    result = libusb_get_configuration(handle, &active_config);
    if (result < 0) {
        fprintf(stderr, "Error: Could not get device configuration: %s\n", libusb_error_name(result));
        active_config = 1; // Assume config 1
    }
    
    // Check if kernel driver is active and detach it
    if (libusb_kernel_driver_active(handle, USB_INTERFACE_NUMBER)) {
        printf("Kernel driver active, attempting to detach...\n");
        result = libusb_detach_kernel_driver(handle, USB_INTERFACE_NUMBER);
        if (result < 0) {
            fprintf(stderr, "Error: Could not detach kernel driver: %s\n", libusb_error_name(result));
            libusb_close(handle);
            handle = nullptr;
            libusb_exit(ctx);
            ctx = nullptr;
            return false;
        }
        kernel_detached = true;
    }
    
    // Claim the interface
    result = libusb_claim_interface(handle, USB_INTERFACE_NUMBER);
    if (result < 0) {
        fprintf(stderr, "Error: Could not claim interface: %s\n", libusb_error_name(result));
        if (kernel_detached) {
            libusb_attach_kernel_driver(handle, USB_INTERFACE_NUMBER);
        }
        libusb_close(handle);
        handle = nullptr;
        libusb_exit(ctx);
        ctx = nullptr;
        return false;
    }
    
    printf("USB device opened successfully with libusb direct access\n");
    printf("Device configuration: %d\n", active_config);
    
    // Get device info
    libusb_device* dev = libusb_get_device(handle);
    struct libusb_device_descriptor desc;
    libusb_get_device_descriptor(dev, &desc);
    
    printf("USB Device: %04x:%04x (bcdUSB: %04x)\n", 
           desc.idVendor, desc.idProduct, desc.bcdUSB);
    printf("Class: %02x, Subclass: %02x, Protocol: %02x\n",
           desc.bDeviceClass, desc.bDeviceSubClass, desc.bDeviceProtocol);
    
    return true;
}

void DirectUSB::Close() {
    // Stop read thread if active
    StopAsyncRead();
    
    if (handle) {
        // Release interface
        libusb_release_interface(handle, USB_INTERFACE_NUMBER);
        
        // Re-attach kernel driver if we detached it
        if (kernel_detached) {
            libusb_attach_kernel_driver(handle, USB_INTERFACE_NUMBER);
            kernel_detached = false;
        }
        
        // Close device
        libusb_close(handle);
        handle = nullptr;
    }
    
    if (ctx) {
        libusb_exit(ctx);
        ctx = nullptr;
    }
}

bool DirectUSB::SendReport(const uint8_t* data, size_t length) {
    if (!handle) return false;
    
    // Create buffer with report ID
    std::vector<uint8_t> buffer(length + 1, 0);
    buffer[0] = 0; // Report ID 0
    std::memcpy(buffer.data() + 1, data, length);
    
    // Send data
    int transferred = 0;
    int result = libusb_interrupt_transfer(
        handle,
        USB_ENDPOINT_OUT,
        buffer.data(),
        buffer.size(),
        &transferred,
        USB_TIMEOUT
    );
    
    if (result < 0) {
        fprintf(stderr, "Error sending USB report: %s\n", libusb_error_name(result));
        return false;
    }
    
    printf("Sent %d bytes via libusb direct transfer\n", transferred);
    return true;
}

bool DirectUSB::ReceiveReport(uint8_t* data, size_t length, int timeout_ms) {
    if (!handle) return false;
    
    // Buffer to receive data (+1 for report ID)
    std::vector<uint8_t> buffer(length + 1, 0);
    
    // Receive data
    int transferred = 0;
    int result = libusb_interrupt_transfer(
        handle,
        USB_ENDPOINT_IN,
        buffer.data(),
        buffer.size(),
        &transferred,
        timeout_ms
    );
    
    if (result < 0) {
        if (result != LIBUSB_ERROR_TIMEOUT) {
            fprintf(stderr, "Error receiving USB report: %s\n", libusb_error_name(result));
        }
        return false;
    }
    
    // Copy data to output buffer (skip report ID)
    if (transferred > 1) {
        std::memcpy(data, buffer.data() + 1, std::min(length, static_cast<size_t>(transferred - 1)));
        printf("Received %d bytes via libusb direct transfer\n", transferred - 1);
        return true;
    }
    
    return false;
}

void DirectUSB::ReadThreadFunc() {
    const size_t BUFFER_SIZE = 65; // 64 + 1 for report ID
    std::vector<uint8_t> buffer(BUFFER_SIZE);
    
    while (running) {
        // Receive data with short timeout
        int transferred = 0;
        int result = libusb_interrupt_transfer(
            handle,
            USB_ENDPOINT_IN,
            buffer.data(),
            buffer.size(),
            &transferred,
            100 // 100ms timeout for responsiveness
        );
        
        if (result == 0 && transferred > 1) {
            // Create report without report ID
            std::vector<uint8_t> report(transferred - 1);
            std::memcpy(report.data(), buffer.data() + 1, transferred - 1);
            
            // Add to queue
            {
                std::lock_guard<std::mutex> lock(buffer_mutex);
                // If queue is too full, remove oldest report
                if (data_queue.size() >= BUFFER_SIZE) {
                    data_queue.pop();
                }
                data_queue.push(std::move(report));
            }
            buffer_cv.notify_one();
        }
        else if (result != LIBUSB_ERROR_TIMEOUT) {
            // Real error occurred
            fprintf(stderr, "USB read error: %s\n", libusb_error_name(result));
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

bool DirectUSB::StartAsyncRead() {
    if (!handle || running) return false;
    
    running = true;
    read_thread = std::thread(&DirectUSB::ReadThreadFunc, this);
    return true;
}

void DirectUSB::StopAsyncRead() {
    if (!running) return;
    
    running = false;
    if (read_thread.joinable()) {
        read_thread.join();
    }
    
    // Clear the queue
    std::lock_guard<std::mutex> lock(buffer_mutex);
    while (!data_queue.empty()) {
        data_queue.pop();
    }
}

bool DirectUSB::GetNextReport(std::vector<uint8_t>& report, int timeout_ms) {
    if (!running) return false;
    
    std::unique_lock<std::mutex> lock(buffer_mutex);
    if (timeout_ms <= 0) {
        // No wait
        if (!data_queue.empty()) {
            report = std::move(data_queue.front());
            data_queue.pop();
            return true;
        }
        return false;
    }
    
    // Wait with timeout
    bool result = buffer_cv.wait_for(lock, std::chrono::milliseconds(timeout_ms),
        [this] { return !data_queue.empty(); });
    
    if (result) {
        report = std::move(data_queue.front());
        data_queue.pop();
        return true;
    }
    
    return false;
}