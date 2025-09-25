//#include "stdafx.h"
#include "InterfaceObj.h"
#include "HidMgr.h"
#include <cstdio>
#include <vector>
#include <thread>
#include <chrono>
#include <hidapi/hidapi.h>

// Linux-specific headers
#ifdef __linux__
#include <unistd.h>
#include <sys/resource.h>
#include <sys/mman.h>
#endif

extern CInterfaceObject theInterfaceObject;
extern uint8_t RxData[RxNum];

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT __attribute__((visibility("default")))
#endif

// C++ linkage function - KEEP THIS OUTSIDE extern "C" block
int reset_usb_endpoints() {
#ifdef __linux__
    if (DeviceHandle) {
        // Implementation details...
        return 1;  // Success
    }
#endif
    return 0;  // Not implemented or failed
}

extern "C" {
    // MAKE SURE ALL THESE FUNCTIONS ARE PRESENT
    EXPORT void selchan(int chan) {
        theInterfaceObject.SelSensor(chan);
    }

    EXPORT void get(int chan) {
        // Try multiple times if needed
        for (int attempts = 0; attempts < 3; attempts++) {
            theInterfaceObject.CaptureFrame12(chan);

            // Check if data looks valid
            bool hasGaps = false;
            for (int i = 1; i < 12; i += 2) {
                if (theInterfaceObject.frame_data[0][i] == 0) {
                    hasGaps = true;
                    break;
                }
            }

            if (!hasGaps) break;

            // Small delay between retries
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    EXPORT void get_frame12(int* outbuf) {
        for (int i = 0; i < 12; ++i) {
            for (int j = 0; j < 12; ++j) {
                outbuf[i * 12 + j] = theInterfaceObject.frame_data[i][j];
            }
        }
    }

    EXPORT void setinttime(float itime) {
        theInterfaceObject.SetIntTime(itime);
    }

    EXPORT void setgain(int gain) {
        theInterfaceObject.SetGainMode(gain);
    }

    EXPORT void reset() {
        FindTheHID();
    }

    // New buffer-related functions
    EXPORT int get_buffer_capacity() {
        return CIRCULAR_BUFFER_SIZE;
    }

    EXPORT int get_buffer_used() {
        return static_cast<int>(GetBufferSize());
    }

    EXPORT int get_buffer_stats(int* stats, int length) {
        if (length >= 3) {
            stats[0] = CIRCULAR_BUFFER_SIZE;
            stats[1] = static_cast<int>(GetBufferSize());
            stats[2] = check_data_flow();
            return 3;
        }
        return 0;
    }

    // C wrapper for reset_usb_endpoints
    EXPORT int c_reset_usb_endpoints() {
        return reset_usb_endpoints();
    }

    EXPORT void optimize_for_pi() {
#ifdef __linux__
        // Increase USB/HID buffer sizes for Raspberry Pi
        if (geteuid() == 0) {
            // 1. Increase kernel USB buffer size
            system("echo 128 > /sys/module/usbcore/parameters/usbfs_memory_mb 2>/dev/null || true");

            // 2. Increase USB transfer timeout
            system("echo 5000 > /sys/module/usbcore/parameters/usb_storage_timeout 2>/dev/null || true");

            // 3. Add udev rule to increase buffer size
            FILE* f = fopen("/etc/udev/rules.d/99-hidapi-buffer.rules", "w");
            if (f) {
                fprintf(f, "# Increase buffer size for HID devices\n");
                fprintf(f, "SUBSYSTEM==\"hidraw\", ATTRS{idVendor}==\"0483\", ATTRS{idProduct}==\"5750\", ATTR{buffer_size}=\"16384\"\n");
                fprintf(f, "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"0483\", ATTRS{idProduct}==\"5750\", ATTR{bInterval}=\"1\"\n");
                fclose(f);
                system("udevadm control --reload-rules");
                system("udevadm trigger");
            }

            // 4. Disable USB autosuspend for our device
            system("for dev in /sys/bus/usb/devices/*/idVendor; do "
                "  if [ -f $dev ] && [ \"$(cat $dev)\" = \"0483\" ]; then "
                "    echo on > $(dirname $dev)/power/control 2>/dev/null || true; "
                "    echo -1 > $(dirname $dev)/power/autosuspend_delay_ms 2>/dev/null || true; "
                "  fi "
                "done");

            // 5. Set maximum performance CPU governor
            system("echo performance | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor >/dev/null 2>&1 || true");
        }
#endif
    }

    EXPORT void cancel_capture() {
        Continue_Flag = false;
    }

    EXPORT void print_hid_reports() {
        std::vector<uint8_t> report;
        while (ReadHIDInputReportFromQueue()) {
            printf("Received report: ");
            for (int i = 0; i < RxNum; ++i) {
                printf("%02x ", RxData[i]);
            }
            printf("\n");
        }
    }

    EXPORT int check_data_flow() {
        // Directly implement the function here
        int count = 0;
        while (ReadHIDInputReportFromQueue()) {
            count++;
        }
        return count;
    }
}