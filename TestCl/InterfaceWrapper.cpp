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

// C++ linkage function
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
        theInterfaceObject.CaptureFrame12(chan);
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
        // Set process priority
        nice(-20);

        // If running as root, we can lock pages in memory
        if (geteuid() == 0) {
            mlockall(MCL_CURRENT | MCL_FUTURE);
        }

        // Disable CPU scaling to maintain consistent performance
        system("echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor >/dev/null 2>&1");
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
}