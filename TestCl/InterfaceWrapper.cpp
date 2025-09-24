//#include "stdafx.h"
#include "InterfaceObj.h"
#include "HidMgr.h"
#include <cstdio>
#include <vector>

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

extern "C" {

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

    EXPORT int check_data_flow() {
        // Try to read from the queue without blocking
        int count = 0;
        while (ReadHIDInputReportFromQueue()) {
            count++;
        }
        return count; // Return how many reports were in the queue
    }

    EXPORT void optimize_for_pi() {
    #ifdef __linux__
        // Set process priority
        nice(-20);  // Use nice() instead of setpriority()
        
        // If running as root, we can lock pages in memory
        if (geteuid() == 0) {
            mlockall(MCL_CURRENT | MCL_FUTURE);
        }
        
        // Disable CPU scaling to maintain consistent performance
        system("echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor >/dev/null 2>&1");
    #endif
        
        return;
    }

    EXPORT void cancel_capture() {
        Continue_Flag = false;
    }

    // New: Print all queued HID reports (non-blocking)
    EXPORT void print_hid_reports() {
        std::vector<uint8_t> report;
        while (ReadHIDInputReportFromQueue()) {
            // RxData is filled by ReadHIDInputReportFromQueue
            printf("Received report: ");
            for (int i = 0; i < RxNum; ++i) {
                printf("%02x ", RxData[i]);
            }
            printf("\n");
        }
    }
}