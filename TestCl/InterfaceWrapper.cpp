#include "stdafx.h"
#include "InterfaceObj.h"
#include "HidMgr.h"

extern CInterfaceObject theInterfaceObject;

extern "C" {

    __declspec(dllexport) void selchan(int chan) {
        theInterfaceObject.SelSensor(chan);
    }

    __declspec(dllexport) void get(int chan) {
        theInterfaceObject.CaptureFrame12(chan);
    }
    
    __declspec(dllexport) void get_frame12(int* outbuf) {
        for (int i = 0; i < 12; ++i) {
            for (int j = 0; j < 12; ++j) {
                outbuf[i * 12 + j] = theInterfaceObject.frame_data[i][j];
            }
        }
    }

    __declspec(dllexport) void setinttime(float itime) {
        theInterfaceObject.SetIntTime(itime);
    }

    __declspec(dllexport) void setgain(int gain) {
        theInterfaceObject.SetGainMode(gain);
    }

    __declspec(dllexport) void reset() {
        FindTheHID();
    }
}