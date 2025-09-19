#include "InterfaceObj.h"
#include "HidMgr.h"

extern CInterfaceObject theInterfaceObject;

extern "C" {

    void selchan(int chan) {
        theInterfaceObject.SelSensor(chan);
    }

    void get(int chan) {
        theInterfaceObject.CaptureFrame12(chan);
        // Optionally, add code to export data if needed
    }

    void setinttime(float itime) {
        theInterfaceObject.SetIntTime(itime);
    }

    void setgain(int gain) {
        theInterfaceObject.SetGainMode(gain);
    }

    void reset() {
        FindTheHID();
    }

}