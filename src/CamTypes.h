#ifndef CAMERAIDS_CAMERATYPES_HPP_
#define CAMERAIDS_CAMERATYPES_HPP_

namespace camera {

/* To transport the informations received with is_CaptureStatus. */
struct CaptureStatus { 

    unsigned int noDestMem;
    unsigned int conversionFailed;
    unsigned int imageLocked;
    unsigned int outOfBuffers;
    unsigned int deviceNotReady;
    unsigned int transferFailed;
    unsigned int timeout;
    unsigned int bufferOverrun;
    unsigned int missedImages;
    unsigned int totalCount;

    CaptureStatus () {
        noDestMem=0;
        conversionFailed=0;
        imageLocked=0;
        outOfBuffers=0;
        deviceNotReady=0;
        transferFailed=0;
        timeout=0;
        bufferOverrun=0;
        missedImages=0;
    }
};

}

#endif
