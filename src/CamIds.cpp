/**
 * Implementation for the IDS camera driver.
 * @file CamIds.cpp
 * @author Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
 * @date Thu Dec 8 13:25:02 CET 2011
 */

#include "CamIds.h"
#include <iostream>

using namespace base::samples::frame;

namespace camera {

//==============================================================================
CamInfo CamIds::fillCamInfo(UEYE_CAMERA_INFO *uEyeCamInfo, HIDS *camHandle) const {
    // the camera information to be returned
    CamInfo camInfo;

    // fill in the camera information for each one
    camInfo.unique_id         = uEyeCamInfo->dwCameraID;
    camInfo.serial_string     = uEyeCamInfo->SerNo;
    camInfo.display_name      = uEyeCamInfo->Model;
    camInfo.interface_id      = uEyeCamInfo->dwSensorID;
    camInfo.device            = "unknown";
    camInfo.reachable         = true;   // assume reachable initially

    // temporary uEye camera info to fetch more info on current cam
    CAMINFO moreCamInfo;

    if (is_GetCameraInfo(*camHandle, &moreCamInfo) != IS_SUCCESS) {
        std::cerr << "** Camera "
                << "dwCameraID(" << uEyeCamInfo->dwCameraID << "), "
                << "dwDeviceID(" << uEyeCamInfo->dwDeviceID << ")"
                << " not found, or device is already in use!" << std::endl;
    }

    if (moreCamInfo.Type == IS_CAMERA_TYPE_UEYE_USB
            || moreCamInfo.Type == IS_CAMERA_TYPE_UEYE_USB_LE
            || moreCamInfo.Type == IS_CAMERA_TYPE_UEYE_USB_SE
            || moreCamInfo.Type == IS_CAMERA_TYPE_UEYE_USB_ME
            || moreCamInfo.Type == IS_CAMERA_TYPE_UEYE_USB_RE)
    {
        // mark the appropriate interface
        camInfo.interface_type = InterfaceUSB;
    }
    else if (moreCamInfo.Type == IS_CAMERA_TYPE_UEYE_ETH
            || moreCamInfo.Type == IS_CAMERA_TYPE_UEYE_ETH_CP
            || moreCamInfo.Type == IS_CAMERA_TYPE_UEYE_ETH_HE
            || moreCamInfo.Type == IS_CAMERA_TYPE_UEYE_ETH_RE
            || moreCamInfo.Type == IS_CAMERA_TYPE_UEYE_ETH_SE)
    {
        // mark the appropriate interface
        camInfo.interface_type = InterfaceEthernet;

        // get information about this Ethernet camera
        UEYE_ETH_DEVICE_INFO ethDeviceInfo;

        if (is_GetEthDeviceInfo(uEyeCamInfo->dwDeviceID | IS_USE_DEVICE_ID,
                &ethDeviceInfo, sizeof(UEYE_ETH_DEVICE_INFO))
                != IS_SUCCESS)
        {
            // unknown interface camera attached
            std::cerr << "** Ethernet camera not found: "
                    << "dwCameraID(" << uEyeCamInfo->dwCameraID << "), "
                    << "dwDeviceID(" << uEyeCamInfo->dwDeviceID << ")"
                    <<std::endl;
        }

        // start with unknown config
        camInfo.ip_settings.config_mode = IpConfigUnknown;

        // using persistent ip
        if (ethDeviceInfo.infoDevControl.dwControlStatus | IS_ETH_CTRLSTATUS_PERSISTENT_IP_USED) {
            camInfo.ip_settings.config_mode = IpConfigPersistent;
            camInfo.ip_settings.config_mode_support |= IpConfigPersistent;
        }

        // using Auto IP
        if (ethDeviceInfo.infoAdapter.bIsValidAutoCfgIpRange) {
            camInfo.ip_settings.config_mode = IpConfigAutoIp;
            camInfo.ip_settings.config_mode_support |= IpConfigAutoIp;
        }

        // using DHCP, preferred
        if (ethDeviceInfo.infoAdapter.bIsEnabledDHCP) {
            camInfo.ip_settings.config_mode = IpConfigDhcp;
            camInfo.ip_settings.config_mode_support |= IpConfigDhcp;
        }

        // current ip address
        UEYE_ETH_ADDR_IPV4 currIP = ethDeviceInfo.infoDevHeartbeat.ipcfgCurrentIpCfg.ipAddress;

        camInfo.ip_settings.current_ip_address =
                (uint32_t)currIP.by.by1 << 24
                | (uint32_t)currIP.by.by2 << 16
                | (uint32_t)currIP.by.by3 << 8
                | (uint32_t)currIP.by.by4 << 0;

        // current subnet mask
        UEYE_ETH_ADDR_IPV4 currSN = ethDeviceInfo.infoDevHeartbeat.ipcfgCurrentIpCfg.ipSubnetmask;

        camInfo.ip_settings.current_ip_subnet =
                (uint32_t)currSN.by.by1 << 24
                | (uint32_t)currSN.by.by2 << 16
                | (uint32_t)currSN.by.by3 << 8
                | (uint32_t)currSN.by.by4 << 0;

        // persistent ip address
        UEYE_ETH_ADDR_IPV4 persIP = ethDeviceInfo.infoDevHeartbeat.ipcfgPersistentIpCfg.ipAddress;

        camInfo.ip_settings.current_ip_address =
                (uint32_t)persIP.by.by1 << 24
                | (uint32_t)persIP.by.by2 << 16
                | (uint32_t)persIP.by.by3 << 8
                | (uint32_t)persIP.by.by4 << 0;

        // persistent subnet mask
        UEYE_ETH_ADDR_IPV4 persSN = ethDeviceInfo.infoDevHeartbeat.ipcfgPersistentIpCfg.ipSubnetmask;

        camInfo.ip_settings.current_ip_subnet =
                (uint32_t)persSN.by.by1 << 24
                | (uint32_t)persSN.by.by2 << 16
                | (uint32_t)persSN.by.by3 << 8
                | (uint32_t)persSN.by.by4 << 0;

        // is the camera on the same subnet as the host
        UEYE_ETH_ADDR_IPV4 hostIP = ethDeviceInfo.infoDevHeartbeat.ipPairedHostIp;

        uint32_t subNetHost =
                hostIP.by.by1 | currSN.by.by1
                | hostIP.by.by2 | currSN.by.by2
                | hostIP.by.by3 | currSN.by.by3
                | hostIP.by.by4 | currSN.by.by4;

        uint32_t subNetCamera = currIP.by.by1 | currSN.by.by1
                | currIP.by.by2 | currSN.by.by2
                | currIP.by.by3 | currSN.by.by3
                | currIP.by.by4 | currSN.by.by4;

        // set the reachable field
        if (subNetHost == subNetCamera) {
            camInfo.reachable = true;
        } else {
            camInfo.reachable = false;
        }
    } else {
        // set the interface type to unknown
        camInfo.interface_type = InterfaceUnknown;
        std::cerr << "** Unknown camera type: "
                << "dwCameraID(" << uEyeCamInfo->dwCameraID << "), "
                << "dwDeviceID(" << uEyeCamInfo->dwDeviceID << ")" << std::endl;
    }
    return camInfo;
}

//==============================================================================
CamIds::CamIds() {
    this->pCam_          = NULL;
    this->pCamInfo_      = NULL;
    this->pFrameBuf_     = NULL;
    this->nFrameBufLen_  = 0;
    this->nSeqCount_     = 0;
}

//==============================================================================
CamIds::CamIds(const CamIds& other) {
    this->pCam_          = other.pCam_;
    this->pCamInfo_      = other.pCamInfo_;
    this->pFrameBuf_     = other.pFrameBuf_;
    this->nFrameBufLen_  = other.nFrameBufLen_;
    this->nSeqCount_     = other.nSeqCount_;
}

//==============================================================================
CamIds::~CamIds() {
    std::cout << "== Destructing camera" << std::endl;

    // release all memory

    // free the image memory allocated
    for (int i = 0; i < this->nFrameBufLen_; ++i) {
        is_FreeImageMem(*this->pCam_,
                this->pFrameBuf_[i].pBuf, this->pFrameBuf_[i].nImageID);
    }

    if (this->pFrameBuf_ != NULL) {
        delete[] this->pFrameBuf_;
    }

    if (this->pCamInfo_ != NULL) {
        delete this->pCamInfo_;
    }

    if (this->pCam_ != NULL) {
        // also release the camera
        INT retVal = is_ExitCamera(*this->pCam_);

        delete this->pCam_;
    }
}

//==============================================================================
int CamIds::listCameras(std::vector<CamInfo> &cam_infos) const {
    // how many cameras do we have?
    INT numCam = countCameras();

    // proceed if we have some cameras
    if (numCam <= 0) {
        std::cerr << "** No cameras found!" << std::endl;
        return 0;
    }

    // place-holder for the camera list
    UEYE_CAMERA_LIST* cameraList;
    cameraList = (UEYE_CAMERA_LIST*) new BYTE [sizeof (DWORD) + numCam * sizeof (UEYE_CAMERA_INFO)];

    // set the number of cameras inside the struct
    cameraList->dwCount = (ULONG) numCam;

    // get the camera list
    if (is_GetCameraList(cameraList) != IS_SUCCESS) {
        std::cerr << "** Unable to retrieve camera list!" << std::endl;
        delete[] cameraList;
        return 0;
    }

    // loop over the cameras and add them to the list
    for (int iCamera = 0; iCamera < (int)cameraList->dwCount; ++iCamera) {

        // handle for the current camera
        UEYE_CAMERA_INFO *uEyeCamInfo = &cameraList->uci[iCamera];

        // window handler for init camera
        HIDS camHandle = uEyeCamInfo->dwCameraID;
        HWND hWnd;

        // open the camera temporarily to retrieve the data
        if (is_InitCamera(&camHandle, &hWnd) != IS_SUCCESS) {
            std::cout << "** Cannot initialize camera "
                    << "dwCameraID(" << uEyeCamInfo->dwCameraID << "), "
                    << "dwDeviceID(" << uEyeCamInfo->dwDeviceID << ")" << std::endl;
        }

        // to be pushed back into the camera list vector
        CamInfo camInfo;
        camInfo = fillCamInfo(&cameraList->uci[iCamera], &camHandle);

        // add the new camera to the list
        cam_infos.push_back(camInfo);

        // release camera
        is_ExitCamera(camHandle);
    }

    // release memory
    delete[] cameraList;

    // must return number of listed cameras
    return cameraList->dwCount;
}

//==============================================================================
int CamIds::countCameras() const {
    INT numberOfCameras = 0;
    INT retVal = is_GetNumberOfCameras(&numberOfCameras);

    if (retVal != IS_SUCCESS) {
        std::cerr << "** Unable to retrieve number of cameras!" << std::endl;
        return 0;
    }
    else {
        return numberOfCameras;
    }
}

//==============================================================================
bool CamIds::open(const CamInfo& cam, const AccessMode mode) {
    std::cout << "== Opening camera number " << cam.unique_id
            << " (" << cam.display_name << ")" << std::endl;

    // cannot open camera a second time
    if (isOpen()) {
        std::cerr << "** Camera already open for current object!" << std::endl;
        return true;
    }

    // wait until camera is available
    is_EnableEvent(cam.unique_id, IS_SET_EVENT_STATUS_CHANGED);
    is_WaitEvent(cam.unique_id, IS_SET_EVENT_STATUS_CHANGED, 4000);

    // disable the event from the beginning of the function
    is_DisableEvent(cam.unique_id, IS_SET_EVENT_STATUS_CHANGED);

    //--------------------------------------------------------------------------
    // prepare to initialize camera
    //--------------------------------------------------------------------------
    // allocate memory for camera handle and camera info
    this->pCam_        = new HIDS;
    this->pCamInfo_    = new CamInfo;

    *this->pCam_ = (HIDS) cam.unique_id;
    HWND hWnd;

    // attempt to open
    INT nRet = is_InitCamera(pCam_, &hWnd);

    if (nRet != IS_SUCCESS) {
        std::cerr << "** Could not open camera " << cam.unique_id << " (" << cam.display_name << ")" << std::endl;
        this->close();
        return false;
    }

    // camera successfully opened
    std::cout << "== Camera " << cam.unique_id
            << " (" << cam.display_name << ")"
            << " successfully opened " << std::endl;

    //----------------------------------------------------------------------
    // fetch camera information
    //----------------------------------------------------------------------
    INT numCam = countCameras();

    // place-holder for the camera list
    UEYE_CAMERA_LIST* cameraList;
    cameraList = (UEYE_CAMERA_LIST*) new BYTE[sizeof(DWORD) + numCam * sizeof(UEYE_CAMERA_INFO)];

    // set the number of cameras inside the struct
    cameraList->dwCount = (ULONG) numCam;

    // proceed if we have some cameras
    if (cameraList->dwCount <= 0) {
        std::cout << "** No cameras found! Please check if properly attached!" << std::endl;
        this->close();
        return false;
    }

    // get the camera list
    if (is_GetCameraList(cameraList) != IS_SUCCESS) {
        std::cerr << "** Unable to retrieve camera list while opening camera!" << std::endl;
        this->close();
        return false;
    }

    // loop over the cameras and add them to the list
    for (int iCamera = 0; iCamera < (int)cameraList->dwCount; ++iCamera) {

        // handle for the current camera
        UEYE_CAMERA_INFO *uEyeCamInfo = &cameraList->uci[iCamera];

        // search for camera by unique id
        // WARNING set the camera ids in the uEye camera manager
        if(uEyeCamInfo->dwCameraID == cam.unique_id) {
            // fill in the camera information
            *this->pCamInfo_ = fillCamInfo(uEyeCamInfo, pCam_);
            break;
        }
    }

    //----------------------------------------------------------------------
    // fetch sensor information
    //----------------------------------------------------------------------
    SENSORINFO sensorInfo;

    if (is_GetSensorInfo(*this->pCam_, &sensorInfo) != IS_SUCCESS) {
        std::cerr << "** Failed to fetch sensor information for camera "
                << cam.unique_id << " (" << cam.display_name << ")" << std::endl;
        this->close();
        return false;
    }

    // set image size to the camera maximum
    this->image_size_.height    = (uint16_t) sensorInfo.nMaxHeight;
    this->image_size_.width     = (uint16_t) sensorInfo.nMaxWidth;

    std::cout << "== Image size set to "
            << this->image_size_.width << " x " << this->image_size_.height << std::endl;

    // color depth in bytes, set default RGB24
    if (is_SetColorMode(*this->pCam_, IS_CM_BGR8_PACKED) != IS_SUCCESS) {
        std::cerr << "** Unable to set color mode for camera "
                << cam.unique_id << " (" << cam.display_name << ")" << std::endl;
        this->close();
        return false;
    }

    this->image_mode_         = MODE_RGB;
    this->image_color_depth_  = 3;

    // initialize the area of interest dimensions
    IS_RECT rectAOI;

    rectAOI.s32Width    = this->image_size_.width;
    rectAOI.s32Height   = this->image_size_.height;
    rectAOI.s32X        = 0 | IS_AOI_IMAGE_POS_ABSOLUTE;
    rectAOI.s32Y        = 0 | IS_AOI_IMAGE_POS_ABSOLUTE;

    // set the area of interest of the camera
    is_AOI(*this->pCam_, IS_AOI_IMAGE_SET_AOI, (void*)&rectAOI, sizeof(rectAOI));

    // set the trigger mode to software, image acquired when calling is_FreezeVideo()
    if (is_SetExternalTrigger(*this->pCam_, IS_SET_TRIGGER_OFF) != IS_SUCCESS) {
        std::cerr << "** Unable to set trigger mode for camera "
                << this->pCamInfo_->unique_id
                << " (" << this->pCamInfo_->display_name << ")"
                << " while attempting to open" << std::endl;
        this->close();
        return false;
    }

    double newFps;
    if (is_SetFrameRate(*this->pCam_, 50, &newFps) != IS_SUCCESS) {
        std::cerr << "** Unable to set frame rate for camera "
                << this->pCamInfo_->unique_id
                << " (" << this->pCamInfo_->display_name << ")"
                << " while attempting to open" << std::endl;
        this->close();
        return false;
    }

    std::cout << "== Frame rate set to " << newFps << std::endl;

    // initial grab mode set to stop
    this->act_grab_mode_ = Stop;

    // release memory
    delete[] cameraList;

    return true;
}

//==============================================================================
bool CamIds::isOpen() const {
    if (this->pCam_ == NULL) {
        return false;
    }
    return true;
}

//==============================================================================
const CamInfo* CamIds::getCameraInfo() const {
    return pCamInfo_;
}

//==============================================================================
bool CamIds::close() {
    // release all memory
    std::cout << "== Closing camera" << std::endl;

    // clear the queue, stop live video
    if (this->act_grab_mode_ == Continuously) {
        is_StopLiveVideo(*this->pCam_, IS_WAIT);
        is_ClearSequence(*this->pCam_);
        is_ExitImageQueue(*this->pCam_);
    }

    // free the image memory allocated
    for (int i = 0; i < this->nFrameBufLen_; ++i) {
        is_FreeImageMem(*this->pCam_,
                this->pFrameBuf_[i].pBuf, this->pFrameBuf_[i].nImageID);
    }

//    if (this->act_grab_mode_ == Continuously) {
//        is_DisableEvent(*this->pCam_, IS_SET_EVENT_FRAME);
//    }

    if (this->pFrameBuf_ != NULL) {
        delete[] this->pFrameBuf_;

        // make sure to mark as NULL
        this->pFrameBuf_      = NULL;
        this->nFrameBufLen_   = 0;
    }

    if (this->pCamInfo_ != NULL) {
        delete this->pCamInfo_;

        // make sure to mark as NULL
        this->pCamInfo_ = NULL;
    }

    if (this->pCam_ != NULL) {
        // also release the camera
        INT retVal = is_ExitCamera(*this->pCam_);

        // make sure to mark as NULL
        delete this->pCam_;
        this->pCam_ = NULL;

        if (retVal != IS_SUCCESS) {
            return false;
        }
    }

    return true;
}

//==============================================================================
bool CamIds::grab(const GrabMode mode, const int buffer_len) {
    //--------------------------------------------------------------------------
    // cannot grab if camera is not open
    //--------------------------------------------------------------------------
    if (this->isOpen() == false) {
        std::cerr << "** Cannot grab, camera is not open!" << std::endl;
        return false;
    }

    // used to check return values of functions
    INT retVal;

    std::cout << "== Setting grab mode ";
    switch(mode) {
    case Stop:
        // never reached
        std::cout << "Stop";
        break;
    case SingleFrame:
        std::cout << "SingleFrame";
        break;
    case MultiFrame:
        std::cout << "MultiFrame";
        break;
    case Continuously:
        std::cout << "Continuously";
        break;
    default:
        throw std::runtime_error("Grab mode not supported by camera!");
    }
    std::cout << " for camera "
            << this->pCamInfo_->unique_id
            << " (" << this->pCamInfo_->display_name << ")" << std::endl;

    //--------------------------------------------------------------------------
    // check the current and the requested grab mode
    //--------------------------------------------------------------------------
    if (this->act_grab_mode_ != Stop && mode != Stop) {
        // we need to stop grabbing before switching grab mode
        if (this->act_grab_mode_ != mode) {
            std::cerr << "Stop grabbing before switching the grab mode!" << std::endl;
            return false;
        }
        else {
            std::cout << "== Same grab mode requested!" << std::endl;
            return true;
        }
    }

    switch(mode) {
    //--------------------------------------------------------------------------
    // stop grabbing
    //--------------------------------------------------------------------------
    case Stop:
        // we may onlt call Stop after a continuous capture
        if (this->act_grab_mode_ != Continuously) {
            throw std::runtime_error("Can only stop after live video!");
        }

        // stop live video
        if (is_StopLiveVideo(*this->pCam_, IS_WAIT) != IS_SUCCESS) {
            std::cerr << "Unable to stop live capture for camera "
                    << this->pCamInfo_->unique_id
                    << " (" << this->pCamInfo_->display_name << ")" << std::endl;
            return false;
        }

        // disable events and exit the image queue
        is_ClearSequence(*this->pCam_);
//        is_DisableEvent(*this->pCam_, IS_SET_EVENT_FRAME);
        is_ExitImageQueue(*this->pCam_);

        // free the image memory allocated
        for (int i = 0; i < this->nFrameBufLen_; ++i) {
            is_FreeImageMem(*this->pCam_,
                    this->pFrameBuf_[i].pBuf, this->pFrameBuf_[i].nImageID);
        }

        // clear the content of the buffer
        if (this->pFrameBuf_ != NULL) {
            delete[] pFrameBuf_;

            // make sure to mark as NULL
            this->pFrameBuf_    = NULL;
            this->nFrameBufLen_ = 0;
        }

        break;

    //--------------------------------------------------------------------------
    // grab a single frame
    //--------------------------------------------------------------------------
    case SingleFrame:
        // resize the buffer
        if (this->pFrameBuf_ != NULL) {
            delete[] this->pFrameBuf_;
        }
        this->pFrameBuf_ = new UEYE_IMAGE[sizeof(UEYE_CAMERA_INFO)];
        this->nFrameBufLen_ = 1;

        // allocate the memory for the frame
        retVal = is_AllocImageMem(*this->pCam_,
                this->image_size_.width, this->image_size_.height,
                this->image_color_depth_ * 8,
                &this->pFrameBuf_[0].pBuf, &this->pFrameBuf_[0].nImageID);

        if (retVal != IS_SUCCESS) {
            std::cerr << "** Unable to allocate image memory for camera "
                    << this->pCamInfo_->unique_id
                    << " (" << this->pCamInfo_->display_name << ")"
                    << " while attempting to grab" << std::endl;
            this->close();
            return false;
        }

        this->pFrameBuf_[0].nImageSeqNum = 0;
        this->pFrameBuf_[0].nBufferSize = this->image_size_.width * this->image_size_.height
                * this->image_color_depth_;

        // set the image memory to the allocated one
        if (is_SetImageMem(*this->pCam_, this->pFrameBuf_[0].pBuf, this->pFrameBuf_[0].nImageID)
                != IS_SUCCESS) {
            std::cerr << "** Unable to set image memory for camera "
                    << this->pCamInfo_->unique_id
                    << " (" << this->pCamInfo_->display_name << ")"
                    << " while attempting to grab" << std::endl;
            this->close();
            return false;
        }

        // set display mode
        if (is_SetDisplayMode(*this->pCam_, IS_SET_DM_DIB) != IS_SUCCESS) {
            // setting fails if we try to set the same display mode
            INT mode = is_SetDisplayMode(*this->pCam_, IS_GET_DISPLAY_MODE);

            if (mode != IS_SET_DM_DIB) {
                std::cerr << "** Unable to set display mode for camera "
                        << this->pCamInfo_->unique_id
                        << " (" << this->pCamInfo_->display_name << ")"
                        << " while attempting to grab" << std::endl;
                this->close();
                return false;
            }
        }
        break;

    //--------------------------------------------------------------------------
    // grab a fixed number of frames
    //--------------------------------------------------------------------------
    case MultiFrame:
        //TODO not supported yet
        throw std::runtime_error("MultiFrame not supported yet!");
        break;

    //--------------------------------------------------------------------------
    // capture continuous video
    //--------------------------------------------------------------------------
    case Continuously:
        // resize the buffer
        if (this->pFrameBuf_ != NULL) {
            delete[] this->pFrameBuf_;
        }
        this->pFrameBuf_ = new UEYE_IMAGE[buffer_len * sizeof(UEYE_CAMERA_INFO)];
        this->nFrameBufLen_ = buffer_len;

        // allocate the memory
        for (int i = 0; i < buffer_len; ++i) {
            retVal = is_AllocImageMem(*this->pCam_,
                    this->image_size_.width, this->image_size_.height,
                    this->image_color_depth_ * 8,
                    &this->pFrameBuf_[i].pBuf, &this->pFrameBuf_[i].nImageID);

            if (retVal != IS_SUCCESS) {
                std::cerr << "** Unable to allocate image memory for camera "
                        << this->pCamInfo_->unique_id
                        << " (" << this->pCamInfo_->display_name << ")"
                        << " while attempting to grab" << std::endl;
                this->close();
                return false;
            }

            this->pFrameBuf_[i].nImageSeqNum = i;
            this->pFrameBuf_[i].nBufferSize = this->image_size_.width * this->image_size_.height
                    * this->image_color_depth_;

            // register the allocated memory
            is_AddToSequence(*this->pCam_,
                    this->pFrameBuf_[i].pBuf, this->pFrameBuf_[i].nImageID);
        }

        // install event handling
//        is_EnableEvent(*this->pCam_, IS_SET_EVENT_FRAME);

        // enable queueing mode
        is_InitImageQueue(*this->pCam_, 0);

        // start capturing video
        is_CaptureVideo(*this->pCam_, IS_WAIT); //TODO play around with wait values
        break;

    //--------------------------------------------------------------------------
    // should never reach here if used correctly
    //--------------------------------------------------------------------------
    default:
        throw std::runtime_error("Grab mode not supported by camera!");
    }

    // update the current mode
    this->act_grab_mode_ = mode;

    return true;
}

//==============================================================================
bool CamIds::retrieveFrame(base::samples::frame::Frame& frame, const int timeout) {
    // a return value dummy integer to be used around the method
    INT retVal;

    // used to retrieve data about next buffered frame
    char* pTempBuf;
    int nTempImageID;

    // used to retrieve more info on an image
    UEYEIMAGEINFO imgInfo;

    // depending on the active grabbing mode, acquire frame
    switch (this->act_grab_mode_) {
    case Stop:
        std::cerr << "** Active mode is set to Stop! Cannot retrieve frame!" << std::endl;
        return false;
        break;
    case SingleFrame:
        // freeze the video for the single frame acquisition
        if (is_FreezeVideo(*this->pCam_, timeout) != IS_SUCCESS) {
            std::cerr << "** Unable to freeze video for camera "
                    << this->pCamInfo_->unique_id
                    << " (" << this->pCamInfo_->display_name << ")"
                    << " while attempting to retrieve frame" << std::endl;
            // set the status of the frame to invalid
            frame.setStatus(STATUS_INVALID);
            return false;
        }

        // get more information on the image
        if (is_GetImageInfo(*this->pCam_, this->pFrameBuf_[0].nImageID, &imgInfo, sizeof(imgInfo))
                != IS_SUCCESS) {
            std::cerr << "** Unable to retrieve image info for camera "
                    << this->pCamInfo_->unique_id
                    << " (" << this->pCamInfo_->display_name << ")"
                    << " while attempting to retrieve frame" << std::endl;
            // set the status of the frame to invalid
            frame.setStatus(STATUS_INVALID);
            return false;
        }

        // begin to fill in the output frame
        frame.image.assign(this->pFrameBuf_[0].pBuf, this->pFrameBuf_[0].pBuf + this->pFrameBuf_[0].nBufferSize);

        // set the rolling frame counter
        frame.attributes.clear();
        frame.setAttribute<uint64_t>("FrameCount", imgInfo.u64FrameNumber);

        // set status to valid as we succeeded capturing the frame
        frame.setStatus(STATUS_VALID);

        // in imgInfo the device time is in 0.1 microS
        frame.time.fromMicroseconds(10 * imgInfo.u64TimestampDevice);
        frame.received_time = frame.time;

        // set the color mode of the frame, we are using RGB24
        frame.frame_mode = this->image_mode_;
        frame.pixel_size = this->image_color_depth_;
        frame.data_depth = 8;

        // set dimensions, row size in bytes, image size in pixels
        frame.row_size      = imgInfo.dwImageWidth * this->image_color_depth_;
        frame.size.width    = imgInfo.dwImageWidth;
        frame.size.height   = imgInfo.dwImageHeight;
        break;

    case MultiFrame:
        throw std::runtime_error("MultiFrame not supported yet!");
        break;

    case Continuously:
        // get a frame
        if (is_WaitForNextImage(*this->pCam_, timeout, &pTempBuf, &nTempImageID) != IS_SUCCESS) {
            std::cerr << "** Unable to retrieve next image for camera "
                    << this->pCamInfo_->unique_id
                    << " (" << this->pCamInfo_->display_name << ")"
                    << " while attempting to retrieve frame" << std::endl;
            // set the status of the frame to invalid
            frame.setStatus(STATUS_INVALID);
            return false;
        }

        // get more information on the image
        if (is_GetImageInfo(*this->pCam_, nTempImageID, &imgInfo, sizeof(imgInfo))
                != IS_SUCCESS) {
            std::cerr << "** Unable to retrieve image info for camera "
                    << this->pCamInfo_->unique_id
                    << " (" << this->pCamInfo_->display_name << ")"
                    << " while attempting to retrieve frame" << std::endl;
            // set the status of the frame to invalid
            frame.setStatus(STATUS_INVALID);
            return false;
        }

        // begin to fill in the output frame
        frame.image.assign(pTempBuf,
                pTempBuf + imgInfo.dwImageHeight * imgInfo.dwImageWidth * this->image_color_depth_);

        // set the rolling frame counter
        frame.attributes.clear();
        frame.setAttribute<uint64_t>("FrameCount", imgInfo.u64FrameNumber);

        // set status to valid as we succeeded capturing the frame
        frame.setStatus(STATUS_VALID);

        // in imgInfo the device time is in 0.1 microS
        frame.time.fromMicroseconds(10 * imgInfo.u64TimestampDevice);
        frame.received_time = frame.time;

        // set the color mode of the frame, we are using RGB24
        frame.frame_mode = this->image_mode_;
        frame.pixel_size = this->image_color_depth_;
        frame.data_depth = 8;

        // set dimensions, row size in bytes, image size in pixels
        frame.row_size      = imgInfo.dwImageWidth * this->image_color_depth_;
        frame.size.width    = imgInfo.dwImageWidth;
        frame.size.height   = imgInfo.dwImageHeight;

        // unlock the buffer, in queue mode, image buffers are automatically locked
        is_UnlockSeqBuf(*this->pCam_, nTempImageID, pTempBuf);
        break;
    default:
        throw std::runtime_error("Grab mode not supported by camera!");
    }

    return true;
}

//==============================================================================
bool CamIds::isFrameAvailable() {
    // get the current camera sequence count
    int nTempSeqCount = is_CameraStatus(*this->pCam_, IS_SEQUENCE_CNT, IS_GET_STATUS);
    int temp = this->nSeqCount_;
    this->nSeqCount_ = nTempSeqCount;

    if (this->act_grab_mode_ != Stop && nTempSeqCount != temp) {
        std::cout << "== New frame available with sequence count "
                << nTempSeqCount
                << ". Previous frame sequence count is "
                << temp<< "." << std::endl;
        return true;
    }

    std::cout << "== New frame not available!" << std::endl;
    return false;
}

//==============================================================================
bool CamIds::setFrameSettings(const base::samples::frame::frame_size_t size,
        const base::samples::frame::frame_mode_t mode,
        const uint8_t color_depth,
        const bool resize_frames)
{
    // no camera
    if (!this->pCam_) {
        return false;
    }

    if (this->act_grab_mode_ != Stop) {
        std::cerr << "Cannot change camera setting during grabbing!" << std::endl;
        return false;
    }

    // get details about the camera sensor
    SENSORINFO sensorInfo;

    if (is_GetSensorInfo(*this->pCam_, &sensorInfo) != IS_SUCCESS) {
        std::cerr << "** Failed to fetch sensor information for camera "
                << this->pCamInfo_->unique_id
                << " (" << this->pCamInfo_->display_name << ")"
                << "while setting frame settings!" << std::endl;
        return false;
    }

    // check if the frame size is supported by the camera
    if (size.width > sensorInfo.nMaxWidth || size.height > sensorInfo.nMaxHeight) {
        std::cerr << "** Frame size not supported!" << std::endl;
        return false;
    }



    return true;
}

//==============================================================================
bool CamIds::getFrameSettings(base::samples::frame::frame_size_t& size,
        base::samples::frame::frame_mode_t& mode,
        uint8_t& color_depth)
{

    return true;
}

//==============================================================================
bool CamIds::isAttribAvail(const int_attrib::CamAttrib attrib) {
    return false;
}

//==============================================================================
bool CamIds::isAttribAvail(const double_attrib::CamAttrib attrib) {
    return false;
}

//==============================================================================
bool CamIds::isAttribAvail(const enum_attrib::CamAttrib attrib) {
    return false;
}
//==============================================================================

} // end of camera namespace
