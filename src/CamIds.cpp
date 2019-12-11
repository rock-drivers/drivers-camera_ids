/**
 * Implementation for the IDS camera driver.
 * @file CamIds.cpp
 * @author Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
 * @date Thu Dec 8 13:25:02 CET 2011
 */

#include "CamIds.h"
#include <base/Time.hpp>
#include <iostream>
#include <boost/current_function.hpp>
#include <base-logging/Logging.hpp>

using namespace base::samples::frame;

namespace camera {

CamIds::CamIds() {
    this->pCam_          = NULL;
    this->pCamInfo_      = NULL;
    this->pFrameBuf_     = NULL;
    this->nFrameBufLen_  = 0;
    this->nSeqCount_     = 0;
    this->mEventTimeout = 10;
    this->mGetEveryFrame = false;
}

CamIds::CamIds(const CamIds& other) {
    this->pCam_          = other.pCam_;
    this->pCamInfo_      = other.pCamInfo_;
    this->pFrameBuf_     = other.pFrameBuf_;
    this->nFrameBufLen_  = other.nFrameBufLen_;
    this->nSeqCount_     = other.nSeqCount_;
    this->mEventTimeout = 10;
    this->mGetEveryFrame = false;
}

CamIds::~CamIds() {
    LOG_INFO_S << "Destructing camera";

    if (this->pFrameBuf_)
        clearBuffers();

    if (this->pCamInfo_ != NULL)
        delete this->pCamInfo_;

    if (this->pCam_ != NULL) {
        is_ExitCamera(*this->pCam_);
        delete this->pCam_;
    }
}

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
        LOG_ERROR_S << "Camera "
                << "dwCameraID(" << uEyeCamInfo->dwCameraID << "), "
                << "dwDeviceID(" << uEyeCamInfo->dwDeviceID << ")"
                << " not found, or device is already in use!";
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
#if UEYE_VERSION_CODE >= UEYE_VERSION(4, 0, 0)
        if (is_DeviceInfo(uEyeCamInfo->dwDeviceID | IS_USE_DEVICE_ID,
                IS_DEVICE_INFO_CMD_GET_DEVICE_INFO, &ethDeviceInfo,
                sizeof(UEYE_ETH_DEVICE_INFO)) != IS_SUCCESS)
#else
        if (is_GetEthDeviceInfo(uEyeCamInfo->dwDeviceID | IS_USE_DEVICE_ID,
                &ethDeviceInfo, sizeof(UEYE_ETH_DEVICE_INFO))
                != IS_SUCCESS)
#endif
        {
            // unknown interface camera attached
            LOG_ERROR_S << "** Ethernet camera not found: "
                    << "dwCameraID(" << uEyeCamInfo->dwCameraID << "), "
                    << "dwDeviceID(" << uEyeCamInfo->dwDeviceID << ")";
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
        LOG_ERROR_S << "Unknown camera type: "
                << "dwCameraID(" << uEyeCamInfo->dwCameraID << "), "
                << "dwDeviceID(" << uEyeCamInfo->dwDeviceID << ")";
    }
    return camInfo;
}


//==============================================================================
int CamIds::listCameras(std::vector<CamInfo> &cam_infos) const {

    INT numCam = countCameras();

    if (numCam <= 0) {
        LOG_ERROR_S << "No cameras found!";
        return 0;
    }

    // place-holder for the camera list
    UEYE_CAMERA_LIST* cameraList;
    cameraList = (UEYE_CAMERA_LIST*) new BYTE [sizeof (ULONG) + numCam * sizeof (UEYE_CAMERA_INFO)];

    // set the number of cameras inside the struct
    cameraList->dwCount = (ULONG) numCam;

    // get the camera list
    if (is_GetCameraList(cameraList) != IS_SUCCESS) {
        LOG_ERROR_S << "Unable to retrieve camera list!";
        delete[] cameraList;
        return 0;
    }

    // loop over the cameras and add them to the list
    for (int iCamera = 0; iCamera < (int)cameraList->dwCount; iCamera++) { //?

        // handle for the current camera
        UEYE_CAMERA_INFO *uEyeCamInfo = &cameraList->uci[iCamera];

        // window handler for init camera
        HIDS camHandle = uEyeCamInfo->dwCameraID;
        HWND hWnd;

        // open the camera temporarily to retrieve the data (must it be opne?)
        if (is_InitCamera(&camHandle, &hWnd) != IS_SUCCESS) {
            LOG_WARN_S << "Cannot initialize (or already open) camera "
                    << "dwCameraID(" << uEyeCamInfo->dwCameraID << "), "
                    << "dwDeviceID(" << uEyeCamInfo->dwDeviceID << ")";
            continue;
        }

        // to be pushed back into the camera list vector
        CamInfo camInfo;
        camInfo = fillCamInfo(&cameraList->uci[iCamera], &camHandle);

        // add the new camera to the list
        cam_infos.push_back(camInfo);

        // release camera
        is_ExitCamera(camHandle);
    }

    delete[] cameraList;

    return numCam;
}


std::string CamIds::doDiagnose() {

    std::stringstream strstr;

    // show CamInfo of the open camera
    const CamInfo *pcam_info;
    try
    {
        pcam_info = getCameraInfo();
        strstr << "Opened Camera:\n";
        strstr << "Unique Id: " << pcam_info->unique_id << "\n";
    }
    catch(std::runtime_error e)
    {
        strstr << e.what() << "\n Cannot display CamInfo of the opened camera. \n";
    }
    return strstr.str();
}

//==============================================================================
int CamIds::countCameras() const {
    INT numberOfCameras = 0;
    INT retVal = is_GetNumberOfCameras(&numberOfCameras);

    if (retVal != IS_SUCCESS) {
        LOG_ERROR_S << "Unable to retrieve number of cameras!";
        return 0;
    }
    else {
        return numberOfCameras;
    }
}

//==============================================================================
bool CamIds::open(const CamInfo& cam, const AccessMode mode) {
    LOG_INFO_S << "Opening camera number " << cam.unique_id
            << " (" << cam.display_name << ")";

    // cannot open camera a second time
    if (isOpen()) {
        LOG_WARN_S << "Camera already open for current object!";
        return true;
    }

    // wait until camera is available
    is_EnableEvent(cam.unique_id, IS_SET_EVENT_STATUS_CHANGED);
    is_WaitEvent(cam.unique_id, IS_SET_EVENT_STATUS_CHANGED, 4000);
    is_DisableEvent(cam.unique_id, IS_SET_EVENT_STATUS_CHANGED);

    this->pCam_        = new HIDS;
    this->pCamInfo_    = new CamInfo;

    *this->pCam_ = (HIDS) cam.unique_id;
    HWND hWnd;

    if (is_InitCamera(pCam_, &hWnd) != IS_SUCCESS) {
        LOG_ERROR_S << "Could not open camera " << cam.unique_id << " (" <<
            cam.display_name << ")";
        this->close();
        return false;
    }

    // camera successfully opened
    LOG_INFO_S << "Camera " << cam.unique_id
            << " (" << cam.display_name << ")"
            << " successfully opened ";

    SENSORINFO sensorInfo;
    if (is_GetSensorInfo(*this->pCam_, &sensorInfo) != IS_SUCCESS) {
        LOG_ERROR_S << "Failed to fetch sensor information for camera "
                << cam.unique_id << " (" << cam.display_name << ")";
        this->close();
        return false;
    }

    // set the trigger mode to software, image acquired when calling is_FreezeVideo()
    setAttrib( enum_attrib::FrameStartTriggerModeToFixedRate);

    // initial grab mode set to stop
    this->act_grab_mode_ = Stop;

    return true;
}
void CamIds::setErrorReport(bool on) {

    if (on)
        is_SetErrorReport(*this->pCam_, IS_ENABLE_ERR_REP);
    else
        is_SetErrorReport(*this->pCam_, IS_DISABLE_ERR_REP);
}

int CamIds::getLastError(std::string& msg) {
    int err_code;
    char* err_msg;
    if (is_GetError(*this->pCam_, &err_code, &err_msg)) {
        msg = std::string(err_msg);
        return err_code;
    }
    return 0;
}

CaptureStatus CamIds::getCaptureStatus() {

    CaptureStatus cs;
    UEYE_CAPTURE_STATUS_INFO ucsi;

    is_CaptureStatus(*this->pCam_, IS_CAPTURE_STATUS_INFO_CMD_GET,
            &ucsi, sizeof(ucsi));

    cs.noDestMem = ucsi.adwCapStatusCnt_Detail[IS_CAP_STATUS_API_NO_DEST_MEM];
    cs.conversionFailed = ucsi.adwCapStatusCnt_Detail[IS_CAP_STATUS_API_CONVERSION_FAILED];
    cs.imageLocked = ucsi.adwCapStatusCnt_Detail[IS_CAP_STATUS_API_IMAGE_LOCKED];
    cs.outOfBuffers = ucsi.adwCapStatusCnt_Detail[IS_CAP_STATUS_DRV_OUT_OF_BUFFERS];
    cs.deviceNotReady = ucsi.adwCapStatusCnt_Detail[IS_CAP_STATUS_DRV_DEVICE_NOT_READY];
    cs.transferFailed = ucsi.adwCapStatusCnt_Detail[IS_CAP_STATUS_USB_TRANSFER_FAILED];
    cs.timeout = ucsi.adwCapStatusCnt_Detail[IS_CAP_STATUS_DEV_TIMEOUT];
    cs.bufferOverrun = ucsi.adwCapStatusCnt_Detail[IS_CAP_STATUS_ETH_BUFFER_OVERRUN];
    cs.missedImages = ucsi.adwCapStatusCnt_Detail[IS_CAP_STATUS_ETH_MISSED_IMAGES];
    cs.totalCount = ucsi.dwCapStatusCnt_Total;

    return cs;
}


bool CamIds::isOpen() const {
    if (this->pCam_ == NULL) {
        return false;
    }
    return true;
}

const CamInfo* CamIds::getCameraInfo() const {
    return pCamInfo_;
}

bool CamIds::close() {
    LOG_INFO_S << "Closing camera";

    // clear the queue, stop live video
    if (this->act_grab_mode_ == Continuously) {
        grabStopFromContinuousMode();
    }

    if (this->pFrameBuf_ != NULL)
        clearBuffers();

    if (this->pCamInfo_ != NULL) {
        delete this->pCamInfo_;
        this->pCamInfo_ = NULL;
    }

    if (this->pCam_ != NULL) {
        INT retVal = is_ExitCamera(*this->pCam_);

        delete this->pCam_;
        this->pCam_ = NULL;

        if (retVal != IS_SUCCESS) {
            return false;
        }
    }

    return true;
}

bool CamIds::grab(const GrabMode mode, const int buffer_len) {

    if (this->isOpen() == false) {
        LOG_ERROR_S << "Cannot grab, camera is not open!";
        return false;
    }

    if (this->act_grab_mode_ != Stop && mode != Stop) {
        if (this->act_grab_mode_ != mode) {
            LOG_ERROR_S << "Stop grabbing before switching the grab mode!";
            return false;
        }
        else {
            LOG_DEBUG_S << "Same grab mode requested!";
            return true;
        }
    }

    bool ret = true;

    switch(mode) {
    case Stop:
        if (this->act_grab_mode_ != Continuously) {
            LOG_WARN_S << "Can only stop after live video!";
            return false;
        }
        LOG_INFO_S << "Set grab mode to Stop";
        grabStopFromContinuousMode();
        break;
    case SingleFrame:
        LOG_WARN_S << "Not implemented yet!";
        break;
    case MultiFrame:
        throw std::runtime_error("MultiFrame not supported yet!");
        break;
    case Continuously:
        LOG_INFO_S << "Set grab mode to Continuously";
        ret = grabContinuousMode ( buffer_len );
        break;
    default:
        throw std::runtime_error("Grab mode not supported by camera!");
    }

    if ( ret ) {
        this->act_grab_mode_ = mode;
        return true;
    } else
        return false;
}

bool CamIds::isFrameAvailable() {

    switch (act_grab_mode_) {
    case Continuously:
        if (mGetEveryFrame)
            return true;
        else
            return isFrameAvailableContinuousMode();
    default:
        return false;
    }

}

bool CamIds::retrieveFrame(base::samples::frame::Frame& frame, const int timeout) {

    switch (this->act_grab_mode_) {
    case Stop:
        LOG_DEBUG_S << "Active mode is set to Stop! Cannot retrieve frame!";
        return false;
        break;
    case SingleFrame:
        LOG_WARN_S << "Not implemented yet!";
        return false;
        break;
    case MultiFrame:
        throw std::runtime_error("MultiFrame not supported yet!");
        break;
    case Continuously:
        if (mGetEveryFrame)
            return retrieveOldestNewFrameContinuousMode(frame, timeout);
        else
            return retrieveFrameContinuousMode(frame, timeout);
        break;
    default:
        throw std::runtime_error("Grab mode not supported by camera!");
    }
    return true;
}

const UEYE_IMAGE& CamIds::getFrameBuf ( char* pbuffer ) {

    if ( !pbuffer )
        throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) +
                ": zero pointer given");

    if ( !pFrameBuf_ )
        throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) +
                ": no frame buffers");

    for ( unsigned int i=0; i < nFrameBufLen_; i++ )
        if ( pFrameBuf_[i].pBuf == pbuffer ) return pFrameBuf_[i];

    throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) +
            ": invalid image pointer");
}

bool CamIds::allocBuffers( int buffer_cnt ) {
    INT retVal;

    if ( pFrameBuf_ ) {
        LOG_ERROR_S << "There is already some buffer thing.";
        return false;
    }

    this->pFrameBuf_ = new UEYE_IMAGE[buffer_cnt];
    this->nFrameBufLen_ = buffer_cnt;

    for (int i = 0; i < buffer_cnt; ++i) {

        retVal = is_AllocImageMem(
                *this->pCam_,
                this->image_size_.width,
                this->image_size_.height,
                this->image_color_depth_ * 8,
                &this->pFrameBuf_[i].pBuf,
                &this->pFrameBuf_[i].nImageID );

        if (retVal != IS_SUCCESS) {
            LOG_ERROR_S << "Unable to allocate image memory for camera "
                    << this->pCamInfo_->unique_id
                    << " (" << this->pCamInfo_->display_name << ")";
            this->close();
            return false;
        }

        this->pFrameBuf_[i].nImageSeqNum = i+1;
        this->pFrameBuf_[i].nBufferSize = this->image_size_.width *
            this->image_size_.height * this->image_color_depth_;

        is_AddToSequence(*this->pCam_,
                this->pFrameBuf_[i].pBuf, this->pFrameBuf_[i].nImageID);
    }
    return true;
}

void CamIds::clearBuffers() {

    if (pFrameBuf_) {

        for ( unsigned int i=0; i<nFrameBufLen_; i++ ) {
            is_FreeImageMem(*pCam_, pFrameBuf_[i].pBuf, pFrameBuf_[i].nImageID);
            pFrameBuf_[i].pBuf = 0;
        }

        delete[] pFrameBuf_;
        pFrameBuf_ = 0;
        nFrameBufLen_ = 0;
    }
}

/** Grab for the continous mode*/
bool CamIds::grabContinuousMode(const int buffer_len) {

    clearBuffers();

    if (!allocBuffers(buffer_len)) return false;
    // install event handling
    is_EnableEvent(*this->pCam_, IS_SET_EVENT_FRAME);

    // enable queueing mode
    is_InitImageQueue(*this->pCam_, 0);

    // start capturing video
    is_CaptureVideo(*this->pCam_, IS_DONT_WAIT); //TODO play around with wait values

    mLastFrameCount = 0;

    return true;
}


/** Retrieve a frame in the continous mode.
 *
 * This one gets the latest captured frame, so it will not */
bool CamIds::retrieveFrameContinuousMode( base::samples::frame::Frame& frame,
        const int timeout) {

        static base::Time tnow = base::Time::now();

        INT dummy;
        char *plast = NULL, *pdummy = NULL;
        int inum, iid;
        UEYEIMAGEINFO imgInfo;

        is_GetActSeqBuf ( *this->pCam_, &dummy, &pdummy, &plast );
        const UEYE_IMAGE& u_img = getFrameBuf(plast);
        inum = u_img.nImageSeqNum;
        iid  = u_img.nImageID;
        is_LockSeqBuf ( *pCam_, inum, plast );
        is_GetImageInfo(*this->pCam_, iid, &imgInfo, sizeof(imgInfo));

        frame.init( (uint16_t)imgInfo.dwImageWidth, (uint16_t)imgInfo.dwImageHeight,
                image_color_depth_, image_mode_ );


        frame.setImage(plast, frame.row_size * frame.size.height);

        frame.setStatus(STATUS_VALID);

        //generate base::Time object with givend camera device time synchronised to system time.
        frame.time = base::Time::fromTimeValues(imgInfo.TimestampSystem.wYear, imgInfo.TimestampSystem.wMonth, imgInfo.TimestampSystem.wDay,
                                               imgInfo.TimestampSystem.wHour, imgInfo.TimestampSystem.wMinute, imgInfo.TimestampSystem.wSecond,
                                               imgInfo.TimestampSystem.wMilliseconds, 0);

        frame.received_time = base::Time::now();
        double dt = (frame.received_time - tnow).toSeconds();
        tnow = frame.received_time;

        frame.attributes.clear();

        frame.setAttribute<uint64_t>("FrameCount", imgInfo.u64FrameNumber);
        int frame_diff = imgInfo.u64FrameNumber - mLastFrameCount;
        if ( frame_diff > 1 ) {
            LOG_WARN("lost frame(s): %3d; dt = %7.5f", imgInfo.u64FrameNumber - mLastFrameCount - 1, dt);
        }
        else if ( frame_diff < 1 ) {
            LOG_WARN_S << "frame increment is less than 1 - that should not happen: " <<
                imgInfo.u64FrameNumber << " - " << mLastFrameCount;
        }
        mLastFrameCount = imgInfo.u64FrameNumber;

        setFrameAttrExposure(frame);
        setFrameAttrPixelClock(frame);
        setFrameAttrGain(frame);

        is_UnlockSeqBuf(*this->pCam_, inum, plast);

        return true;
}

bool CamIds::setFrameAttrPixelClock(base::samples::frame::Frame &frame) {
    // get pixel clock frequency in MHz
    unsigned int pixel_clock = 0;
    if( is_PixelClock(*(this->pCam_), IS_PIXELCLOCK_CMD_GET, &pixel_clock, sizeof(pixel_clock)) == IS_SUCCESS ) {
        frame.setAttribute<int>("PixelClock_MHz", pixel_clock);
        return true;
    }
    else {
        LOG_WARN_S<<"could not access the current pixel clock value";
        return false;
    }
}

bool CamIds::setFrameAttrExposure(base::samples::frame::Frame &frame) {
    double exposure = 0;
    // get the current exposure time in ms
    if( is_Exposure(*this->pCam_, IS_EXPOSURE_CMD_GET_EXPOSURE, &exposure, sizeof(exposure)) == IS_SUCCESS ) {
        frame.setAttribute<double>("Exposure_ms", exposure);
        return true;
    }
    else {
        LOG_WARN_S<<"could not access the current exposure time";
        return false;
    }
}

bool CamIds::setFrameAttrGain(base::samples::frame::Frame &frame) {
    int gain = is_SetHardwareGain(*(this->pCam_), IS_GET_MASTER_GAIN, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER);
    frame.setAttribute<double>("Gain", gain);
}

/** A wrapper for is_WatiForNextImage to get all the messages out. */
bool CamIds::waitForNextImage(int timeout, char** ppcMem, int* p_img_id) {

    INT result = is_WaitForNextImage(*this->pCam_, timeout, ppcMem, p_img_id);
    if (result != IS_SUCCESS) {
        switch(result) {
            case IS_CANT_COMMUNICATE_WITH_DRIVER:
                LOG_ERROR_S << "Communication with the driver failed because no "
                    << "driver has been loaded.";
                break;
            case IS_CANT_OPEN_DEVICE:
                LOG_ERROR_S << "An attempt to initialize or select the camera "
                    << "failed (no camera connected or initialization error).";
                break;
            case IS_INVALID_CAMERA_HANDLE:
                LOG_ERROR_S << "Invalid camera handle.";
                break;
            case IS_INVALID_MEMORY_POINTER:
                LOG_ERROR_S << "Invalid pointer or invalid memory ID.";
                break;
            case IS_INVALID_PARAMETER:
                LOG_ERROR_S << "One of the submitted parameters is outside the "
                    << "valid range or is not supported for this sensor or is "
                    << "not available in this mode.";
                break;
            case IS_IO_REQUEST_FAILED:
                LOG_ERROR_S << "An IO request from the uEye driver failed. "
                    << "Possibly the versions of the ueye_api.dll (API) and the "
                    << "driver file (ueye_usb.sys or ueye_eth.sys) do not match.";
                break;
            case IS_NO_SUCCESS:
                LOG_ERROR_S << "General error.";
                break;
            case IS_TIMED_OUT:
                LOG_ERROR_S << "A timeout occurred. An image capturing process "
                    << "could not be terminated within the allowable period.";
                break;
            // hast the same value as another macro.
            //case IS_CAPTURE_STATUS:
            //    LOG_ERROR_S << "A transfer error occurred or no image memory was "
            //        << "available for saving.";
            //    break;
            default:
                LOG_ERROR_S << "Unknown error.";
        }
        return false;
    }
    return true;
}

/** Retireve a frame in the continous mode.
 *
 * This one gets the oldest not retrieved Frame. Furthermore it uses
 * an eventless apprach.*/
bool CamIds::retrieveOldestNewFrameContinuousMode( base::samples::frame::Frame& frame,
        const int timeout) {

    static base::Time tnow = base::Time::now();

    char* pcMem;
    INT img_id;

    if (!waitForNextImage(timeout, &pcMem, &img_id)) return false;

    UEYEIMAGEINFO imgInfo;
    is_GetImageInfo(*this->pCam_, img_id, &imgInfo, sizeof(imgInfo));

    frame.init( (uint16_t)imgInfo.dwImageWidth, (uint16_t)imgInfo.dwImageHeight,
            image_color_depth_, image_mode_ );


    frame.setImage(pcMem, frame.row_size * frame.size.height);

    frame.setStatus(STATUS_VALID);

    //generate base::Time object with givend camera device time synchronised to system time.
    frame.time = base::Time::fromTimeValues(imgInfo.TimestampSystem.wYear,
                                            imgInfo.TimestampSystem.wMonth,
                                            imgInfo.TimestampSystem.wDay,
                                            imgInfo.TimestampSystem.wHour,
                                            imgInfo.TimestampSystem.wMinute,
                                            imgInfo.TimestampSystem.wSecond,
                                            imgInfo.TimestampSystem.wMilliseconds,
                                            0);

    frame.received_time = base::Time::now();
    double dt = (frame.received_time - tnow).toSeconds();
    tnow = frame.received_time;

    frame.attributes.clear();

    frame.setAttribute<uint64_t>("FrameCount", imgInfo.u64FrameNumber);
    int frame_diff = imgInfo.u64FrameNumber - mLastFrameCount;
    if ( frame_diff > 1 ) {
        LOG_WARN("lost frame(s): %3d; dt = %7.5f", imgInfo.u64FrameNumber - mLastFrameCount - 1, dt);
    }
    else if ( frame_diff < 1 ) {
        LOG_WARN_S << "frame increment is less than 1 - that should not happen: " <<
            imgInfo.u64FrameNumber << " - " << mLastFrameCount;
    }
    mLastFrameCount = imgInfo.u64FrameNumber;

    setFrameAttrExposure(frame);
    setFrameAttrPixelClock(frame);
    setFrameAttrGain(frame);

    is_UnlockSeqBuf(*this->pCam_, IS_IGNORE_PARAMETER, pcMem);

    return true;
}


/** Checks weather a frame is available in continous mode */
bool CamIds::isFrameAvailableContinuousMode () {
    if ( is_WaitEvent(*this->pCam_, IS_SET_EVENT_FRAME, mEventTimeout) == IS_SUCCESS )
        return true;
    else
        return false;
}

bool CamIds::grabStopFromContinuousMode() {
        // stop live video
        if (is_StopLiveVideo(*this->pCam_, IS_WAIT) != IS_SUCCESS) {
            LOG_ERROR_S << "Unable to stop live capture for camera "
                    << this->pCamInfo_->unique_id
                    << " (" << this->pCamInfo_->display_name << ")";
            return false;
        }

        // disable events and exit the image queue
        is_ClearSequence(*this->pCam_);
        clearBuffers();
        is_DisableEvent(*this->pCam_, IS_SET_EVENT_FRAME);
        is_ExitImageQueue(*this->pCam_);

        return true;
}

//==============================================================================
bool CamIds::setFrameSettings(const base::samples::frame::frame_size_t size,
        const base::samples::frame::frame_mode_t mode,
        const uint8_t color_depth, // Bytes per Pixel
        const bool resize_frames)
{
    // no camera
    if (!this->pCam_) {
        return false;
    }

    if (this->act_grab_mode_ != Stop) {
        LOG_ERROR_S << "Cannot change camera setting during grabbing!";
        return false;
    }

    // get details about the camera sensor
    SENSORINFO sensorInfo;

    if (is_GetSensorInfo(*this->pCam_, &sensorInfo) != IS_SUCCESS) {
        LOG_ERROR_S << "Failed to fetch sensor information for camera "
                << this->pCamInfo_->unique_id
                << " (" << this->pCamInfo_->display_name << ")"
                << "while setting frame settings!";
        return false;
    }

    // check if the frame size is supported by the camera
    if (size.width > sensorInfo.nMaxWidth || size.height > sensorInfo.nMaxHeight) {
        LOG_ERROR_S << "Frame size not supported!";
        return false;
    }

    // get the area of interest dimensions
    IS_RECT rectAOI;

    int ret = is_AOI(*this->pCam_, IS_AOI_IMAGE_GET_AOI, (void*)&rectAOI,
            sizeof(rectAOI));

    if (ret != IS_SUCCESS) {
        std::stringstream ss;
        ss << std::string(BOOST_CURRENT_FUNCTION) << ": unable to get AOI."
            << " Returned " << ret;
        throw std::runtime_error(ss.str());
    }
    LOG_INFO("Current AOI is w=%4d h=%4d x=%4d y=%4d", rectAOI.s32Width,
            rectAOI.s32Height, rectAOI.s32X, rectAOI.s32Y);

    // set new AOI values
    rectAOI.s32Width    = size.width;
    rectAOI.s32Height   = size.height;

    LOG_INFO("Set AOI with w=%4d h=%4d x=%4d y=%4d", size.width, size.height, rectAOI.s32X,
            rectAOI.s32Y);
    // set the area of interest of the camera
    ret = is_AOI(*this->pCam_, IS_AOI_IMAGE_SET_AOI, (void*)&rectAOI,
            sizeof(rectAOI));

    if (ret != IS_SUCCESS) {
        std::stringstream ss;
        ss << std::string(BOOST_CURRENT_FUNCTION) << ": unable to set AOI."
            << " Returned " << ret;
        IS_SIZE_2D iss;
        if ( is_AOI(*this->pCam_, IS_AOI_IMAGE_GET_SIZE_INC, (void*)&iss,
                    sizeof(iss)) == IS_SUCCESS ) {
            ss << ". Sizeincr. is " << iss.s32Width << "/" << iss.s32Height;
        }
        throw std::runtime_error(ss.str());
    }

    // compute the channel count of the given mode
    int channelCount = Frame::getChannelCount(mode);
    if (channelCount <= 0) {
        LOG_ERROR_S << "Unsupported frame model for camera "
                << this->pCamInfo_->unique_id
                << " (" << this->pCamInfo_->display_name << ")"
                << "while setting frame settings!";
        return false;
    }

    // we require this later on
    int dataDepth = (color_depth * 8) / channelCount;

    // start from something
    INT selectedMode;

    switch (mode) {
    case MODE_BAYER:
    case MODE_BAYER_BGGR:
    case MODE_BAYER_GRBG:
    case MODE_BAYER_GBRG:
        LOG_ERROR_S << "Camera " << this->pCamInfo_->unique_id << " uses MODE_BAYER_RGGB.";
        return false;
        break;
    case MODE_BAYER_RGGB:
        if (dataDepth == 8) {
#if UEYE_VERSION_CODE >= UEYE_VERSION(4, 20, 0)
            selectedMode = IS_CM_SENSOR_RAW8;
#else
            selectedMode = IS_CM_BAYER_RG8;
#endif
        }
        else if (dataDepth == 12) {
#if UEYE_VERSION_CODE >= UEYE_VERSION(4, 20, 0)
            selectedMode = IS_CM_SENSOR_RAW12;
#else
            selectedMode = IS_CM_BAYER_RG12;
#endif
        }
        else if (dataDepth == 16) {
#if UEYE_VERSION_CODE >= UEYE_VERSION(4, 20, 0)
            selectedMode = IS_CM_SENSOR_RAW16;
#else
            selectedMode = IS_CM_BAYER_RG16;
#endif
        }
        else {
            LOG_ERROR_S << "Invalid color depth and color mode for camera "
                    << this->pCamInfo_->unique_id
                    << " (" << this->pCamInfo_->display_name << ")"
                    << "while setting frame settings!";
            return false;
        }
        break;
    case MODE_GRAYSCALE:
        if (dataDepth == 8) {
            selectedMode = IS_CM_MONO8;
        }
        else if (dataDepth == 12) {
            selectedMode = IS_CM_MONO12;
        }
        else if (dataDepth == 16) {
            selectedMode = IS_CM_MONO16;
        }
        else {
            LOG_ERROR_S << "** Invalid color depth and color mode for camera "
                    << this->pCamInfo_->unique_id
                    << " (" << this->pCamInfo_->display_name << ")"
                    << "while setting frame settings!";
            return false;
        }
        break;
    case MODE_RGB:
        if (dataDepth == 8) {
            selectedMode = IS_CM_RGB8_PACKED;
        }
        else {
            LOG_ERROR_S << "Invalid color depth and color mode for camera "
                    << this->pCamInfo_->unique_id
                    << " (" << this->pCamInfo_->display_name << ")"
                    << "while setting frame settings!";
            return false;
        }
        break;
    case MODE_BGR:
        if (dataDepth == 8) {
            selectedMode = IS_CM_BGR8_PACKED;
        }
        else {
            LOG_ERROR_S << "Invalid color depth and color mode for camera "
                    << this->pCamInfo_->unique_id
                    << " (" << this->pCamInfo_->display_name << ")"
                    << "while setting frame settings!";
            return false;
        }
        break;
    case MODE_RGB32:
        if (dataDepth == 8) {
            selectedMode = IS_CM_RGBA8_PACKED;
        }
        else {
            LOG_ERROR_S << "** Invalid color depth and color mode for camera "
                    << this->pCamInfo_->unique_id
                    << " (" << this->pCamInfo_->display_name << ")"
                    << "while setting frame settings!";
            return false;
        }
        break;
    case MODE_UYVY:
        selectedMode = IS_CM_UYVY_PACKED;
        break;
    default:
        LOG_ERROR_S << "Unknown color mode for camera "
                << this->pCamInfo_->unique_id
                << " (" << this->pCamInfo_->display_name << ")"
                << "while setting frame settings!" << std::endl;
        return false;
        break;
    }

    // now set the camera to the selected color mode
    if (is_SetColorMode(*this->pCam_, selectedMode) != IS_SUCCESS) {
        throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) + ": unable to set color mode");
    }

    // set the respective fields to the proper values
    this->image_size_           = size;
    this->image_mode_           = mode;
    this->image_color_depth_    = color_depth;

    return true;
}

//==============================================================================
bool CamIds::getFrameSettings(base::samples::frame::frame_size_t& size,
        base::samples::frame::frame_mode_t& mode,
        uint8_t& color_depth)
{
    IS_RECT rectAOI;

    int ret = is_AOI(*this->pCam_, IS_AOI_IMAGE_GET_AOI, (void*)&rectAOI,
            sizeof(rectAOI));

    if (ret != IS_SUCCESS) {
        std::stringstream ss;
        ss << std::string(BOOST_CURRENT_FUNCTION) << ": unable to get AOI."
            << " Returned " << ret;
        throw std::runtime_error(ss.str());
    }
    size.width = rectAOI.s32Width;
    size.height = rectAOI.s32Height;
    size = this->image_size_;
    color_depth = this->image_color_depth_;

    int color_mode = is_SetColorMode(*this->pCam_, IS_GET_COLOR_MODE);

    switch(color_mode) {
#if UEYE_VERSION_CODE >= UEYE_VERSION(4, 20, 0)
        case IS_CM_SENSOR_RAW8:
        case IS_CM_SENSOR_RAW12:
        case IS_CM_SENSOR_RAW16:
#else
        case IS_CM_BAYER_RG8:
        case IS_CM_BAYER_RG12:
        case IS_CM_BAYER_RG16:
#endif
            mode = MODE_BAYER_RGGB;
            break;
        case IS_CM_MONO8:
        case IS_CM_MONO12:
        case IS_CM_MONO16:
            mode = MODE_GRAYSCALE;
            color_depth = 1;
            break;
        case IS_CM_RGB8_PACKED:
            mode = MODE_RGB;
            break;
        case IS_CM_BGR8_PACKED:
            mode = MODE_BGR;
            break;
        case IS_CM_RGBA8_PACKED:
            mode = MODE_RGB32;
            break;
        case IS_CM_UYVY_PACKED:
            mode = MODE_UYVY;
            break;
        default:
            mode = MODE_UNDEFINED;
    }

    return true;
}

INT CamIds::BinningXFactorToMode(int factor) {
    switch (factor) {
    case 1: return IS_BINNING_DISABLE;
    case 2: return IS_BINNING_2X_HORIZONTAL;
    case 3: return IS_BINNING_3X_HORIZONTAL;
    case 4: return IS_BINNING_4X_HORIZONTAL;
    case 5: return IS_BINNING_5X_HORIZONTAL;
    case 6: return IS_BINNING_6X_HORIZONTAL;
    case 8: return IS_BINNING_8X_HORIZONTAL;
    case 16: return IS_BINNING_16X_HORIZONTAL;
    default:
        throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) +
                ": unsupported BinningX factor");
    }
}

INT CamIds::BinningYFactorToMode(int factor) {
    switch (factor) {
    case 1: return IS_BINNING_DISABLE;
    case 2: return IS_BINNING_2X_VERTICAL;
    case 3: return IS_BINNING_3X_VERTICAL;
    case 4: return IS_BINNING_4X_VERTICAL;
    case 5: return IS_BINNING_5X_VERTICAL;
    case 6: return IS_BINNING_6X_VERTICAL;
    case 8: return IS_BINNING_8X_VERTICAL;
    case 16: return IS_BINNING_16X_VERTICAL;
    default:
        throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) +
                ": unsupported BinningY factor");
    }
}

void CamIds::setBinningX(int factor) {

    INT mode = BinningXFactorToMode(factor);
    this->image_size_.width /= factor;

    int factor_y = is_SetBinning(*this->pCam_, IS_GET_BINNING_FACTOR_VERTICAL);
    mode |= BinningYFactorToMode(factor_y);

    if (IS_SUCCESS != is_SetBinning(*this->pCam_, mode)) {
        throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) +
                ": BinningX factor not supported by camera");
    }
    else {
        LOG_INFO_S<<"Set horizontal binning to "<<factor;
    }
}

void CamIds::setBinningY( int factor ) {

    INT mode = BinningYFactorToMode(factor);
    this->image_size_.height /= factor;

    int factor_x = is_SetBinning(*this->pCam_, IS_GET_BINNING_FACTOR_HORIZONTAL);
    mode |= BinningXFactorToMode(factor_x);

    if (IS_SUCCESS != is_SetBinning(*this->pCam_, mode)) {
        throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) +
                ": BinningY factor not supported by camera");
    }
    else {
        LOG_INFO_S<<"Set vertical binning to "<<factor;
    }

}


//==============================================================================
bool CamIds::setAttrib(const int_attrib::CamAttrib attrib, const int value) {
    // camera is not open
    if (!this->pCam_) {
        return false;
    }

    // some temporary variables
    double temp;
    double red, blue;

    // used to set area of interest for camera sensor
    IS_RECT rectAOI;

    int ret; // to get return value

    switch (attrib) {
        case int_attrib::ExposureValue:
            {
                // get the state of auto exposure
                double isAutoExp = 0;
                is_SetAutoParameter(*(this->pCam_), IS_GET_ENABLE_AUTO_SHUTTER, &isAutoExp, NULL);

                // set exposure time only if auto exposure is turned off
                if(!isAutoExp) {
                    // convert exposure time from SI unit microseconds to milliseconds
                    temp = (double) value / 1000.0;
                    // set exposure time in ms
                    if (IS_SUCCESS != is_Exposure(*this->pCam_, IS_EXPOSURE_CMD_SET_EXPOSURE,
                                (void*) &temp, sizeof(temp))) {
                        throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) +
                                ": unable to set exposure");
                    }
                    else {
                        LOG_INFO_S<<"Set exposure time to "<<temp<<" ms (target: "<<value<<" us)";
                    }
                }
                else {
                    LOG_WARN_S<<"Auto exposure enabled. Ignoring exposure time "<<(double)value<<" ms";
                }
            }
            break;
        case int_attrib::ExposureAutoMax:
            temp = (double) value;

            if (IS_SUCCESS != is_SetAutoParameter(*this->pCam_, IS_SET_AUTO_SHUTTER_MAX, &temp, NULL)) {
                throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) + ": unable to set max auto exposure");
            }
            break;
        case int_attrib::GainValue:
            {
                // get the current auto gain status
                double isautogain;
                if(is_SetAutoParameter(*(this->pCam_), IS_GET_ENABLE_AUTO_GAIN, &isautogain, NULL) != IS_SUCCESS) {
                    throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) + ": unable to retrieve auto gain status");
                }

                // prevent setting of gain value, when auto gain is enabled
                if(isautogain == false) {
                    if (IS_SUCCESS != is_SetHardwareGain(*this->pCam_, value, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER, IS_IGNORE_PARAMETER)) {
                        throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) + ": unable to set gain factor value");
                    }
                    else {
                        LOG_INFO_S<<"Set gain to: "<<value;
                    }
                }
                else {
                    LOG_WARN_S<<"Auto gain enabled. Ignoring gain value: "<<value;
                }
            }
            break;
        case int_attrib::GainAutoMax:
            if (IS_SUCCESS != is_SetAutoParameter(*this->pCam_, IS_SET_AUTO_GAIN_MAX, &temp, NULL)) {
                throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) + ": unable to set max auto gain");
            }
            break;
        case int_attrib::WhitebalValueRed: // XXX -50 to +50 offset
            // retrieve current values
            if (IS_SUCCESS != is_SetAutoParameter(*this->pCam_, IS_GET_AUTO_WB_OFFSET, &red, &blue)) {
                throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) + ": unable to get white balance offset for red channel");
            }

            // set the new value for red
            red = (double) value;
            if (IS_SUCCESS != is_SetAutoParameter(*this->pCam_, IS_SET_AUTO_WB_OFFSET, &red, &blue)) {
                throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) + ": unable to set white balance offset for red channel");
            }
            break;
        case int_attrib::WhitebalValueBlue: // XXX -50 to +50 offset
            // retrieve current values
            if (IS_SUCCESS != is_SetAutoParameter(*this->pCam_, IS_GET_AUTO_WB_OFFSET, &red, &blue)) {
                throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) + ": unable to get white balance offset for blue channel");
            }

            // set the new value for red
            blue = (double) value;
            if (IS_SUCCESS != is_SetAutoParameter(*this->pCam_, IS_SET_AUTO_WB_OFFSET, &red, &blue)) {
                throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) + ": unable to set white balance offset for blue channel");
            }
            break;
        case int_attrib::SaturationValue:
            if (IS_SUCCESS != is_Saturation(*this->pCam_, SATURATION_CMD_SET_VALUE, (void*) &value, sizeof(value))) {
                throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) + ": unable to set saturation value");
            }
            LOG_INFO("Set SaturationValue to %i",value);
            break;
        case int_attrib::PixelClock:
#if UEYE_VERSION_CODE >= UEYE_VERSION(4, 0, 0)
            if (IS_SUCCESS != is_PixelClock(*this->pCam_, IS_PIXELCLOCK_CMD_SET,
                        (void*)&value, sizeof(value))) {
#else
            if (IS_SUCCESS != is_SetPixelClock(*this->pCam_, value)) {
#endif
                throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) + ": unable to set pixel clock");
            }
            LOG_INFO("Set PixelClock to %i",value);
            break;
        case int_attrib::BinningX:
            setBinningX(value);
            break;
        case int_attrib::BinningY:
            setBinningY(value);
            break;
        // X-Offset
        case int_attrib::RegionX:
            ret = is_AOI(*this->pCam_, IS_AOI_IMAGE_GET_AOI, (void*)&rectAOI,
                    sizeof(rectAOI));

            if ( ret == IS_SUCCESS ) {
                rectAOI.s32X = value;
                UINT nAbsPos;
                ret = is_AOI(*this->pCam_, IS_AOI_IMAGE_GET_POS_X_ABS, (void*)&nAbsPos,
                        sizeof(nAbsPos));

                if ( ret == IS_SUCCESS && nAbsPos == IS_AOI_IMAGE_POS_ABSOLUTE )
                    rectAOI.s32X |= IS_AOI_IMAGE_POS_ABSOLUTE;

                ret = is_AOI(*this->pCam_, IS_AOI_IMAGE_SET_AOI,
                    (void*)&rectAOI, sizeof(rectAOI));

                if ( ret != IS_SUCCESS) {
                    std::stringstream ss;
                    ss << std::string(BOOST_CURRENT_FUNCTION) << ": unable to set AOI."
                        << " Returned " << ret;
                    throw std::runtime_error(ss.str());
                }
            }
            break;
        // Y-Offset
        case int_attrib::RegionY:
            ret = is_AOI(*this->pCam_, IS_AOI_IMAGE_GET_AOI, (void*)&rectAOI,
                    sizeof(rectAOI));

            if ( ret == IS_SUCCESS ) {
                rectAOI.s32Y = value;
                UINT nAbsPos;
                ret = is_AOI(*this->pCam_, IS_AOI_IMAGE_GET_POS_Y_ABS, (void*)&nAbsPos,
                        sizeof(nAbsPos));

                if ( ret == IS_SUCCESS && nAbsPos == IS_AOI_IMAGE_POS_ABSOLUTE )
                    rectAOI.s32Y |= IS_AOI_IMAGE_POS_ABSOLUTE;

                ret = is_AOI(*this->pCam_, IS_AOI_IMAGE_SET_AOI,
                    (void*)&rectAOI, sizeof(rectAOI));

                if ( ret != IS_SUCCESS) {
                    std::stringstream ss;
                    ss << std::string(BOOST_CURRENT_FUNCTION) << ": unable to set AOI."
                        << " Returned " << ret;
                    throw std::runtime_error(ss.str());
                }
            }
            break;
        default:
            throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) + ": unknown attribute");
            break;
    }
    return true;
}

//==============================================================================
bool CamIds::setAttrib(const double_attrib::CamAttrib attrib, const double value) {
    // camera is not open
    if (!this->pCam_) {
        return false;
    }

    switch (attrib) {
        case double_attrib::FrameRate:
            double newFrameRate;
            if (IS_SUCCESS != is_SetFrameRate(*this->pCam_, value, &newFrameRate)) {
                throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) + ": unable to set frame rate");
            }
            LOG_INFO("Set framerate to %5.1f", value);
            break;
        default:
            throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) + ": unknown attribute");
            break;
    }

    return true;
}

//==============================================================================
bool CamIds::setAttrib(const str_attrib::CamAttrib attrib, const std::string value) {
    // camera is not open
    if (!this->pCam_) {
        return false;
    }

    switch (attrib) {
    default:
        throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) + ": unknown attribute");
        break;
    }

    return true;
}

//==============================================================================
bool CamIds::setAttrib(const enum_attrib::CamAttrib attrib) {
    // camera is not open
    if (!this->pCam_) {
        return false;
    }

    // temporary variables
    double temp;

    // TODO turn on and off all attributes
    switch (attrib) {
    case enum_attrib::MirrorXToOn:
        if (IS_SUCCESS != is_SetRopEffect(*this->pCam_, IS_SET_ROP_MIRROR_LEFTRIGHT, 1, 0)) {
            throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) + ": error while enabling mirrorX");
        }
        LOG_INFO_S << "Set horizontal mirror to on.";
        break;
    case enum_attrib::MirrorXToOff:
        if (IS_SUCCESS != is_SetRopEffect(*this->pCam_, IS_SET_ROP_MIRROR_LEFTRIGHT, 0, 0)) {
            throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) + ": error while disabling mirrorX");
        }
        LOG_INFO_S << "Set horizontal mirror to off.";
        break;
    case enum_attrib::MirrorYToOn:
        if (IS_SUCCESS != is_SetRopEffect(*this->pCam_, IS_SET_ROP_MIRROR_UPDOWN, 1, 0)) {
            throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) + ": error while enabling mirrorY");
        }
        LOG_INFO_S << "Set vertical mirror to on.";
        break;
    case enum_attrib::MirrorYToOff:
        if (IS_SUCCESS != is_SetRopEffect(*this->pCam_, IS_SET_ROP_MIRROR_UPDOWN, 0, 0)) {
            throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) + ": error while disabling mirrorY");
        }
        LOG_INFO_S << "Set vertical mirror to off.";
        break;
    case enum_attrib::FrameStartTriggerModeToSoftware:
        if (is_SetExternalTrigger(*this->pCam_, IS_SET_TRIGGER_SOFTWARE) != IS_SUCCESS) {
            throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) + ": unabel to set trigger to software");
        }
        LOG_INFO_S << "Set trigger mode to software.";
        break;
    case enum_attrib::FrameStartTriggerModeToFixedRate:
        // This is off in the context off ids camera.
        if (is_SetExternalTrigger(*this->pCam_, IS_SET_TRIGGER_OFF) != IS_SUCCESS) {
            throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) + ": unabel to set trigger to off");
        }
        LOG_INFO_S << "Set trigger mode to off.";
        break;
    case enum_attrib::ExposureModeToAuto:
        // turn on auto
        temp = 1;

        // turn on auto exposure
        if (IS_SUCCESS != is_SetAutoParameter(*this->pCam_, IS_SET_ENABLE_AUTO_SHUTTER, &temp, NULL)) {
            throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) + ": unable to set exposure to auto");
        }
        LOG_INFO_S << "Set exposure to auto.";
        { double exp, dummy;
            if ( is_SetAutoParameter(*this->pCam_, IS_GET_AUTO_SHUTTER_MAX, &exp, &dummy) == IS_SUCCESS ) {
                LOG_INFO_S << "Exposure max is " << exp;
            }
        }
        break;
    case enum_attrib::ExposureModeToManual:
        // turn off auto
        temp = 0;

        // turn on auto exposure
        if (IS_SUCCESS != is_SetAutoParameter(*this->pCam_, IS_SET_ENABLE_AUTO_SHUTTER, &temp, NULL)) {
            throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) + ": unable to set exposure to manual");
        }
        LOG_INFO_S << "Set exposure to manual.";
        break;
    case enum_attrib::GainModeToAuto:
        {
            double autogain = true;
            if(is_SetAutoParameter(*(this->pCam_), IS_SET_ENABLE_AUTO_GAIN, &autogain, NULL) != IS_SUCCESS) {
                LOG_ERROR_S<<"Unable to enable auto gain";
            }
            else {
                LOG_INFO_S<<"Auto gain enabled";
            }
        }
        break;
    case enum_attrib::GainModeToManual:
        {
            double autogain = false;
            if(is_SetAutoParameter(*(this->pCam_), IS_SET_ENABLE_AUTO_GAIN, &autogain, NULL) != IS_SUCCESS) {
                LOG_ERROR_S<<"Unable to disable auto gain";
            }
            else {
                LOG_INFO_S<<"Auto gain disabled";
            }
        }
        break;
    default:
        throw std::runtime_error(std::string(BOOST_CURRENT_FUNCTION) + ": unknown attribute");
        break;
    }

    return true;
}

bool CamIds::setGainBoost(const bool boost) {
    if(!this->pCam_) {
        return false;
    }

    if(boost && (is_SetGainBoost(*(this->pCam_), IS_GET_SUPPORTED_GAINBOOST) != IS_SET_GAINBOOST_ON) ) {
        // unsupported
        LOG_WARN_S<<"Gain boost not supported!";
        return false;
    }

    // IS_SET_GAINBOOST_ON   0x0001
    // IS_SET_GAINBOOST_OFF  0x0000
    if(is_SetGainBoost(*(this->pCam_), boost) != IS_SUCCESS) {
        LOG_WARN_S<<"Gain boost could not be set to: "<<boost;
        return false;
    }

    return true;
}

bool CamIds::setGainRed(const unsigned int gain) {
    return setGainChannel(gain, 0);
}

bool CamIds::setGainGreen(const unsigned int gain) {
    return setGainChannel(gain, 1);
}

bool CamIds::setGainBlue(const unsigned int gain) {
    return setGainChannel(gain, 2);
}

bool CamIds::setGainChannel(const unsigned int gain, const unsigned int channel) {

    unsigned int chg[3]; // gain per channel

    std::fill_n(chg, 3, IS_IGNORE_PARAMETER);
    chg[channel] = gain;

    if(is_SetHardwareGain(*(this->pCam_), IS_IGNORE_PARAMETER, chg[0], chg[1], chg[2]) != IS_SUCCESS) {
        LOG_WARN_S<<"Could not set gain "<<gain<<" for channel "<<channel;
        return false;
    }
    else {
        LOG_INFO_S<<"Set gain on channel "<<channel<<" to "<<gain;
    }

    return true;
}

//==============================================================================
bool CamIds::isAttribAvail(const int_attrib::CamAttrib attrib) {
    switch (attrib) {
    case int_attrib::ExposureValue:
    case int_attrib::ExposureAutoMax:
    case int_attrib::GainValue:
    case int_attrib::GainAutoMax:
    case int_attrib::WhitebalValueRed:
    case int_attrib::WhitebalValueBlue:
    case int_attrib::SaturationValue:
    case int_attrib::PixelClock:
    case int_attrib::BinningX:
    case int_attrib::BinningY:
    case int_attrib::RegionX:
    case int_attrib::RegionY:
        return true;
        break;
    default:
        return false;
        break;
    }

    return false;
}

//==============================================================================
bool CamIds::isAttribAvail(const double_attrib::CamAttrib attrib) {
    switch (attrib) {
    case double_attrib::FrameRate:
        return true;
        break;
    default:
        return false;
        break;
    }
    return false;
}

//==============================================================================
bool CamIds::isAttribAvail(const str_attrib::CamAttrib attrib) {
    return false;
}

//==============================================================================
bool CamIds::isAttribAvail(const enum_attrib::CamAttrib attrib) {
    switch (attrib) {
    case enum_attrib::MirrorXToOn:
    case enum_attrib::MirrorXToOff:
    case enum_attrib::MirrorYToOn:
    case enum_attrib::MirrorYToOff:
    case enum_attrib::FrameStartTriggerModeToSoftware:
    case enum_attrib::FrameStartTriggerModeToFixedRate:
    case enum_attrib::ExposureModeToAuto:
    case enum_attrib::ExposureModeToManual:
    case enum_attrib::GainModeToAuto:
    case enum_attrib::GainModeToManual:
        return true;
        break;
    default:
        return false;
        break;
    }
    return false;
}
//==============================================================================

} // end of camera namespace
