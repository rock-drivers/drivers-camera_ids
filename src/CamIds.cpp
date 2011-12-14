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
    camInfo.reachable         = true;   // assume reachable initially,
                                        // change later in code
    // temporary uEye camera info to fetch more info on current cam
    CAMINFO moreCamInfo;

    if (is_GetCameraInfo(*camHandle, &moreCamInfo) == IS_SUCCESS) {
        //========
        // USB
        //========
        if (moreCamInfo.Type == IS_CAMERA_TYPE_UEYE_USB
                || moreCamInfo.Type == IS_CAMERA_TYPE_UEYE_USB_LE
                || moreCamInfo.Type == IS_CAMERA_TYPE_UEYE_USB_SE
                || moreCamInfo.Type == IS_CAMERA_TYPE_UEYE_USB_ME
                || moreCamInfo.Type == IS_CAMERA_TYPE_UEYE_USB_RE)
        {
            // mark the appropriate interface
            camInfo.interface_type = InterfaceUSB;
        }

        //==========
        // Ethernet
        //==========
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
                    == IS_SUCCESS)
            {
                //-------------------------------
                // set up the configuration mode
                //-------------------------------
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

                //--------------------------------
                // set up the current ip settings
                //--------------------------------
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

                //-----------------------------------
                // set up the persistent ip settings
                //-----------------------------------
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

                //----------------------------------------------
                // is the camera on the same subnet as the host
                //----------------------------------------------
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
                }
                else {
                    camInfo.reachable = false;
                }
            }
            else {
                // unknown interface camera attached
                std::cerr << "** Ethernet camera not found: "
                        << "dwCameraID(" << uEyeCamInfo->dwCameraID << "), "
                        << "dwDeviceID(" << uEyeCamInfo->dwDeviceID << ")"
                        <<std::endl;
            }
        }
        //===================
        // Unknown interface
        //===================
        else {
            // set the interface type to unknown
            camInfo.interface_type = InterfaceUnknown;
            std::cerr << "** Unknown camera type: "
                    << "dwCameraID(" << uEyeCamInfo->dwCameraID << "), "
                    << "dwDeviceID(" << uEyeCamInfo->dwDeviceID << ")"
                    << std::endl;
        }
    }
    else {
        std::cerr << "** Camera "
                << "dwCameraID(" << uEyeCamInfo->dwCameraID << "), "
                << "dwDeviceID(" << uEyeCamInfo->dwDeviceID << ")"
                << " not found, or device is already in use!"
                << std::endl;
    }

    return camInfo;
}

//==============================================================================
CamIds::CamIds() {
    this->hCam      = NULL;
    this->hCamInfo  = NULL;
}

//==============================================================================
CamIds::CamIds(const CamIds& other) {
    this->hCam      = other.hCam;
    this->hCamInfo  = other.hCamInfo;
}

//==============================================================================
CamIds::~CamIds() {
    std::cout << "== Destructing camera" << std::endl;

    // release all memory
    if (this->hCamInfo != NULL) {
        delete hCamInfo;
    }

    if (this->hCam != NULL) {
        // also release the camera
        INT retVal = is_ExitCamera(*hCam);

        delete hCam;
    }
}

//==============================================================================
int CamIds::listCameras(std::vector<CamInfo> &cam_infos) const {
    // how many cameras do we have?
    INT numCam = countCameras();

    // place-holder for the camera list
    UEYE_CAMERA_LIST* cameraList;
    cameraList = (UEYE_CAMERA_LIST*) new BYTE [sizeof (DWORD) + numCam * sizeof (UEYE_CAMERA_INFO)];

    // set the number of cameras inside the struct
    cameraList->dwCount = (ULONG) numCam;

    // proceed if we have some cameras
    if (cameraList->dwCount >= 1) {

        // get the camera list
        if (is_GetCameraList(cameraList) == IS_SUCCESS) {

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
                            << "dwDeviceID(" << uEyeCamInfo->dwDeviceID << ")"
                            << std::endl;
                }

                // to be pushed back into the camera list vector
                CamInfo camInfo;
                camInfo = fillCamInfo(&cameraList->uci[iCamera], &camHandle);

                // add the new camera to the list
                cam_infos.push_back(camInfo);

                // release camera
                is_ExitCamera(camHandle);
            }
        }
        else {
            std::cerr << "** Unable to retrieve camera list!" << std::endl;
        }
    }
    else {
        std::cerr << "** No cameras found!" << std::endl;
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

    if (retVal == IS_NO_SUCCESS) {
        std::cerr << "** Unable to retrieve number of cameras!"
                << std::endl;
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

    //##########################################################################
    // WARNING temporary fix to open cameras after calling get list
    //##########################################################################
    sleep(4);

    //--------------------------------------------------------------------------
    // cannot open a second time
    //--------------------------------------------------------------------------
    if (isOpen()) {
        std::cerr << "** Camera already open for current object!"
                << std::endl;
        return true;
    }

    //--------------------------------------------------------------------------
    // prepare to initialize camera
    //--------------------------------------------------------------------------
    // allocate memory for camera handle and camera info
    hCam        = new HIDS;
    hCamInfo    = new CamInfo;

    *hCam = (HIDS) cam.unique_id;
    HWND hWnd;

    // attempt to open
    INT nRet = is_InitCamera(hCam, &hWnd);

    //--------------------------------------------------------------------------
    // camera initialized
    //--------------------------------------------------------------------------
    if (nRet == IS_SUCCESS) {
        std::cout << "== Camera " << cam.unique_id
                << " (" << cam.display_name << ")"
                << " successfully opened " << std::endl;

        //----------------------------------------------------------------------
        // fetch camera information
        //----------------------------------------------------------------------
        INT numCam = countCameras();

        // place-holder for the camera list
        UEYE_CAMERA_LIST* cameraList;
        cameraList = (UEYE_CAMERA_LIST*) new BYTE [sizeof (DWORD) + numCam * sizeof (UEYE_CAMERA_INFO)];

        // set the number of cameras inside the struct
        cameraList->dwCount = (ULONG) numCam;

        // proceed if we have some cameras
        if (cameraList->dwCount >= 1) {

            // get the camera list
            if (is_GetCameraList(cameraList) == IS_SUCCESS) {

                // loop over the cameras and add them to the list
                for (int iCamera = 0; iCamera < (int)cameraList->dwCount; ++iCamera) {

                    // handle for the current camera
                    UEYE_CAMERA_INFO *uEyeCamInfo = &cameraList->uci[iCamera];

                    // search for camera by unique id
                    // WARNING set the camera ids in the uEye camera manager
                    if(uEyeCamInfo->dwCameraID == cam.unique_id) {
                        // fill in the camera information
                        *hCamInfo = fillCamInfo(uEyeCamInfo, hCam);
                        break;
                    }
                }
            }
            else {
                std::cerr << "** Unable to retrieve camera list while opening camera!"
                        << std::endl;
                close();
                return false;
            }
        }
        else {
            std::cout << "** No cameras found! Please check if properly attached!"
                    << std::endl;
            close();
            return false;
        }

        //----------------------------------------------------------------------
        // fetch sensor information
        //----------------------------------------------------------------------
        SENSORINFO sensorInfo;

        if (is_GetSensorInfo(*hCam, &sensorInfo) != IS_SUCCESS) {
            std::cerr << "** Failed to fetch sensor information for camera "
                    << cam.unique_id << " (" << cam.display_name << ")"
                    << std::endl;
            close();
            return false;
        }

        // set image size to the camera maximum
        this->image_size_.height    = (uint16_t) sensorInfo.nMaxHeight;
        this->image_size_.width     = (uint16_t) sensorInfo.nMaxWidth;

        std::cout << "== Image size set to "
                << this->image_size_.width << " x " << this->image_size_.height
                << std::endl;

        // color depth in bytes, set default RGB32
        if (is_SetColorMode(*hCam, IS_CM_RGBA8_PACKED) != IS_SUCCESS) {
            std::cerr << "** Unable to set color mode for camera "
                    << cam.unique_id << " (" << cam.display_name << ")"
                    << std::endl;
            close();
            return false;
        }

        this->image_color_depth_  = 8;
        this->image_mode_         = MODE_RGB32;

        // initial grab mode set to stop
        this->act_grab_mode_ = Stop;

        // release memory
        delete[] cameraList;
    }
    //--------------------------------------------------------------------------
    // failed to initialize
    //--------------------------------------------------------------------------
    else {
        std::cerr << "** Could not open camera " << cam.unique_id
                    << " (" << cam.display_name << ")" << std::endl;
        close();
        return false;
    }

    return true;
}

//==============================================================================
bool CamIds::isOpen() const {
    if (this->hCam == NULL) {
        return false;
    }
    else {
        return true;
    }
}

//==============================================================================
const CamInfo* CamIds::getCameraInfo() const {
    return hCamInfo;
}

//==============================================================================
bool CamIds::close() {
    std::cout << "== Closing camera" << std::endl;

    // release all memory
    if (this->hCamInfo != NULL) {
        delete hCamInfo;

        // make sure to mark as NULL
        hCamInfo = NULL;
    }

    if (this->hCam != NULL) {
        // also release the camera
        INT retVal = is_ExitCamera(*hCam);

        delete hCam;

        // make sure to mark as NULL
        hCam = NULL;

        if (retVal != IS_SUCCESS) {
            return false;
        }
    }

    return true;
}

//==============================================================================
bool CamIds::grab(const GrabMode mode, const int buffer_len) {

    return true;
}

//==============================================================================
bool CamIds::retrieveFrame(base::samples::frame::Frame& frame, const int timeout) {
    return true;
}

//==============================================================================
bool CamIds::isFrameAvailable() {

    return true;
}

//==============================================================================
bool CamIds::setFrameSettings(const base::samples::frame::frame_size_t size,
        const base::samples::frame::frame_mode_t mode,
        const uint8_t color_depth,
        const bool resize_frames)
{

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

} // end of camera namespace
