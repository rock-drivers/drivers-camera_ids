/**
 * Implementation for the IDS camera driver.
 * @file CamIds.cpp
 * @author Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
 * @date Thu Dec  8 13:25:02 CET 2011
 */

#include "CamIds.h"
#include <iostream>

using namespace base::samples::frame;

namespace camera {
//==============================================================================
CamIds::CamIds() {
    this->hCam = NULL;
}

//==============================================================================
CamIds::CamIds(const CamIds& other) {
    this->hCam = other.hCam;
}

//==============================================================================
CamIds::~CamIds() {
    if (this->hCam != NULL) {
        is_ExitCamera(*hCam);
        delete hCam;
    }
}

//==============================================================================
int CamIds::listCameras(std::vector<CamInfo> &cam_infos) const {
    // retrieve the number of cameras
    INT numberOfCameras = countCameras();

    if (numberOfCameras >= 1) {
        // place-holder for the camera list
        UEYE_CAMERA_LIST cameraList;

        // set the number of cameras inside the struct
        cameraList.dwCount = (ULONG) numberOfCameras;

        // get the camera list
        if (is_GetCameraList(&cameraList) == IS_SUCCESS) {

            // loop over the cameras and add them to the list
            for (int iCamera = 0; iCamera < (int)cameraList.dwCount; ++iCamera) {

                // handle for the current camera
                UEYE_CAMERA_INFO *uEyeCamInfo = &cameraList.uci[iCamera];

//                std::cout << "== Camera number " << iCamera + 1           << std::endl
//                        << "\t** Model: "     << uEyeCamInfo->Model         << std::endl
//                        << "\t** SerNo: "       << uEyeCamInfo->SerNo       << std::endl
//                        << "\t** dwCameraID: "  << uEyeCamInfo->dwCameraID  << std::endl
//                        << "\t** dwDeviceID: "  << uEyeCamInfo->dwDeviceID  << std::endl
//                        << "\t** dwInUse: "     << uEyeCamInfo->dwInUse     << std::endl
//                        << "\t** dwSensorID: "  << uEyeCamInfo->dwSensorID  << std::endl
//                        << "\t** dwStatus: "    << uEyeCamInfo->dwStatus    << std::endl;

                // to be pushed back into the camera list vector
                CamInfo camInfo;

                // fill in the camera information for each one
                camInfo.unique_id         = uEyeCamInfo->dwCameraID;
                camInfo.serial_string     = std::string(uEyeCamInfo->SerNo);
                camInfo.display_name      = std::string(uEyeCamInfo->Model);
                camInfo.reachable         = true;
                camInfo.device            = "unknown";

                // window handler for init camera
                HIDS camHandle = uEyeCamInfo->dwCameraID;
                HWND hWnd;

                // open the camera temporarilly to retrieve the data
                is_InitCamera(&camHandle, &hWnd);

                // temporary uEye camera info to fetch more info on current cam
                CAMINFO moreCamInfo;

                if (is_GetCameraInfo(camInfo.unique_id, &moreCamInfo) == IS_SUCCESS) {
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

//                        std::cout << "\t** Date: " << moreCamInfo.Date            << std::endl
//                                << "\t** ID: " << moreCamInfo.ID                  << std::endl
//                                << "\t** Select: " << (int)moreCamInfo.Select     << std::endl
//                                << "\t** Type: " << moreCamInfo.Type << " USB"    << std::endl
//                                << "\t** Version: " << moreCamInfo.Version        << std::endl;

                        // TODO camInfo.device = ?
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

                        if (is_GetEthDeviceInfo(uEyeCamInfo->dwCameraID, &ethDeviceInfo, sizeof(UEYE_ETH_DEVICE_INFO))
                                == IS_SUCCESS)
                        {
                            //###############################
                            // set up the configuration mode
                            //###############################
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

                            //################################
                            // set up the current ip settings
                            //################################
                            // current ip address
                            camInfo.ip_settings.current_ip_address =
                                    ethDeviceInfo.infoDevHeartbeat.ipcfgCurrentIpCfg.ipAddress.by.by1 << 24
                                    | ethDeviceInfo.infoDevHeartbeat.ipcfgCurrentIpCfg.ipAddress.by.by2 << 16
                                    | ethDeviceInfo.infoDevHeartbeat.ipcfgCurrentIpCfg.ipAddress.by.by3 << 8
                                    | ethDeviceInfo.infoDevHeartbeat.ipcfgCurrentIpCfg.ipAddress.by.by4 << 0;

                            // current subnet mask
                            camInfo.ip_settings.current_ip_subnet =
                                    ethDeviceInfo.infoDevHeartbeat.ipcfgCurrentIpCfg.ipSubnetmask.by.by1 << 24
                                    | ethDeviceInfo.infoDevHeartbeat.ipcfgCurrentIpCfg.ipSubnetmask.by.by2 << 16
                                    | ethDeviceInfo.infoDevHeartbeat.ipcfgCurrentIpCfg.ipSubnetmask.by.by3 << 8
                                    | ethDeviceInfo.infoDevHeartbeat.ipcfgCurrentIpCfg.ipSubnetmask.by.by4 << 0;

                            //###################################
                            // set up the persistent ip settings
                            //###################################
                            // persistent ip address
                            // TODO this is a typo in the ROCK header
                            camInfo.ip_settings.persisten_ip_addr =
                                    ethDeviceInfo.infoDevHeartbeat.ipcfgPersistentIpCfg.ipAddress.by.by1 << 24
                                    | ethDeviceInfo.infoDevHeartbeat.ipcfgPersistentIpCfg.ipAddress.by.by2 << 16
                                    | ethDeviceInfo.infoDevHeartbeat.ipcfgPersistentIpCfg.ipAddress.by.by3 << 8
                                    | ethDeviceInfo.infoDevHeartbeat.ipcfgPersistentIpCfg.ipAddress.by.by4 << 0;

                            // setup the persistent subnet mask
                            camInfo.ip_settings.persistent_ip_subnet =
                                    ethDeviceInfo.infoDevHeartbeat.ipcfgPersistentIpCfg.ipSubnetmask.by.by1 << 24
                                    | ethDeviceInfo.infoDevHeartbeat.ipcfgPersistentIpCfg.ipSubnetmask.by.by2 << 16
                                    | ethDeviceInfo.infoDevHeartbeat.ipcfgPersistentIpCfg.ipSubnetmask.by.by3 << 8
                                    | ethDeviceInfo.infoDevHeartbeat.ipcfgPersistentIpCfg.ipSubnetmask.by.by4 << 0;

                            //##############################################
                            // is the camera on the same subnet as the host
                            //##############################################
                            // TODO then tune up reachable

                        }
                        else {
                            // unknown interface camera attached
                            camInfo.interface_type = InterfaceUnknown;
                            std::cerr << "== Ethernet camera not found: "
                                    << uEyeCamInfo->dwCameraID
                                    << std::endl;
                        }
                    }
                    //===================
                    // Unknown interface
                    //===================
                    else {
                        // set the interface type to unknown
                        camInfo.interface_type = InterfaceUnknown;
                        std::cerr << "== Unknown camera type: "
                                << uEyeCamInfo->dwCameraID
                                << std::endl;
                    }
                }
                else {
                    std::cerr << "== Camera "
                            << uEyeCamInfo->dwCameraID
                            << " not found, or device is already in use!"
                            << std::endl;
                }

                // add the new camera to the list
                cam_infos.push_back(camInfo);

                // release camera
                is_ExitCamera(camHandle);
                std::cout << std::endl;
            }
        }
    }
    else {
        std::cout << "== No camera found!" << std::endl;
    }

    // must return number of listed cameras
    return numberOfCameras;
}

//==============================================================================
int CamIds::countCameras() const {
    INT numberOfCameras = 0;
    INT retVal = is_GetNumberOfCameras(&numberOfCameras);

    if (retVal == IS_NO_SUCCESS) {
        std::cerr << "Unable to retrieve number of cameras!"
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

    // params for init camera
    hCam = new HIDS;
    *hCam = (HIDS)cam.unique_id;
    HWND hWnd;

    INT nRet = is_InitCamera(hCam, &hWnd);

    if (nRet == IS_SUCCESS) {
        std::cout << "== Camera " << cam.unique_id
                << " (" << cam.display_name << ")"
                << " successfully opened " << std::endl;

        this->act_grab_mode_ = Stop;
        this->

        return true;
    }
    else {
        std::cout << "== Could not open camera " << cam.unique_id
                    << " (" << cam.display_name << ")" << std::endl;
        return false;
    }
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
} // end of camera namespace
