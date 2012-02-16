/**
  * Definition of the IDS camera driver.
  * @file CamIds.h
  * @author Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
  * @date Thu Dec 8 13:25:02 CET 2011
  */

#ifndef _CAMIDS_H
#define _CAMIDS_H


#include <camera_interface/CamInterface.h>  
#include <camera_interface/CamInfoUtils.h>                 

#include <base/samples/frame.h>

#include <ueye.h>                           // API for the IDS cameras

// a single ueye frame
typedef struct _UEYE_IMAGE {
    char *pBuf;                 // memory location of the frame
    INT nImageID;               // id of the frame
    INT nImageSeqNum;           // frame sequence number
    INT nBufferSize;            // the buffer size in bytes
} UEYE_IMAGE;

namespace camera {
/**
 * The ROCK camera interface class for IDS cameras.
 */
class CamIds : public CamInterface {
private:
    /**
     * Camera handle for the current open IDS camera.
     */
    HIDS *pCam_;

    /**
     * Camera information storage.
     */
    CamInfo *pCamInfo_;

    /**
     * This is where we keep the frames.
     */
    UEYE_IMAGE *pFrameBuf_;

    /**
     * Number of frames in the buffer.
     */
    unsigned int nFrameBufLen_;

    /**
     * Used when checking if new frames have arrived.
     */
    int nSeqCount_;

    /**
     * Retrieves camera information for a single camera.
     * @warning camera must be initialized when this method is used
     * @param uEyeCamInfo the ueye camera information to be extracted
     * @param camHandle a pointer to the camera handler that is should be initialized
     * @return a filled CamInfo struct
     */
    CamInfo fillCamInfo(UEYE_CAMERA_INFO *uEyeCamInfo, HIDS *camHandle) const;

public:
    //======================================================================
    // Inherited methods.
    //======================================================================

    /**
     * Constructor.
     */
    CamIds();

    /**
     * Copy-constructor
     */
    CamIds(const CamIds& other);

    /**
     * Destructor.
     */
    virtual ~CamIds();

    /**
     * Lists all available cameras.
     * @param cam_infos vector of all available cameras
     * @return number of listed cameras
     */
    int listCameras(std::vector<CamInfo>& cam_infos) const;

    /**
     * Returns information about the first camera that matches the pattern.
     * @param pattern camera pattern
     * @param structure where the camera information is stored
     * @return returns false if no camera matches
     */
//        bool findCamera(const CamInfo& pattern, CamInfo& cam) const;

    /**
     * Counts the number of available cameras.
     * @return number of available cameras
     */
    int countCameras() const;

    /**
     * Opens a specific camera.
     * @warning opening the last camera in a list after calling listCameras() \
     *      will not succeed; the daemon requires some time to close the camera \
     *      after is_ExitCamera()
     * @param cam details about the camera to be opened
     * @param mode one of the defined access modes
     * @return true if camera has been opened
     */
    bool open(const CamInfo& cam, const AccessMode mode = Master);

//        /**
//          * Opens the first camera that matches the pattern.
//          */
//        bool open2(const std::string& display_name, const AccessMode mode = Master);

//        /**
//          * Opens the first camera that matches the pattern.
//          */
//        bool open2(unsigned long& unique_camera_id, const AccessMode mode = Master);

//        /**
//          * Opens the first camera that matches the pattern.
//          */
//        bool open2(const CamInfo& pattern, const AccessMode mode = Master);

    /**
     * Checks if the camera is open.
     * @return true if the camera is opened
     */
    bool isOpen() const;

    /**
     * Returns a pointer to CamInfo of the opened camera.
     * @return information about a camera, or NULL if camera not initialized
     */
    const CamInfo* getCameraInfo() const;

    /**
     * Closes the camera.
     * @return true if camera has been closed
     */
    bool close();

    /**
     * Starts capturing into a buffer.
     * @param mode grab mode
     * @param buffer_length size of the buffer
     * @return true if successful
     */
    bool grab(const GrabMode mode = SingleFrame, const int buffer_len = 1);

    /**
     * Retrieves the next frame from the buffer(no data are copied).
     * @param frame destination frame
     * @param timeout timeout to wait for a frame in miliseconds
     */
    bool retrieveFrame(base::samples::frame::Frame& frame, const int timeout = 1000);

    /**
     * Checks if a frame can be retrieved from the buffer.
     * @return true if a new frame is available
     */
    bool isFrameAvailable();

//        /**
//          * Skips all buffered frame beside the last one.
//          */
//        int skipFrames();

//        /**
//          * Sets the IP adress and subnetmask of a ethernet camera.
//          */
//        bool setIpSettings(const CamInfo& cam, const IPSettings& ip_settings) const;

//        /**
//          * Sets the value of an integer attribute.
//          */
//        bool setAttrib(const int_attrib::CamAttrib attrib, const int value);

//        /**
//          * Sets the value of a double attribute.
//          */
//        bool setAttrib(const double_attrib::CamAttrib attrib, const double value);

//        /**
//          * Sets the value of a string attribute.
//          */
//        bool setAttrib(const str_attrib::CamAttrib attrib, const std::string value);

//        /**
//          * Sets the value of an enum attribute.
//          */
//        bool setAttrib(const enum_attrib::CamAttrib attrib);

//        /**
//          * Checks if an integer attribute is available.
//          */
          bool isAttribAvail(const int_attrib::CamAttrib attrib) { return false; }

//        /**
//          * Checks if a double attribute is available.
//          */
          bool isAttribAvail(const double_attrib::CamAttrib attrib) { return false; }

//        /**
//          * Checks if a string attribute is available.
//          */
          bool isAttribAvail(const str_attrib::CamAttrib attrib) { return false; }

//        /**
//          * Checks if a enum attribute is available.
//          */
          bool isAttribAvail(const enum_attrib::CamAttrib attrib) { return false; }

//        /**
//          * Returns the value of an integer attribute.
//          */
//        int getAttrib(const int_attrib::CamAttrib attrib);

//        /**
//          * Returns the value of a double attribute.
//          */
//        double getAttrib(const double_attrib::CamAttrib attrib);

//        /**
//          * Returns the value of a string attribute.
//          */
//        std::string getAttrib(const str_attrib::CamAttrib attrib);

//        /**
//          * Checks if the enum attribute is set.
//          */
//        bool isAttribSet(const enum_attrib::CamAttrib attrib);

//        /**
//          * Retrieves a camera frame(no data are copied).
//          */
//        CamInterface&  operator>>(base::samples::frame::Frame& frame);

    /**
     * Sets the frame settings size, mode and color depth.
     * @param size frame size
     * @param mode color mode
     * @param color_depth color depth
     * @param resize_frames boolean value to control whether or not to resize frames
     * @return true if successful
     */
    bool setFrameSettings(const base::samples::frame::frame_size_t size, const base::samples::frame::frame_mode_t mode, const uint8_t color_depth, const bool resize_frames = true);

//        /**
//          * Sets the camera frame settings to the values of the frame.
//          */
//        bool setFrameSettings(const base::samples::frame::Frame& frame, const bool resize_frames = true);

    /**
     * Gets the actual frame settings size, mode and color depth.
     * @param size output destination size
     * @param mode output destination mode
     * @param color_depth output destination color_depth
     * @return true if successful
     */
    bool getFrameSettings(base::samples::frame::frame_size_t& size, base::samples::frame::frame_mode_t& mode, uint8_t& color_depth);

//
//        /**
//          * Triggers a new frame if FrameStartTriggerMode is set to Software.
//          */
//        bool triggerFrame();

//        /**
//          * Sets the camera to default settings.
//          */
//        bool setToDefault();

//        /**
//          * Configures the frame that it matches the camera frame settings.
//          */
//        bool setFrameToCameraFrameSettings(base::samples::frame::Frame& frame);

//        /**
//          * Sets a callback function which is called when a new frame can be retrieved.
//          */
//        bool setCallbackFcn(void(*pcallback_function)(const void *p), void *p);

//        /**
//          * Synchronizes the camera time with the system time. Should be called only once.
//          */
//        void synchronizeWithSystemTime(uint32_t time_interval);

//        /**
//          * Saves the current camera configuration to the non-volatile memory inside the camera.
//          */
//        void saveConfiguration(uint8_t index);

//        /**
//          * Loads the camera configuration from the non-volatile memory.
//          */
//        void loadConfiguration(uint8_t index);

//        /**
//          * Returns the range of an double_attrib.
//          */
//        void getRange(const double_attrib::CamAttrib attrib, double& dmin, double& dmax);

//        /**
//          * Returns the range of an int_attrib.
//          */
//        void getRange(const int_attrib::CamAttrib attrib, int& imin, int& imax);

//        /**
//          * Returns the file descriptor of the camera.
//          */
//        int getFileDescriptor() const;

//        /**
//          * Does a camera diagnose.
//          */
//        std::string doDiagnose();
};

} // end of camera namespace

#endif // _CAMIDS_H


