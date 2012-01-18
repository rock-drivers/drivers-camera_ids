#include "CamIds.h"
#include <iostream>

using namespace std;
using namespace camera;

int main() {
    CamIds camusb, cameth;

    std::vector<CamInfo> camera_list;

    cout << "Number of cameras: "
            << cameth.listCameras(camera_list)
            << endl;

    showCamInfos(camera_list);

//    cameth.grab(SingleFrame, 1);

    if (camera_list.size() >= 1) {
        cameth.open(camera_list[1]);
//        camusb.open(camera_list[0]);
    }

    //camusb.grab(SingleFrame, 1);
    cameth.grab(SingleFrame, 1);
//    camusb.grab(SingleFrame, 1);
    base::samples::frame::Frame frame;
    cameth.retrieveFrame(frame, 1000);

//    for (int i = 0; i < frame.size.height; ++i) {
//        for (int j = 0; j < frame.size.width * frame.pixel_size; ++j) {
//            std::cout << (int)frame.image[3 * i * frame.size.width + j] << " ";
//        }
//        std::cout << std::endl;
//    }

    camera_list.clear();

//    const CamInfo* camPt = cam.getCameraInfo();
//
//    if (camPt != NULL) {
//        camera_list.push_back(*camPt);
//    }

//    showCamInfos(camera_list);

    std::cout << "Is it open eth: " << cameth.isOpen() << std::endl;
    std::cout << "Is it open usb: " << camusb.isOpen() << std::endl;
    return 0;
}
