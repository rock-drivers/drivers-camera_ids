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

    if (camera_list.size() >= 1) {
        cameth.open(camera_list[1]);
        camusb.open(camera_list[0]);
    }

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
