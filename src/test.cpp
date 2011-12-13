#include "CamIds.h"
#include <iostream>

using namespace std;
using namespace camera;

int main() {
    CamIds cam;

    std::vector<CamInfo> camera_list;

    cout << "Number of cameras: "
            << cam.listCameras(camera_list)
            << endl;


    if (camera_list.size() >= 1) {
        cam.open(camera_list[0]);
    }

    std::cout << "Is it open: " << cam.isOpen() << std::endl;

    return 0;
}
