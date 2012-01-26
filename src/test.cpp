#include <iostream>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include "CamIds.h"
#include <sys/stat.h>

using namespace camera;
using namespace base::samples::frame;
using namespace std;

int main(int argc, char**argv)
{
    CamIds cam, cameth;

    std::vector<CamInfo> camera_list;

    cout << "Number of cameras: "
            << cam.listCameras(camera_list)
            << endl;

    showCamInfos(camera_list);

    if (camera_list.size() >= 1) {
        cam.open(camera_list[1]);
    }

    cam.grab(Continuously, 5);

    base::samples::frame::Frame frame;
    base::Time prev = base::Time::now();
    int nrFrames = 10000, count = 0;

    cv::namedWindow("frame", CV_WINDOW_FREERATIO);

    for (int i = 0; i < nrFrames; ++i) {
        cout << "Frame number " << i+1 << endl;

        int avail = cam.isFrameAvailable();
        cout << "Frame available: " << avail << endl;

        if (avail) {
            count++;
        }

        cam.retrieveFrame(frame, 500);

        cvWaitKey(10);
        cv::imshow("frame", frame.convertToCvMat());

        cout << "Actual fps: " << count * 1000000.0 / (base::Time::now().microseconds - prev.microseconds) << endl;
        cout << endl;
    }
    cout << "Average fps: " << nrFrames * 1000000.0 / (base::Time::now().microseconds - prev.microseconds) << endl;

    cam.grab(Stop, 0);

    cam.close();

    std::cout << "Is it open: " << cam.isOpen() << std::endl;

    return 0;
}


