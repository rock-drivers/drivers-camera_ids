#include <iostream>
#include <sstream>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <boost/program_options.hpp>

#include "CamIds.h"

using namespace base::samples::frame;
namespace po = boost::program_options;

int main(int argc, char**argv) {

    po::options_description desc("Options");
    desc.add_options()
        ("help", "show help")
        ("list", "show camera list")
        ("id", po::value<int>()->default_value(0), "camera id to open")
        ("width,w", po::value<int>(), "width of the image")
        ("height,h", po::value<int>(), "height of the image")
        ("fps,f", po::value<double>()->default_value(15), "frame rate")
        ("framemode,m", po::value<int>(), "frame mode")
        ("colordepth,c", po::value<int>(), "color depth")
        ("exposure,e", po::value<int>(), "exposure")
        ("pixelclock,p", po::value<int>(), "pixelclock");
   
    po::variables_map vm;
    po::store(po::parse_command_line(argc,argv,desc), vm); 
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }
    
    camera::CamIds cam;

    if (vm.count("list")) {

        std::vector<camera::CamInfo> camera_list;

        std::cout << "Number of cameras: "
                << cam.listCameras(camera_list)
                << std::endl;

        camera::showCamInfos(camera_list);
        return 1;
    }

    camera::CamInfo cam_info;
    cam_info.unique_id = vm["id"].as<int>();

    cam.open(cam_info);

    //Frame settings
    frame_size_t fsize;
    frame_mode_t fmode;
    uint8_t cdepth;
    cam.getFrameSettings(fsize, fmode, cdepth);
    std::cout << "Standard frame settings: " << fsize.width << " x " << fsize.height << 
        " depth: " << (int)cdepth << " mode: " << fmode <<  std::endl;


    // test attributes here
    if (vm.count("width") )
        fsize.width = vm["width"].as<int>();
    if (vm.count("height") )
        fsize.height = vm["height"].as<int>();
    if ( vm.count("framemode") )
        fmode = (frame_mode_t) vm["framemode"].as<int>();
    if ( vm.count("colordepth") )
        cdepth = vm["colordepth"].as<int>();
    std::cout << "New frame settings: " << fsize.width << " x " << fsize.height << 
        " depth: " << (int)cdepth << " mode: " << fmode <<  std::endl;
    cam.setFrameSettings(fsize, fmode, cdepth, true);

    if ( vm.count("pixelclock") )
        cam.setAttrib(camera::int_attrib::PixelClock, vm["pixelclock"].as<int>());
    if ( vm.count("fps") )
        cam.setAttrib(camera::double_attrib::FrameRate, vm["fps"].as<double>());
    
    // start grabbing
    cam.grab(camera::Continuously, 5);

    base::samples::frame::Frame frame;
    base::Time prev = base::Time::now();
    int nrFrames = 1000, count = 0;

    //cv::namedWindow("frame", CV_WINDOW_AUTOSIZE);

    std::cout << "Ctrl-C for exit!" << std::endl;
    //for (int i = 0; i < nrFrames; ++i) {
    
    while ( cv::waitKey(5) == -1 ) {
        if ( cam.isFrameAvailable() ) {

            base::Time cur = base::Time::now();
            std::stringstream ss;
            ss << "fps: " << 1. / (cur-prev).toSeconds();
            prev = cur;
            
            cam.retrieveFrame(frame, 500);
            if ( count %10 == 0) {
                cv::Mat img = frame.convertToCvMat();
                cv::putText(img, ss.str(), cv::Point(10,50), cv::FONT_HERSHEY_SIMPLEX,
                    0.8, CV_RGB(255,0.0,0.0), 1, 8, false);
                cv::imshow("frame", img);
            }
            count++;
        }
    }
    
    cam.grab(camera::Stop, 0);

    cam.close();

    return 0;
}
