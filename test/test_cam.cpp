/** \file test_cam.cpp
 * Test application for the camera_ids driver.
 */

#include <iostream>
#include <sstream>
#include <algorithm>
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
        ("blind", "without showing the image")
        ("id", po::value<int>()->default_value(0), "camera id to open")
        ("width,w", po::value<int>(), "width of the image")
        ("height,h", po::value<int>(), "height of the image")
        ("posx,x", po::value<int>(), "x position of aoi")
        ("posy,y", po::value<int>(), "y position of aoi")
        ("mirrorx", "mirror horizontally")
        ("mirrory", "mirros vertically")
        ("binningx", po::value<int>(), "binning in x direction")
        ("binningy", po::value<int>(), "binning in y direction")
        ("fps,f", po::value<double>()->default_value(15), "frame rate in Hz")
        ("framemode,m", po::value<int>(), "frame mode")
        ("colordepth,c", po::value<int>(), "color depth")
        ("exposure,e", po::value<int>(), "exposure in us")
        ("pixelclock,p", po::value<int>(), "pixelclock in MHz")
        ("framebuffer,b",po::value<int>()->default_value(5),"framebuffer count");
   
    po::variables_map vm;
    po::store(po::parse_command_line(argc,argv,desc), vm); 
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
        return 1;
    }
    
    camera::CamIds cam; //instanciate the driver class

    if (vm.count("list")) {
        // Shows a list of all availavle cameras.

        std::vector<camera::CamInfo> camera_list;

        std::cout << "Number of cameras: "
                << cam.listCameras(camera_list)
                << std::endl;

        camera::showCamInfos(camera_list);
        return 1;
    }

    // open takse a CamInfo object to indentify the camera to open vie the unique_id
    // in case none is given it takes the first available (see the list).
    camera::CamInfo cam_info;
    cam_info.unique_id = vm["id"].as<int>();
    cam.open(cam_info);

    // Get initial frame settings
    frame_size_t fsize;
    frame_mode_t fmode;
    uint8_t cdepth;
    cam.getFrameSettings(fsize, fmode, cdepth);
    std::cout << "Standard frame settings: " << fsize.width << " x " << fsize.height << 
        " depth: " << (int)cdepth << " mode: " << fmode <<  std::endl;


    // set the frame attributes here
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

    // set further attributes here
    if ( vm.count("posx") )
        cam.setAttrib(camera::int_attrib::RegionX, vm["posx"].as<int>());
    if ( vm.count("posy") )
        cam.setAttrib(camera::int_attrib::RegionY, vm["posy"].as<int>());
    if ( vm.count("mirrorx") )
        cam.setAttrib(camera::enum_attrib::MirrorXToOn);
    if ( vm.count("mirrory") )
        cam.setAttrib(camera::enum_attrib::MirrorYToOn);
    // for width and height always set the values before binning
    // (e.g. setting width=640 with binningx=2 gives an image width of 320)
    if ( vm.count("binningx") )
        cam.setAttrib(camera::int_attrib::BinningX, vm["binningx"].as<int>() );
    if ( vm.count("binningy") )
        cam.setAttrib(camera::int_attrib::BinningY, vm["binningy"].as<int>() );

    // these settings should be made after setting the image geometry because the 
    // value range of the properties changes with the image size
    if ( vm.count("pixelclock") )
        cam.setAttrib(camera::int_attrib::PixelClock, vm["pixelclock"].as<int>());
    if ( vm.count("fps") )
        cam.setAttrib(camera::double_attrib::FrameRate, vm["fps"].as<double>());
    if ( vm.count("exposure") )
        cam.setAttrib(camera::int_attrib::ExposureValue, vm["exposure"].as<int>());
    
    // start grabbing in continious mode
    cam.grab(camera::Continuously, vm["framebuffer"].as<int>());

    base::samples::frame::Frame frame;
    base::Time prev = base::Time::now();
    int nrFrames = 1000, count = 0;

    //cv::namedWindow("frame", CV_WINDOW_AUTOSIZE);

    std::cout << "Ctrl-C for exit!" << std::endl;
    //for (int i = 0; i < nrFrames; ++i) {
    
    if ( vm.count("blind") ) {
        // blind capture
        std::cout << "get for 10 s" << std::endl;
        int fcnt;
        base::Time start = base::Time::now();
        std::vector<double> fpslist;
        while ( (base::Time::now()-start).toSeconds() < 10.0 ) {
            // caputring is to wait for a new image first
            // isFrameAvailable waits a certain amount of time for a new image
            if ( cam.isFrameAvailable() ) {
                base::Time cur = base::Time::now();
                fpslist.push_back ( (cur-prev).toSeconds() );
                prev = cur;
                // if there is a new image get it into frame
                cam.retrieveFrame(frame, 50);
            }
        }

    } else {
        std::vector<double> fps;
        unsigned int cnt = 0;
        while ( cv::waitKey(5) == -1 ) {
            if ( cam.isFrameAvailable() ) {

                cam.retrieveFrame(frame, 100);

                base::Time cur = frame.time;
                std::stringstream ss;
                double period = (cur - prev).toSeconds();
                if (cnt>0) fps.push_back(1./period);
                ss << "fps: " << 1. / period;
                prev = cur;
                cnt ++;
                
                ss << " size: " << frame.size.width << " x " << frame.size.height;
                ss << " cnt: " << frame.getAttribute<uint64_t>("FrameCount");
                cv::Mat img = frame.convertToCvMat();
                cv::putText(img, ss.str(), cv::Point(10,20), cv::FONT_HERSHEY_SIMPLEX,
                    0.5, CV_RGB(128,255,0.0), 1, 8, false);
                cv::imshow("frame", img);
            }
        }
        std::cout << "Received " << cnt << " frames." << std::endl;
        std::cout << "min fps: " << *std::min_element(fps.begin(), fps.end()) 
            << std::endl;
        std::cout << "max fps: " << *std::max_element(fps.begin(), fps.end()) 
            << std::endl;
    }
   
    // stop grabbing and close the camera. 
    cam.grab(camera::Stop, 0);
    cam.close();

    return 0;
}
