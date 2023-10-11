#include <chrono>
#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static constexpr auto FPS = 8;

int main() {

    dai::Pipeline pipeline;
    auto odispa = pipeline.create<dai::node::XLinkOut>();
    auto ocolor = pipeline.create<dai::node::XLinkOut>();

    ocolor->setStreamName("rgb");
    odispa->setStreamName("disp");


    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    camRgb->setVideoNumFramesPool(10);
    camRgb->setFps(FPS);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
    camRgb->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::INPUT);

    auto left = pipeline.create<dai::node::MonoCamera>();
    left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
    left->setCamera("left");
    left->setFps(FPS);
    left->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::OUTPUT);

    auto right = pipeline.create<dai::node::MonoCamera>();
    right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
    right->setCamera("right");
    right->setFps(FPS);
    right->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::INPUT);

    auto stereo = pipeline.create<dai::node::StereoDepth>();
    stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
    stereo->setLeftRightCheck(true);
    stereo->setExtendedDisparity(false);
    stereo->setSubpixel(false);
    stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);
    left->out.link(stereo->left);
    right->out.link(stereo->right);


    stereo->disparity.link(odispa->input);
    camRgb->isp.link(ocolor->input);


    dai::Device device(pipeline);
    dai::Device::Config c = pipeline.getDeviceConfig();
    c.board.gpio[6] = dai::BoardConfig::GPIO(dai::BoardConfig::GPIO::Direction::OUTPUT, dai::BoardConfig::GPIO::Level::HIGH);
    c.board.gpio[41] = dai::BoardConfig::GPIO(dai::BoardConfig::GPIO::Direction::OUTPUT, dai::BoardConfig::GPIO::Level::LOW);
    
    device.setLogOutputLevel(dai::LogLevel::DEBUG);
    device.setLogLevel(dai::LogLevel::DEBUG);
  
    auto dispq = device.getOutputQueue("disp",1, false);
    auto rgbq = device.getOutputQueue("rgb",1, false);


    while(true) {

        auto dispFrame = dispq->get<dai::ImgFrame>();
        auto rgbFrame = rgbq->get<dai::ImgFrame>();

        if(dispFrame){
            std::cout << "DISP " << dispFrame->getSequenceNum() << std::endl; 
        }

        if(rgbFrame){
            std::cout << "RGB " << rgbFrame->getSequenceNum() << std::endl; 
        }


    }
}





