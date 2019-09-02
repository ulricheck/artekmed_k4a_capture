
#include <Corrade/configure.h>
#include <Corrade/Utility/Arguments.h>

#include <Magnum/GL/DefaultFramebuffer.h>

#ifdef CORRADE_TARGET_UNIX
#include <Magnum/Platform/WindowlessGlxApplication.h>
#endif
#ifdef CORRADE_TARGET_WINDOWS
#include <Magnum/Platform/WindowlessWglApplication.h>
#endif


#include <Magnum/ImageView.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/GL/PixelFormat.h>


#include <memory>
#include <iostream>
#include <vector>
#include <chrono>

#include <opencv2/opencv.hpp>
#include "nvenc_rtsp/ServerPipeRTSP.h"


using namespace Magnum;

class OpenCVStreamSender : public Platform::WindowlessApplication {
public:
    explicit OpenCVStreamSender(const Arguments &arguments);
    int exec() override;

private:
    std::shared_ptr<cv::VideoCapture> m_dev;

    std::shared_ptr<nvenc_rtsp::ServerPipeRTSP> m_colorStream;

    std::string m_ipAddress{"127.0.0.1"};
    unsigned int m_dstPort{55555};
    unsigned int m_bitrate{50};
    unsigned int m_framerate{30};

    bool m_debug{false};

    bool m_colorStreamEnabled{true};

};

OpenCVStreamSender::OpenCVStreamSender(const Arguments &arguments) : Platform::WindowlessApplication{arguments} {
    Magnum::Utility::Arguments args;
    args.addOption("ip", "127.0.0.1").setHelp("ip", "IP Address to bind to")
        .addOption("port", "55555").setHelp("port", "Port to bind to")
        .addBooleanOption("debug").setHelp("debug", "Show debug windows")
        .addBooleanOption("nocolor").setHelp("nocolor", "Skip color stream")
        .addSkippedPrefix("magnum", "engine-specific options");

    args.parse(arguments.argc, arguments.argv);

    m_ipAddress = args.value("ip");
    m_dstPort = args.value<Magnum::Int>("port");
    m_debug = args.isSet("debug");
    m_colorStreamEnabled = !args.isSet("nocolor");


    m_dev = std::make_shared<cv::VideoCapture>(0);
    if (!m_dev->isOpened()) {
        throw std::runtime_error("No OpenCV camera detected!");
    }

    Magnum::Debug{} << "Finished opening OpenCV camera device.";

}

int OpenCVStreamSender::exec() {
    Magnum::Debug{} << "Start Streaming";
    try{
        if (m_debug) {
            cv::namedWindow("ColorImage",cv::WINDOW_NORMAL);
        }
        cv::Mat colorImage;
        cv::Mat depthImage;

        for(;;) {
            *m_dev >> colorImage;
            if (m_colorStreamEnabled) {

                if (!m_colorStream) {
                    m_colorStream = std::make_shared<nvenc_rtsp::ServerPipeRTSP>(m_ipAddress, m_dstPort+1, NVPIPE_RGBA32, NVPIPE_LOSSLESS, NVPIPE_H264, m_bitrate, m_framerate);
                }

                cv::Mat image;
                cv::cvtColor(colorImage, image, cv::COLOR_BGR2RGBA);
                unsigned long ts = std::chrono::system_clock::now().time_since_epoch().count();
                m_colorStream->send_frame(image);
                if (m_debug) {
                    Magnum::Debug{} << "got color images: " << ts << "elemSize: " << colorImage.elemSize()  << "elemSize1: " << colorImage.elemSize1();
                    cv::imshow("ColorImage", colorImage);
                    if(cv::waitKey(5) >= 0) break;
                }
            }

        }
    } catch (std::exception &e) {
        Magnum::Debug{} << "Exception: " << e.what();
        return 1;
    }
    m_dev->release();
    return 0;
}

MAGNUM_WINDOWLESSAPPLICATION_MAIN(OpenCVStreamSender)
