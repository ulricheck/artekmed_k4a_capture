
#include <Corrade/configure.h>
#include <Corrade/Utility/Arguments.h>

#include <Magnum/GL/DefaultFramebuffer.h>

#include <Magnum/Platform/GlfwApplication.h>

#include <Magnum/ImageView.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/GL/PixelFormat.h>


#include <memory>
#include <iostream>
#include <vector>
#include <chrono>
#include <deque>
#include <mutex>

#include <opencv2/opencv.hpp>
#include "nvenc_rtsp/ClientPipeRTSP.h"


using namespace Magnum;

struct Frame {
  cv::Mat depthImage;
  cv::Mat colorImage;
  uint64_t timestamp{0};
  bool have_colorImage{false};
  bool have_depthImage{false};
};

class K4AStreamReceiver : public Platform::GlfwApplication {
public:
    explicit K4AStreamReceiver(const Arguments &arguments);

private:
    void drawEvent() override;

    void forwardCompletedFrames();

    std::shared_ptr<nvenc_rtsp::ClientPipeRTSP> m_colorStream;
    std::shared_ptr<nvenc_rtsp::ClientPipeRTSP> m_depthStream;

    size_t m_maxReceiveQueueSize{16};
    std::deque<Frame> m_resultQueue;
    std::deque<Frame> m_receiveQueue;
    std::mutex m_receiverWriteLock;

    std::string m_depthStreamUrl;
    std::string m_colorStreamUrl;
};

K4AStreamReceiver::K4AStreamReceiver(const Arguments &arguments) : Platform::GlfwApplication{arguments} {
    Magnum::Utility::Arguments args;
    args.addOption("ip", "127.0.0.1").setHelp("ip", "IP Address to bind to")
        .addOption("port", "55555").setHelp("port", "Port to bind to")
        .addSkippedPrefix("magnum", "engine-specific options");

    args.parse(arguments.argc, arguments.argv);

    std::string ipAddress = args.value("ip");
    unsigned int srcPort = args.value<Magnum::Int>("port");

    m_depthStreamUrl = "rtsp://" + ipAddress + ":" + std::to_string(srcPort) + "/live";
    m_colorStreamUrl = "rtsp://" + ipAddress + ":" + std::to_string(srcPort+1) + "/live";

    m_depthStream = std::make_shared<nvenc_rtsp::ClientPipeRTSP>(m_depthStreamUrl, NVPIPE_UINT16, NVPIPE_H264,
                                                                 [&](cv::Mat mat, uint64_t timestamp) {

                                                                     Magnum::Debug{} << "Received Depth Image: "
                                                                                     << timestamp;
                                                                     std::lock_guard<std::mutex> lock(
                                                                             m_receiverWriteLock);

                                                                     bool frameStored{false};
                                                                     for (Frame &frame: m_receiveQueue) {
                                                                         if (!frame.have_depthImage) {
                                                                             if (frame.timestamp == timestamp) {
                                                                                 frame.have_depthImage = true;
                                                                                 frame.depthImage = mat;
                                                                                 Magnum::Debug{}
                                                                                         << "Completed Frame with depthImage "
                                                                                         << timestamp;
                                                                                 frameStored = true;
                                                                                 break;
                                                                             } else {
                                                                                 Magnum::Debug{}
                                                                                         << "Have frame non-matching timestamp: "
                                                                                         << frame.timestamp << " != "
                                                                                         << timestamp;
                                                                             }
                                                                         }
                                                                     }

                                                                     if (!frameStored) {
                                                                         Frame frame;
                                                                         frame.have_depthImage = true;
                                                                         frame.timestamp = timestamp;
                                                                         frame.depthImage = mat;
                                                                         Magnum::Debug{}
                                                                                 << "Created Frame with depthImage "
                                                                                 << timestamp;
                                                                         m_receiveQueue.emplace_back(std::move(frame));
                                                                     }

                                                                     forwardCompletedFrames();
                                                                 });

    m_colorStream = std::make_shared<nvenc_rtsp::ClientPipeRTSP>(m_colorStreamUrl, NVPIPE_RGBA32, NVPIPE_H264,
                                                                 [&](cv::Mat mat, uint64_t timestamp) {

                                                                     Magnum::Debug{} << "Received Color Image: "
                                                                                     << timestamp;
                                                                     std::lock_guard<std::mutex> lock(
                                                                             m_receiverWriteLock);

                                                                     bool frameStored{false};
                                                                     for (Frame &frame: m_receiveQueue) {
                                                                         if (!frame.have_colorImage) {
                                                                             if (frame.timestamp == timestamp) {
                                                                                 frame.have_colorImage = true;
                                                                                 frame.colorImage = mat;
                                                                                 Magnum::Debug{}
                                                                                         << "Completed Frame with colorImage "
                                                                                         << timestamp;
                                                                                 frameStored = true;
                                                                                 break;
                                                                             } else {
                                                                                 Magnum::Debug{}
                                                                                         << "Have frame non-matching timestamp: "
                                                                                         << frame.timestamp << " != "
                                                                                         << timestamp;
                                                                             }
                                                                         }
                                                                     }

                                                                     if (!frameStored) {
                                                                         Frame frame;
                                                                         frame.have_colorImage = true;
                                                                         frame.timestamp = timestamp;
                                                                         frame.colorImage = mat;
                                                                         Magnum::Debug{}
                                                                                 << "Created Frame with colorImage "
                                                                                 << timestamp;
                                                                         m_receiveQueue.emplace_back(std::move(frame));
                                                                     }

                                                                     forwardCompletedFrames();
                                                                 });

    Magnum::Debug{} << "Finished starting receivers.";
}

void K4AStreamReceiver::drawEvent() {
    Magnum::Debug{} << "Begin drawEvent";
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color);

    if (!m_resultQueue.empty()) {
        Frame& frame = m_resultQueue.front();
        m_resultQueue.pop_front();
        Magnum::Debug{} << "Received complete frame for rendering: " << frame.timestamp;
    }

    swapBuffers();
    redraw();
}

void K4AStreamReceiver::forwardCompletedFrames() {
    // forward all completed frames (only pop from front..
    while ((m_receiveQueue.front().have_colorImage) && (m_receiveQueue.front().have_depthImage)) {
        m_resultQueue.emplace_back(m_receiveQueue.front());
        m_receiveQueue.pop_front();
    }

    // cleanup queue remove leftover incomplete frames
    while (m_receiveQueue.size() > m_maxReceiveQueueSize) {
        Magnum::Warning{} << "Dropping incomplete frame: " << m_receiveQueue.front().timestamp;
        m_receiveQueue.pop_front();
    }
}

MAGNUM_APPLICATION_MAIN(K4AStreamReceiver)
