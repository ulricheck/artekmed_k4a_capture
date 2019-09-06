
#include <Corrade/configure.h>
#include <Corrade/Utility/Arguments.h>
#include <Corrade/Utility/System.h>

#include <Magnum/GL/DefaultFramebuffer.h>

#include <Magnum/Platform/GlfwApplication.h>

#include <Magnum/ImageView.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/GL/PixelFormat.h>


#include <atomic>
#include <memory>
#include <iostream>
#include <vector>
#include <chrono>
#include <deque>
#include <mutex>
#include <thread>

#include <opencv2/opencv.hpp>
#include <NvPipe.h>
#include "h264_stream.h"

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <azmq/socket.hpp>

using namespace Magnum;

struct Frame {

    Frame() = default;
    Frame(const Frame& other) noexcept {
        depthImage = other.depthImage;
        colorImage = other.colorImage;
        have_depthImage = other.have_depthImage;
        have_colorImage = other.have_colorImage;
        timestamp = other.timestamp;
        is_valid = other.is_valid;
    }

    Frame& operator=(const Frame& other)  noexcept {
        depthImage = other.depthImage;
        colorImage = other.colorImage;
        have_depthImage = other.have_depthImage;
        have_colorImage = other.have_colorImage;
        timestamp = other.timestamp;
        is_valid = other.is_valid;
        return *this;
    }

    Frame(Frame&& other) noexcept
    : depthImage(std::move(other.depthImage))
    , colorImage(std::move(other.colorImage))
    , timestamp(other.timestamp)
    , have_colorImage(other.have_colorImage)
    , have_depthImage(other.have_depthImage)
    , is_valid(other.is_valid)
    {}

    cv::Mat depthImage;
    cv::Mat colorImage;
    uint64_t timestamp{0};
    bool have_colorImage{false};
    bool have_depthImage{false};
    bool is_valid{false};
};

class K4AStreamReceiver : public Platform::GlfwApplication {
public:
    explicit K4AStreamReceiver(const Arguments &arguments);

    ~K4AStreamReceiver();


    void watchdogTimer();
    void receiveDepthFrame();
    void receiveColorFrame();


private:
    void drawEvent() override;

    void forwardCompletedFrames();


    NvPipe* m_colorStreamDecoder{nullptr};
    NvPipe* m_depthStreamDecoder{nullptr};
    std::vector<uint8_t> m_colorBuffer;
    std::vector<uint8_t> m_depthBuffer;
    std::atomic<unsigned int> m_colorBufferIndex{0};
    std::atomic<unsigned int> m_depthBufferIndex{0};

    std::shared_ptr<h264_stream_t> m_h264parser_depth;
    std::shared_ptr<h264_stream_t> m_h264parser_color;

    std::shared_ptr<std::thread > m_NetworkThread;
    std::shared_ptr<boost::asio::io_service> m_ioservice;
    std::shared_ptr<boost::asio::deadline_timer> m_ioserviceKeepAlive;
    std::shared_ptr<azmq::socket> m_zmq_sub_socket;
    std::shared_ptr<azmq::socket> m_zmq_depth_stream_socket;
    std::shared_ptr<azmq::socket> m_zmq_color_stream_socket;


    size_t m_maxReceiveQueueSize{16};
    std::deque<Frame> m_resultQueue;
    std::deque<Frame> m_receiveQueue;
    std::mutex m_receiverWriteLock;

    std::string m_configStreamUrl;
    std::string m_depthStreamUrl;
    std::string m_colorStreamUrl;

    bool m_debug{false};
    bool m_display{false};

    bool m_depthStreamEnabled{true};
    bool m_colorStreamEnabled{true};
};


K4AStreamReceiver::K4AStreamReceiver(const Arguments &arguments) : Platform::GlfwApplication{arguments} {
    Magnum::Utility::Arguments args;
    args.addOption("ip", "127.0.0.1").setHelp("ip", "IP Address to bind to")
        .addOption("depthport", "55555").setHelp("depthport", "Port to connect depth stream")
        .addOption("colorport", "55556").setHelp("colorport", "Port to connect color stream")
        .addOption("configport", "55557").setHelp("configport", "Port to connect config")
        .addBooleanOption("debug").setHelp("debug", "Show debug windows")
        .addBooleanOption("nodepth").setHelp("nodepth", "Skip depth stream")
        .addBooleanOption("nocolor").setHelp("nocolor", "Skip color stream")
        .addBooleanOption("display").setHelp("display", "Display images")
        .addSkippedPrefix("magnum", "engine-specific options");

    args.parse(arguments.argc, arguments.argv);

    std::string ipAddress = args.value("ip");
    unsigned int srcDepthPort = args.value<Magnum::Int>("depthport");
    unsigned int srcColorPort = args.value<Magnum::Int>("colorport");
    unsigned int srcConfigPort = args.value<Magnum::Int>("configport");
    m_debug = args.isSet("debug");
    m_display = args.isSet("display");
    m_depthStreamEnabled = !args.isSet("nodepth");
    m_colorStreamEnabled = !args.isSet("nocolor");

    m_depthStreamUrl = "tcp://" + ipAddress + ":" + std::to_string(srcDepthPort);
    m_colorStreamUrl = "tcp://" + ipAddress + ":" + std::to_string(srcColorPort);
    m_configStreamUrl = "tcp://" + ipAddress + ":" + std::to_string(srcConfigPort);

    // start communication services
    Magnum::Debug{} << "Create IOService";
    m_ioservice = std::make_shared<boost::asio::io_service>();
    m_ioserviceKeepAlive = std::make_shared<boost::asio::deadline_timer>(*m_ioservice);
    watchdogTimer();
    m_NetworkThread = std::make_shared< std::thread >( [&]{ m_ioservice->run(); } );
    m_zmq_sub_socket = std::make_shared< azmq::socket >( *m_ioservice, ZMQ_SUB );
    m_zmq_depth_stream_socket = std::make_shared< azmq::socket >( *m_ioservice, ZMQ_SUB );
    m_zmq_color_stream_socket = std::make_shared< azmq::socket >( *m_ioservice, ZMQ_SUB );

    m_h264parser_depth.reset(h264_new());
    m_h264parser_color.reset(h264_new());


    // request config via req/rep here to get resolution/intrincics ..


    if (m_depthStreamEnabled) {

        m_zmq_depth_stream_socket->connect(m_depthStreamUrl);
        m_zmq_depth_stream_socket->set_option(azmq::socket::subscribe(""));
        receiveDepthFrame();

        if (m_display) {
            cv::namedWindow("DepthImage",cv::WINDOW_NORMAL);
        }
    }

    if (m_colorStreamEnabled) {
        m_zmq_color_stream_socket->connect(m_colorStreamUrl);
        m_zmq_color_stream_socket->set_option(azmq::socket::subscribe(""));
        receiveColorFrame();

        if (m_display) {
            cv::namedWindow("ColorImage",cv::WINDOW_NORMAL);
        }
    }
    Magnum::Debug{} << "Finished starting receivers.";
}

K4AStreamReceiver::~K4AStreamReceiver () {
    try {
        m_ioservice->stop();
        m_NetworkThread->join();
    } catch (std::exception &e) {
        Magnum::Debug{} << "Exception while tearing down: " << e.what();
    }
}

void K4AStreamReceiver::watchdogTimer() {
    m_ioserviceKeepAlive->expires_from_now(boost::posix_time::seconds(1));
    m_ioserviceKeepAlive->async_wait(boost::bind(&K4AStreamReceiver::watchdogTimer, this));
}


void K4AStreamReceiver::receiveDepthFrame() {

    m_zmq_depth_stream_socket->async_receive([this] (const boost::system::error_code& error, azmq::message& message, size_t ) {
        if (!error) {

            if (m_debug) {
                Magnum::Debug{} << "Received depth message " << message.size() << " bytes";
            }

            auto ts = std::chrono::system_clock::now().time_since_epoch().count();
            // hardcoded for now
            int w = 1024;
            int h = 1024;
            int bytesPerPixel = sizeof(uint16_t);

            if (!m_depthStreamDecoder) {

                m_depthStreamDecoder = NvPipe_CreateDecoder(NVPIPE_UINT16, NVPIPE_H264, w, h);
                if (!m_depthStreamDecoder) {
                    Magnum::Error{} << "Failed to create depth encoder: " << NvPipe_GetError(nullptr);
                    // need to handle failure ..
                    return;
                }
                m_depthBuffer.resize(w * h * bytesPerPixel);
            }

            // parse stream
            size_t rsz = 0;
            size_t sz = message.size();
            int64_t off = 0;

            // should change the library function ..
            uint8_t* p = const_cast<uint8_t*>(static_cast<const uint8_t*>(message.data()));

            int nal_start, nal_end;

            while (find_nal_unit(p, sz, &nal_start, &nal_end) > 0)
            {
                p += nal_start;
                int nal_unit_type = peek_nal_unit(m_h264parser_color.get(), p, nal_end - nal_start);

                // only send "Coded slice of an IDR picture" packages to decoder
                if (nal_unit_type == 5) {

                    uint64_t size = NvPipe_Decode(m_depthStreamDecoder, p,nal_end - nal_start, m_depthBuffer.data(), w, h);
                    if (size == 0) {
                        Magnum::Error{} << "Decode error depth: " << NvPipe_GetError(m_depthStreamDecoder);
                    } else {
                        auto mat = cv::Mat({w,h}, CV_16UC1, &m_depthBuffer[0], cv::Mat::AUTO_STEP);

                        if (m_debug) {
                            Magnum::Debug{} << "Received Depth Image: " << ts << " size: " << mat.size();
                        }
                        std::lock_guard<std::mutex> lock(m_receiverWriteLock);

                        bool frameStored{false};
                        for (Frame &frame: m_receiveQueue) {
                            if (!frame.have_depthImage) {
                                if (frame.timestamp == ts) {
                                    frame.have_depthImage = true;
                                    frame.depthImage = mat;
                                    if (m_debug) {
                                        Magnum::Debug{} << "Completed Frame with depthImage " << ts;
                                    }
                                    frameStored = true;
                                    break;
                                } else {
                                    Magnum::Debug{} << "Have frame non-matching timestamp: " << frame.timestamp << " != " << ts;
                                }
                            }
                        }

                        if (!frameStored) {
                            Frame frame;
                            frame.have_depthImage = true;
                            frame.timestamp = ts;
                            frame.depthImage = mat;
                            if (m_debug) {
                                Magnum::Debug{} << "Created Frame with depthImage " << ts;
                            }
                            m_receiveQueue.push_back(std::move(frame));
                        }
                    }
                }

                p += (nal_end - nal_start);
                sz -= nal_end;
            }
        }
        // wait for next message
        receiveDepthFrame();
    });
}

void K4AStreamReceiver::receiveColorFrame() {

    m_zmq_color_stream_socket->async_receive([this] (const boost::system::error_code& error, azmq::message& message, size_t ) {
        if (!error) {

            if (m_debug) {
                Magnum::Debug{} << "Received color message " << message.size() << " bytes";
            }

            auto ts = std::chrono::system_clock::now().time_since_epoch().count();
            // hardcoded for now
            int w = 2048;
            int h = 1536;
            int bytesPerPixel = 4 * sizeof(uint8_t);

            if (!m_colorStreamDecoder) {

                m_colorStreamDecoder = NvPipe_CreateDecoder(NVPIPE_RGBA32, NVPIPE_H264, w, h);
                if (!m_colorStreamDecoder) {
                    Magnum::Error{} << "Failed to create color encoder: " << NvPipe_GetError(nullptr);
                    // need to handle failure ..
                    return;
                }
                m_colorBuffer.resize(w * h * bytesPerPixel);
            }


            size_t rsz = 0;
            size_t sz = message.size();
            int64_t off = 0;

            // should change the library function ..
            uint8_t* p = const_cast<uint8_t*>(static_cast<const uint8_t*>(message.data()));

            int nal_start, nal_end;

            while (find_nal_unit(p, sz, &nal_start, &nal_end) > 0)
            {
                p += nal_start;
                int nal_unit_type = peek_nal_unit(m_h264parser_color.get(), p, nal_end - nal_start);
                // only send "Coded slice of an IDR picture" packages to decoder
                if (nal_unit_type == 5) {
                    uint64_t size = NvPipe_Decode(m_colorStreamDecoder, static_cast<const uint8_t*>(message.data()), message.size(), m_colorBuffer.data(), w, h);
                    if (size == 0) {
                        Magnum::Error{} << "Decode error color: " << NvPipe_GetError(m_colorStreamDecoder);
                    }  else {
                        auto mat = cv::Mat({w,h}, CV_8UC4, &m_colorBuffer[0], cv::Mat::AUTO_STEP);

                        if (m_debug) {
                            Magnum::Debug{} << "Received Color Image: " << ts << " size: " << mat.size();
                        }
                        std::lock_guard<std::mutex> lock(m_receiverWriteLock);

                        bool frameStored{false};
                        for (Frame &frame: m_receiveQueue) {
                            if (!frame.have_colorImage) {
                                if (frame.timestamp == ts) {
                                    frame.have_colorImage = true;
                                    frame.colorImage = mat;
                                    if (m_debug) {
                                        Magnum::Debug{} << "Completed Frame with colorImage " << ts;
                                    }
                                    frameStored = true;
                                    break;
                                } else {
                                    Magnum::Debug{} << "Have frame non-matching timestamp: " << frame.timestamp << " != " << ts;
                                }
                            }
                        }

                        if (!frameStored) {
                            Frame frame;
                            frame.have_colorImage = true;
                            frame.timestamp = ts;
                            frame.colorImage = mat;
                            if (m_debug) {
                                Magnum::Debug{} << "Created Frame with colorImage " << ts;
                            }
                            m_receiveQueue.push_back(std::move(frame));
                        }
                    }
                }
                p += (nal_end - nal_start);
                sz -= nal_end;
            }
         }
        // wait for next message
        receiveColorFrame();
    });
}


void K4AStreamReceiver::drawEvent() {
    forwardCompletedFrames();

    if (!m_resultQueue.empty()) {
        auto frame = m_resultQueue.front();
        m_resultQueue.pop_front();
        if (m_debug) {
            Magnum::Debug{} << "Received complete frame for rendering: " << frame.timestamp;
        }
        if (frame.is_valid) {
            if ((m_display) && (frame.have_depthImage)) {
                cv::imshow("DepthImage", frame.depthImage);
            }
            if ((m_display) && (frame.have_colorImage)) {
                cv::Mat displImage;
                cv::cvtColor(frame.colorImage, displImage, cv::COLOR_RGBA2BGR);
                cv::imshow("ColorImage", displImage);
                if(cv::waitKey(5) >= 0) {
                    exit();
                };
            }
        }
    }
    GL::defaultFramebuffer.clear(GL::FramebufferClear::Color);

    // render the pointcloud here.

    swapBuffers();
    redraw();
}

void K4AStreamReceiver::forwardCompletedFrames() {

    std::lock_guard<std::mutex> lock(m_receiverWriteLock);
    while ((!m_receiveQueue.empty()) &&
           (m_receiveQueue.front().have_colorImage == m_colorStreamEnabled) &&
           (m_receiveQueue.front().have_depthImage == m_depthStreamEnabled))
    {
        auto frame = m_receiveQueue.front();
        frame.is_valid = true;
        m_resultQueue.push_back(std::move(frame));
        m_receiveQueue.pop_front();
    }

    // cleanup queue remove leftover incomplete frames
    while (m_receiveQueue.size() > m_maxReceiveQueueSize) {
        auto frame = m_receiveQueue.front();
        Magnum::Warning{} << "Dropping incomplete frame: " << frame.timestamp << " color image: " << frame.have_colorImage << " depth image: " << frame.have_depthImage;
        m_receiveQueue.pop_front();
    }
}

MAGNUM_APPLICATION_MAIN(K4AStreamReceiver)
