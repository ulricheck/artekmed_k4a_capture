
#include <Corrade/configure.h>
#include <Corrade/Utility/Arguments.h>


#ifdef CORRADE_TARGET_UNIX
#include <Magnum/Platform/WindowlessGlxApplication.h>
#endif
#ifdef CORRADE_TARGET_WINDOWS
#include <Magnum/Platform/WindowlessWglApplication.h>
#endif


#include <Magnum/GL/PixelFormat.h>


#include <atomic>
#include <memory>
#include <vector>
#include <chrono>
#include <sstream>
#include <thread>

#include <k4a/k4a.hpp>
#include <opencv2/opencv.hpp>
#include <NvPipe.h>

#include "../../include/MsgpackSerialization.h"

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <azmq/socket.hpp>


using namespace Magnum;


class K4AStreamSender : public Platform::WindowlessApplication {
public:
    explicit K4AStreamSender(const Arguments &arguments);
    int exec() override;

private:

//    void receivePushMessage();
//    void handlePullRequest();

    void watchdogTimer();

    k4a::device m_dev;
    k4a_device_configuration_t m_dev_config{K4A_DEVICE_CONFIG_INIT_DISABLE_ALL};

    NvPipe* m_colorStreamEncoder{nullptr};
    NvPipe* m_depthStreamEncoder{nullptr};
    std::vector<uint8_t> m_colorBuffer;
    std::vector<uint8_t> m_depthBuffer;
    std::atomic<unsigned int> m_colorBufferIndex{0};
    std::atomic<unsigned int> m_depthBufferIndex{0};

    std::shared_ptr<std::thread > m_NetworkThread;
    std::shared_ptr<boost::asio::io_service> m_ioservice;
    std::shared_ptr<boost::asio::deadline_timer> m_ioserviceKeepAlive;
    std::shared_ptr<azmq::socket> m_zmq_pub_socket;
    std::shared_ptr<azmq::socket> m_zmq_depth_stream_socket;
    std::shared_ptr<azmq::socket> m_zmq_color_stream_socket;

    std::string m_cameraSerialNumber{""};
    std::string m_ipAddress{"127.0.0.1"};
    unsigned int m_dstDepthPort{55555};
    unsigned int m_dstColorPort{55556};
    unsigned int m_dstConfigPort{55557};
    unsigned int m_bitrate{50};
    unsigned int m_framerate{30};

    bool m_debug{false};

    bool m_depthStreamEnabled{true};
    bool m_colorStreamEnabled{true};

};

K4AStreamSender::K4AStreamSender(const Arguments &arguments) : Platform::WindowlessApplication{arguments} {
    Magnum::Utility::Arguments args;
    args.addOption("serialnumber", "").setHelp("serialnumber", "Camera serial number")
        .addOption("ip", "127.0.0.1").setHelp("ip", "IP address to bind to")
        .addOption("depthport", "55555").setHelp("depthport", "Port to bind depth stream")
        .addOption("colorport", "55556").setHelp("colorport", "Port to bind color stream")
        .addOption("configport", "55557").setHelp("configport", "Port to bind config publisher")
        .addBooleanOption("debug").setHelp("debug", "Show debug windows")
        .addBooleanOption("nodepth").setHelp("nodepth", "Skip depth stream")
        .addBooleanOption("nocolor").setHelp("nocolor", "Skip color stream")
        .addSkippedPrefix("magnum", "engine-specific options");

    args.parse(arguments.argc, arguments.argv);

    m_cameraSerialNumber = args.value("serialnumber");
    m_ipAddress = args.value("ip");
    m_dstDepthPort = args.value<Magnum::Int>("depthport");
    m_dstColorPort = args.value<Magnum::Int>("colorport");
    m_dstConfigPort = args.value<Magnum::Int>("configport");
    m_debug = args.isSet("debug");
    m_depthStreamEnabled = !args.isSet("nodepth");
    m_colorStreamEnabled = !args.isSet("nocolor");

    // start communication services
    Magnum::Debug{} << "Create IOService";
    m_ioservice = std::make_shared<boost::asio::io_service>();
    m_ioserviceKeepAlive = std::make_shared<boost::asio::deadline_timer>(*m_ioservice);
    watchdogTimer();
    m_NetworkThread = std::make_shared< std::thread >( [&]{ m_ioservice->run(); } );
    m_zmq_pub_socket = std::make_shared< azmq::socket >( *m_ioservice, ZMQ_PUB );
    m_zmq_depth_stream_socket = std::make_shared< azmq::socket >( *m_ioservice, ZMQ_PUB );
    m_zmq_color_stream_socket = std::make_shared< azmq::socket >( *m_ioservice, ZMQ_PUB );

    // Check for devices
    //
    const uint32_t deviceCount = k4a::device::get_installed_count();
    if (deviceCount == 0) {
        throw std::runtime_error("No Azure Kinect devices detected!");
    }


    bool found_device = false;
    Magnum::Debug{} << "Found " << int(deviceCount) << " connected devices:";

    if (m_cameraSerialNumber.empty()) {
        m_dev = k4a::device::open(0);
        found_device = true;
    } else {
        for (uint8_t deviceIndex = 0; deviceIndex < deviceCount; deviceIndex++)
        {
            try {
                auto candidate = k4a::device::open(deviceIndex);
                auto serialnr = candidate.get_serialnum();
                if (m_cameraSerialNumber == serialnr) {
                    Magnum::Debug{} << "Found Azure Kinect with Serial: " << serialnr;
                    found_device = true;
                    m_dev = std::move(candidate);
                    break;
                }
                else {
                    Magnum::Debug{} << "Found Azure Kinect with NON MATCHING Serial: " << serialnr;
                }
            } catch (std::exception &e) {
                Magnum::Error{} << "Error opening device: " << e.what();
            }
        }
    }

    if (!found_device) {
        throw std::runtime_error("No Azure Kinect device with matching serial number detected!");
    }


    // Start the device
    //
    Magnum::Debug{} << "Start opening K4A device...";
    m_dev_config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    m_dev_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    m_dev_config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    m_dev_config.color_resolution = K4A_COLOR_RESOLUTION_1536P;

    // This means that we'll only get captures that have both color and
    // depth images, so we don't need to check if the capture contains
    // a particular type of image.
    //
    m_dev_config.synchronized_images_only = true;
    m_dev.start_cameras(&m_dev_config);

    Magnum::Debug{} << "Finished opening K4A device.";


}

void K4AStreamSender::watchdogTimer() {
    m_ioserviceKeepAlive->expires_from_now(boost::posix_time::seconds(1));
    m_ioserviceKeepAlive->async_wait(boost::bind(&K4AStreamSender::watchdogTimer, this));
}

int K4AStreamSender::exec() {
    int process_result{0};
    Magnum::Debug{} << "Start Kinect Streaming Client";

    // start zmq socket
    std::string zmq_bind_config_str = "tcp://" + m_ipAddress + ":" + std::to_string(m_dstConfigPort);
    Magnum::Debug{} << "ZMQ Publisher for config: " << zmq_bind_config_str;

    try {
        m_zmq_pub_socket->bind(zmq_bind_config_str);
    } catch (boost::system::system_error &e) {
        std::ostringstream log;
        Magnum::Error{} << "Error initializing ZMQNetwork: " << zmq_bind_config_str;
        Magnum::Error{} << e.what();
        return 1;
    }

    if (m_depthStreamEnabled) {
        std::string zmq_bind_depth_str = "tcp://" + m_ipAddress + ":" + std::to_string(m_dstDepthPort);
        Magnum::Debug{} << "ZMQ Publisher for depth: " << zmq_bind_depth_str;
        try {
            m_zmq_depth_stream_socket->bind(zmq_bind_depth_str);
        } catch (boost::system::system_error &e) {
            std::ostringstream log;
            Magnum::Error{} << "Error initializing ZMQNetwork: " << zmq_bind_config_str;
            Magnum::Error{} << e.what();
            return 1;
        }
    }

    if (m_colorStreamEnabled) {
        std::string zmq_bind_color_str = "tcp://" + m_ipAddress + ":" + std::to_string(m_dstColorPort);
        Magnum::Debug{} << "ZMQ Publisher for color: " << zmq_bind_color_str;
        try {
            m_zmq_color_stream_socket->bind(zmq_bind_color_str);
        } catch (boost::system::system_error &e) {
            std::ostringstream log;
            Magnum::Error{} << "Error initializing ZMQNetwork: " << zmq_bind_config_str;
            Magnum::Error{} << e.what();
            return 1;
        }
    }

    // retrieve configuration
    auto calibration = m_dev.get_calibration(m_dev_config.depth_mode, m_dev_config.color_resolution);

    // start streaming
    try{
        if (m_debug) {
            cv::namedWindow("ColorImage",cv::WINDOW_NORMAL);
            cv::namedWindow("DepthImage",cv::WINDOW_NORMAL);
        }
        cv::Mat colorImage;
        cv::Mat depthImage;

        k4a::capture capture;
        size_t frame_number{0};

        for(;;) {
            if (m_dev.get_capture(&capture, std::chrono::milliseconds(50)))
            {
                bool do_send_config = false;
                if ((frame_number % m_framerate) == 0) {
                    do_send_config = true;
                    Magnum::Debug{} << "Send config " << frame_number;
                }

                // Depth Image Stream
                if (m_depthStreamEnabled) {

                    const k4a::image inputImage = capture.get_depth_image();
                    unsigned long ts = inputImage.get_system_timestamp().count();

                    if (do_send_config) {
                        // depth model
                        {
                            std::string name{"depth_model"};
                            std::ostringstream stream;
                            msgpack::packer<std::ostringstream> pk(&stream);
                            pk.pack(name);
                            pk.pack(static_cast<int>(MeasurementType::CameraIntrinsics));

                            // measurement
                            pk.pack_array(2);
                            pk.pack(static_cast<unsigned long long>(ts));
                            pk.pack(calibration.depth_camera_calibration);

                            pk.pack(ts);

                            azmq::message message(boost::asio::buffer(stream.str().data(), stream.str().size()));
                            m_zmq_pub_socket->async_send(message, [&] (boost::system::error_code const& ec, size_t ) {
                                if (ec != boost::system::error_code()) {
                                    Magnum::Error{} << "Error sending message on ZMQSink " << name << " - " << ec.message();
                                }
                            });
                        }

                        // depth2color transform
                        {
                            std::string name{"depth2color"};
                            std::ostringstream stream;
                            msgpack::packer<std::ostringstream> pk(&stream);
                            pk.pack(name);
                            pk.pack(static_cast<int>(MeasurementType::Pose));

                            // measurement
                            pk.pack_array(2);
                            pk.pack(static_cast<unsigned long long>(ts));
                            pk.pack(calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR]);

                            pk.pack(ts);

                            azmq::message message(boost::asio::buffer(stream.str().data(), stream.str().size()));
                            m_zmq_pub_socket->async_send(message, [&] (boost::system::error_code const& ec, size_t ) {
                                if (ec != boost::system::error_code()) {
                                    Magnum::Error{} << "Error sending message on ZMQSink " << name << " - " << ec.message();
                                }
                            });
                        }
                    }

                    NvPipe_Format nvpFormat{NVPIPE_UINT16};
                    int w = inputImage.get_width_pixels();
                    int h = inputImage.get_height_pixels();
                    size_t bytesPerPixel = sizeof(uint16_t);
                    uint64_t dataSize = w * h;

                    switch (inputImage.get_format()) {
                        case K4A_IMAGE_FORMAT_DEPTH16:
                            depthImage = cv::Mat(cv::Size(w, h), CV_16UC1, const_cast<void*>(static_cast<const void*>(inputImage.get_buffer())), cv::Mat::AUTO_STEP);
                            nvpFormat = NVPIPE_UINT16;
                            bytesPerPixel = sizeof(uint16_t);
                            break;

                        case K4A_IMAGE_FORMAT_IR16:
                        case K4A_IMAGE_FORMAT_COLOR_BGRA32:
                        case K4A_IMAGE_FORMAT_COLOR_MJPG:
                        case K4A_IMAGE_FORMAT_COLOR_NV12:
                        case K4A_IMAGE_FORMAT_COLOR_YUY2:
//                case K4A_IMAGE_FORMAT_CUSTOM8:
//                case K4A_IMAGE_FORMAT_CUSTOM16:
                        case K4A_IMAGE_FORMAT_CUSTOM:
                        default:
                            throw std::runtime_error("kinect4azure depth frame format is (currently) not supported!");
                    }
                    uint64_t dataPitch = w * bytesPerPixel;

                    if (!m_depthStreamEncoder) {
                        m_depthStreamEncoder = NvPipe_CreateEncoder(nvpFormat, NVPIPE_H264, NVPIPE_LOSSLESS,
                                                                    m_bitrate * 1000 * 1000, m_framerate, w, h);
                        if (!m_depthStreamEncoder) {
                            Magnum::Error{} << "Failed to create depth encoder: " << NvPipe_GetError(nullptr);
                            return 1;
                        }
                        m_depthBuffer.resize(dataSize * bytesPerPixel);
                    }

                    bool haveNewFrame{false};
                    uint64_t encodedBufferSize{0};
                    if (m_depthBufferIndex.fetch_add(1, std::memory_order_relaxed) == 0) {
                        std::atomic_thread_fence(std::memory_order_acquire);

                        encodedBufferSize = NvPipe_Encode(m_depthStreamEncoder, depthImage.data, dataPitch, m_depthBuffer.data(), m_depthBuffer.size(), w, h, false);
                        if (encodedBufferSize == 0) {
                            Magnum::Error{} << "Encode error depth: " << NvPipe_GetError(m_depthStreamEncoder);
                            return 1;
                        }
                        haveNewFrame = true;
                    } else {
                        Magnum::Debug{} << "Need double buffering to avoid local copies";
                    }

                    if (haveNewFrame) {
                        azmq::message message(azmq::nocopy, boost::asio::buffer(&m_depthBuffer[0], encodedBufferSize), static_cast<void*>(&m_depthBufferIndex), [](void *, void *hint){
                            if (hint != nullptr) {
                                auto b = static_cast<std::atomic<unsigned int>*>(hint);
                                if ((*b).fetch_sub(1, std::memory_order_release) == 1) {
                                    std::atomic_thread_fence(std::memory_order_acquire);
                                    // all good !!
                                } else {
                                    Magnum::Debug{} << "Need double buffering to avoid local copies";
                                }
                            }
                        });

                        // multipart message (header/body) ??
                        // flags = ZMQ_SNDMORE
                        m_zmq_depth_stream_socket->async_send(message, [&] (boost::system::error_code const& ec, size_t ) {
                            if (ec != boost::system::error_code()) {
                                Magnum::Error{} << "Error sending depth buffer - " << ec.message();
                            }
                        });
                    }

                    if (m_debug) {
                        Magnum::Debug{} << "got depth image: " << ts << "elemSize: " << depthImage.elemSize()  << "elemSize1: " << depthImage.elemSize1();
                        cv::imshow("DepthImage", depthImage);
                    }
                }

                if (m_colorStreamEnabled) {

                    const k4a::image inputImage = capture.get_color_image();
                    unsigned long ts = inputImage.get_system_timestamp().count();

                    if (do_send_config) {
                        // color model
                        {
                            std::string name{"color_model"};
                            std::ostringstream stream;
                            msgpack::packer<std::ostringstream> pk(&stream);
                            pk.pack(name);
                            pk.pack(static_cast<int>(MeasurementType::CameraIntrinsics));

                            // measurement
                            pk.pack_array(2);
                            pk.pack(static_cast<unsigned long long>(ts));
                            pk.pack(calibration.color_camera_calibration);

                            pk.pack(ts);

                            azmq::message message(boost::asio::buffer(stream.str().data(), stream.str().size()));
                            m_zmq_pub_socket->async_send(message, [&] (boost::system::error_code const& ec, size_t ) {
                                if (ec != boost::system::error_code()) {
                                    Magnum::Error{} << "Error sending message on ZMQSink " << name << " - " << ec.message();
                                }
                            });
                        }
                    }
                    NvPipe_Format nvpFormat{NVPIPE_RGBA32};
                    int w = inputImage.get_width_pixels();
                    int h = inputImage.get_height_pixels();
                    size_t bytesPerPixel = 4 * sizeof(uint8_t);
                    uint64_t dataSize = w * h;

                    switch (inputImage.get_format()) {
                        case K4A_IMAGE_FORMAT_COLOR_BGRA32:
                        {
                            auto tmpImage = cv::Mat(cv::Size(w, h), CV_8UC4, const_cast<void*>(static_cast<const void*>(inputImage.get_buffer())), cv::Mat::AUTO_STEP);
                            cv::cvtColor(tmpImage, colorImage, cv::COLOR_BGRA2RGBA);
                            nvpFormat = NVPIPE_RGBA32;
                            bytesPerPixel = 4 * sizeof(uint8_t);
                        }
                            break;

                        case K4A_IMAGE_FORMAT_DEPTH16:
                        case K4A_IMAGE_FORMAT_IR16:
                        case K4A_IMAGE_FORMAT_COLOR_MJPG:
                        case K4A_IMAGE_FORMAT_COLOR_NV12:
                        case K4A_IMAGE_FORMAT_COLOR_YUY2:
//                case K4A_IMAGE_FORMAT_CUSTOM8:
//                case K4A_IMAGE_FORMAT_CUSTOM16:
                        case K4A_IMAGE_FORMAT_CUSTOM:
                        default:
                            throw std::runtime_error("kinect4azure color frame format is (currently) not supported!");
                    }
                    uint64_t dataPitch = w * bytesPerPixel;

                    if (!m_colorStreamEncoder) {
                        m_colorStreamEncoder = NvPipe_CreateEncoder(nvpFormat, NVPIPE_H264, NVPIPE_LOSSLESS,
                                                                    m_bitrate * 1000 * 1000, m_framerate, w, h);
                        if (!m_colorStreamEncoder) {
                            Magnum::Error{} << "Failed to create depth encoder: " << NvPipe_GetError(nullptr);
                            return 1;
                        }
                        m_colorBuffer.resize(dataSize * 4 * sizeof(uint8_t));
                    }

                    bool haveNewFrame{false};
                    uint64_t encodedBufferSize{0};
                    if (m_colorBufferIndex.fetch_add(1, std::memory_order_relaxed) == 0) {
                        std::atomic_thread_fence(std::memory_order_acquire);

                        encodedBufferSize = NvPipe_Encode(m_colorStreamEncoder, colorImage.data, dataPitch, m_colorBuffer.data(), m_colorBuffer.size(), w, h, false);
                        if (encodedBufferSize == 0) {
                            Magnum::Error{} << "Encode error color: " << NvPipe_GetError(m_colorStreamEncoder);
                            return 1;
                        }
                        haveNewFrame = true;
                    } else {
                        Magnum::Debug{} << "Need double buffering to avoid local copies";
                    }

                    if (haveNewFrame) {
                        azmq::message message(azmq::nocopy, boost::asio::buffer(&m_colorBuffer[0], encodedBufferSize), static_cast<void*>(&m_colorBufferIndex), [](void *, void *hint){
                            if (hint != nullptr) {
                                auto b = static_cast<std::atomic<unsigned int>*>(hint);
                                if ((*b).fetch_sub(1, std::memory_order_release) == 1) {
                                    std::atomic_thread_fence(std::memory_order_acquire);
                                    // all good !!
                                } else {
                                    Magnum::Debug{} << "Need double buffering to avoid local copies";
                                }
                            }
                        });

                        m_zmq_color_stream_socket->async_send(message, [&] (boost::system::error_code const& ec, size_t ) {
                            if (ec != boost::system::error_code()) {
                                Magnum::Error{} << "Error sending color buffer - " << ec.message();
                            }
                        });
                    }


                    if (m_debug) {
                        Magnum::Debug{} << "got color images: " << ts << "elemSize: " << colorImage.elemSize()  << "elemSize1: " << colorImage.elemSize1();
                        cv::Mat rgb;
                        cv::cvtColor(colorImage, rgb, cv::COLOR_RGBA2BGR);
                        cv::imshow("ColorImage", rgb);
                        if(cv::waitKey(5) >= 0) break;

                    }
                }

            } else {
                Magnum::Debug{} << "No Frame received for 50ms.";
            }

            // increase frame counter
            frame_number++;
        }
    } catch (std::exception &e) {
        Magnum::Debug{} << "Exception in main-loop: " << e.what();
        process_result = 1;
    }

    // tear down
    try{
        m_ioservice->stop();
        m_NetworkThread->join();
        m_NetworkThread.reset();
        m_ioservice.reset();
        m_dev.close();
    } catch (std::exception &e) {
        Magnum::Debug{} << "Exception while tearing down: " << e.what();
        process_result = 1;
    }

    return process_result;
}

MAGNUM_WINDOWLESSAPPLICATION_MAIN(K4AStreamSender)
