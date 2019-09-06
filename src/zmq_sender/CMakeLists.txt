find_package(Magnum REQUIRED GL)
if(CORRADE_TARGET_UNIX)
    find_package(Magnum REQUIRED WindowlessGlxApplication)
endif()
if(CORRADE_TARGET_WINDOWS)
    find_package(Magnum REQUIRED WindowlessWglApplication)
endif()

find_package(CUDA REQUIRED)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

add_executable(k4a_zmq_sender k4a_zmq_sender.cpp ${CMAKE_SOURCE_DIR}/include/MsgpackSerialization.h)
set_property(TARGET k4a_zmq_sender PROPERTY CXX_STANDARD 11)

target_link_libraries(k4a_zmq_sender PRIVATE
        CONAN_PKG::magnum
        CONAN_PKG::corrade
        CONAN_PKG::opencv
        CONAN_PKG::msgpack
        CONAN_PKG::Boost
        CONAN_PKG::azmq
        CONAN_PKG::kinect-azure-sensor-sdk
        CONAN_PKG::nvenc_rtsp
)
target_include_directories(k4a_zmq_sender
        PRIVATE ${CUDA_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/include
        )
