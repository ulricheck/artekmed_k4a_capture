find_package(Magnum REQUIRED GL GlfwApplication)
find_package(CUDA REQUIRED)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

add_executable(k4a_zmq_receiver k4a_zmq_receiver.cpp)
set_property(TARGET k4a_zmq_receiver PROPERTY CXX_STANDARD 11)

target_link_libraries(k4a_zmq_receiver PRIVATE
        CONAN_PKG::magnum
        CONAN_PKG::corrade
        CONAN_PKG::opencv
        CONAN_PKG::Boost
        CONAN_PKG::nvpipe
        CONAN_PKG::h264bitstream
        CONAN_PKG::azmq
)
target_include_directories(k4a_zmq_receiver
        PRIVATE ${CUDA_INCLUDE_DIRS}
        )
