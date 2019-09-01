find_package(Magnum REQUIRED GL GlfwApplication)
find_package(CUDA REQUIRED)

set_directory_properties(PROPERTIES CORRADE_USE_PEDANTIC_FLAGS ON)

add_executable(k4acapture K4ACapture.cpp)
target_link_libraries(k4acapture PRIVATE
        CONAN_PKG::magnum
        CONAN_PKG::corrade
        CONAN_PKG::opencv
        CONAN_PKG::nvenc_rtsp
        CONAN_PKG::kinect-azure-sensor-sdk
)
target_include_directories(k4acapture
        PRIVATE ${CUDA_INCLUDE_DIRS}
        )
