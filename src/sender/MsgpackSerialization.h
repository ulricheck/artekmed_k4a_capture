//
// Created by narvis on 03.09.19.
//

#ifndef MYAPPLICATION_MSGPACKSERIALIZATION_H
#define MYAPPLICATION_MSGPACKSERIALIZATION_H

#include <Magnum/Math/RectangularMatrix.h>
#include <Magnum/Math/Quaternion.h>
#include <k4a/k4a.hpp>
#include <msgpack.hpp>


//typedef union
//{
//    /** individual parameter or array representation of intrinsic model. */
//    struct _param
//    {
//        float cx;            /**< Principal point in image, x */
//        float cy;            /**< Principal point in image, y */
//        float fx;            /**< Focal length x */
//        float fy;            /**< Focal length y */
//        float k1;            /**< k1 radial distortion coefficient */
//        float k2;            /**< k2 radial distortion coefficient */
//        float k3;            /**< k3 radial distortion coefficient */
//        float k4;            /**< k4 radial distortion coefficient */
//        float k5;            /**< k5 radial distortion coefficient */
//        float k6;            /**< k6 radial distortion coefficient */
//        float codx;          /**< Center of distortion in Z=1 plane, x (only used for Rational6KT) */
//        float cody;          /**< Center of distortion in Z=1 plane, y (only used for Rational6KT) */
//        float p2;            /**< Tangential distortion coefficient 2 */
//        float p1;            /**< Tangential distortion coefficient 1 */
//        float metric_radius; /**< Metric radius */
//    } param;                 /**< Individual parameter representation of intrinsic model */
//    float v[15];             /**< Array representation of intrinsic model parameters */
//} k4a_calibration_intrinsic_parameters_t;

//
//void serialize_camera_intrinsics(msgpack::object::with_zone& o, const k4a_calibration_intrinsic_parameters_t& p) {
//
//    Math::Matrix< double, 3, 3 > intrinsicMatrix = Math::Matrix3x3d::identity();
//    intrinsicMatrix(0, 0) = (double)p.param.fx;
//    intrinsicMatrix(1, 1) = (double)p.param.fy;
//    intrinsicMatrix(0, 2) = -(double)p.param.cx;
//    intrinsicMatrix(1, 2) = -(double)p.param.cy;
//    intrinsicMatrix(2, 2) = -1.0;
//
//    Math::Vector< double, 6 > radial;
//    radial(0) = (double)p.param.k1;
//    radial(1) = (double)p.param.k2;
//    radial(2) = (double)p.param.k3;
//    radial(3) = (double)p.param.k4;
//    radial(4) = (double)p.param.k5;
//    radial(5) = (double)p.param.k6;
//
//    Math::Vector< double, 2 > tangential(
//            (double)p.param.p1,
//            (double)p.param.p2);
//
//    auto width = (std::size_t)k4a_calib.resolution_width;
//    auto height = (std::size_t)k4a_calib.resolution_height;
//
//
//}


//typedef struct _k4a_calibration_extrinsics_t
//{
//    float rotation[9];    /**< 3x3 Rotation matrix stored in row major order */
//    float translation[3]; /**< Translation vector, x,y,z (in millimeters) */
//} k4a_calibration_extrinsics_t;



namespace msgpack {
    MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS) {
            namespace adaptor {

            }
    }
}


#endif //MYAPPLICATION_MSGPACKSERIALIZATION_H
