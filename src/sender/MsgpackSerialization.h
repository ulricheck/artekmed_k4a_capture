//
// Created by narvis on 03.09.19.
//

#ifndef MYAPPLICATION_MSGPACKSERIALIZATION_H
#define MYAPPLICATION_MSGPACKSERIALIZATION_H

#include <Magnum/Math/RectangularMatrix.h>
#include <Magnum/Math/Quaternion.h>

#include <k4a/k4a.hpp>
#include <msgpack.hpp>


// from utCore/Math/CameraIntrinsics.h
enum CalibType {
    UNKNOWN, OPENCV_2_2, OPENCV_3_2, OPENCV_6_2, OPENCV_4_0_FISHEYE
};

namespace msgpack {
    MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS) {
        namespace adaptor {

/*
 * Ubitrack::Math::Pose
 */


//typedef struct _k4a_calibration_extrinsics_t
//{
//    float rotation[9];    /**< 3x3 Rotation matrix stored in row major order */
//    float translation[3]; /**< Translation vector, x,y,z (in millimeters) */
//} k4a_calibration_extrinsics_t;

            template<>
            struct pack<k4a_calibration_extrinsics_t> {
                template<typename Stream>
                msgpack::packer<Stream> &
                operator()(msgpack::packer<Stream> &o, const k4a_calibration_extrinsics_t &v) const {
                    // from float to double
                    std::vector<double> rotation(9);
                    for (int i = 0; i < 9; i++) {
                        rotation.at(i) = static_cast<double>(v.rotation[i]);
                    }
                    std::vector<double> translation(3);
                    for (int i = 0; i < 3; i++) {
                        translation.at(i) = static_cast<double>(v.translation[i]);
                    }

                    // convert matrix to quaternion
                    auto mat = Magnum::Matrix3x3d::from(&rotation[0]);
                    auto q = Magnum::Quaterniond::fromMatrix(mat);
                    std::vector<double> qrot{q.vector().x(), q.vector().y(), q.vector().z(), q.scalar()};

                    o.pack_array(2);
                    o.pack(qrot);
                    o.pack(translation);
                    return o;
                }
            };

            template<>
            struct object_with_zone<k4a_calibration_extrinsics_t> {
                void operator()(msgpack::object::with_zone &o, const k4a_calibration_extrinsics_t &v) const {
                    // from float to double
                    std::vector<double> rotation(9);
                    for (int i = 0; i < 9; i++) {
                        rotation.at(i) = static_cast<double>(v.rotation[i]);
                    }
                    std::vector<double> translation(3);
                    for (int i = 0; i < 3; i++) {
                        translation.at(i) = static_cast<double>(v.translation[i]);
                    }

                    // convert matrix to quaternion
                    auto mat = Magnum::Matrix3x3d::from(&rotation[0]);
                    auto q = Magnum::Quaterniond::fromMatrix(mat);
                    std::vector<double> qrot{q.vector().x(), q.vector().y(), q.vector().z(), q.scalar()};

                    o.type = type::ARRAY;
                    o.via.array.size = 2;
                    o.via.array.ptr = static_cast<msgpack::object *>(
                            o.zone.allocate_align(sizeof(msgpack::object) * o.via.array.size));
                    o.via.array.ptr[0] = msgpack::object(qrot, o.zone);
                    o.via.array.ptr[1] = msgpack::object(translation, o.zone);
                }
            };



/*
 * Ubitrack::Math::CameraIntrinsics<T>
 */

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

            template<>
            struct pack<k4a_calibration_camera_t> {
                template<typename Stream>
                msgpack::packer<Stream> &
                operator()(msgpack::packer<Stream> &o, const k4a_calibration_camera_t &v) const {
                    auto &p = v.intrinsics.parameters;

                    Magnum::Matrix3x3d mat{Magnum::Math::ZeroInit};
                    mat[0][0] = static_cast<double>(p.param.fx);
                    mat[1][1] = static_cast<double>(p.param.fy);
                    mat[2][0] = -static_cast<double>(p.param.cx);
                    mat[2][1] = -static_cast<double>(p.param.cy);
                    mat[2][2] = -1.0;

                    auto width = (std::size_t) v.resolution_width;
                    auto height = (std::size_t) v.resolution_height;
                    std::vector<size_t> dimension{width, height};

                    Magnum::Math::Vector<6, double> radial;
                    radial[0] = static_cast<double>(p.param.k1);
                    radial[1] = static_cast<double>(p.param.k2);
                    radial[2] = static_cast<double>(p.param.k3);
                    radial[3] = static_cast<double>(p.param.k4);
                    radial[4] = static_cast<double>(p.param.k5);
                    radial[5] = static_cast<double>(p.param.k6);

                    Magnum::Math::Vector<2, double> tangential(
                            static_cast<double>(p.param.p1),
                            static_cast<double>(p.param.p2));

                    o.pack_array(6);
                    o.pack(static_cast<int>(OPENCV_6_2));
                    o.pack(dimension);
                    o.pack(mat.toVector());
                    o.pack(6);
                    o.pack(radial);
                    o.pack(tangential);
                    return o;
                }
            };

            template<>
            struct object_with_zone<k4a_calibration_camera_t> {
                void operator()(msgpack::object::with_zone &o, const k4a_calibration_camera_t &v) const {

                    auto &p = v.intrinsics.parameters;

                    Magnum::Matrix3x3d mat{Magnum::Math::ZeroInit};
                    mat[0][0] = static_cast<double>(p.param.fx);
                    mat[1][1] = static_cast<double>(p.param.fy);
                    mat[2][0] = -static_cast<double>(p.param.cx);
                    mat[2][1] = -static_cast<double>(p.param.cy);
                    mat[2][2] = -1.0;

                    auto width = (std::size_t) v.resolution_width;
                    auto height = (std::size_t) v.resolution_height;
                    std::vector<size_t> dimension{width, height};

                    Magnum::Math::Vector<6, double> radial;
                    radial[0] = static_cast<double>(p.param.k1);
                    radial[1] = static_cast<double>(p.param.k2);
                    radial[2] = static_cast<double>(p.param.k3);
                    radial[3] = static_cast<double>(p.param.k4);
                    radial[4] = static_cast<double>(p.param.k5);
                    radial[5] = static_cast<double>(p.param.k6);

                    Magnum::Math::Vector<2, double> tangential(
                            static_cast<double>(p.param.p1),
                            static_cast<double>(p.param.p2));


                    o.type = type::ARRAY;
                    o.via.array.size = 6;
                    o.via.array.ptr = static_cast<msgpack::object *>(
                            o.zone.allocate_align(sizeof(msgpack::object) * o.via.array.size));
                    o.via.array.ptr[0] = msgpack::object(static_cast<int>(OPENCV_6_2), o.zone);
                    o.via.array.ptr[1] = msgpack::object(dimension, o.zone);
                    o.via.array.ptr[2] = msgpack::object(mat.toVector(), o.zone);
                    o.via.array.ptr[3] = msgpack::object(6, o.zone);
                    o.via.array.ptr[4] = msgpack::object(radial, o.zone);
                    o.via.array.ptr[5] = msgpack::object(tangential, o.zone);
                }
            };

        }
    }
}


#endif //MYAPPLICATION_MSGPACKSERIALIZATION_H
