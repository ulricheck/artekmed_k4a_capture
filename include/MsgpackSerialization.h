//
// Created by narvis on 03.09.19.
//

#ifndef MYAPPLICATION_MSGPACKSERIALIZATION_H
#define MYAPPLICATION_MSGPACKSERIALIZATION_H

#include <Magnum/Math/RectangularMatrix.h>
#include <Magnum/Math/Quaternion.h>
#include <Magnum/Math/Vector.h>
#include <Magnum/Math/RectangularMatrix.h>
#include <Magnum/Math/Matrix3.h>

#include <k4a/k4a.hpp>
#include <msgpack.hpp>


// from utCore/Math/CameraIntrinsics.h
enum CalibType {
    UNKNOWN, OPENCV_2_2, OPENCV_3_2, OPENCV_6_2, OPENCV_4_0_FISHEYE
};

// from utCore/Measurement/MeasurementTraits.h
enum class MeasurementType {
    Undefined = 0,
    ScalarInt,
    ScalarDouble,
    ScalarUnsignedLong,
    Vector2,
    Vector3,
    Vector4,
    Vector8,
    Quaternion,
    Matrix3x3,
    Matrix3x4,
    Matrix4x4,
    Pose,
    ErrorPose,
    ErrorVector2,
    ErrorVector3,
    RotationVelocity,
    CameraIntrinsics,
    Image // Forward declaration as we cannot extend an enumeration later
};


namespace msgpack {
    MSGPACK_API_VERSION_NAMESPACE(MSGPACK_DEFAULT_API_NS) {
        namespace adaptor {



/*
 * Ubitrack::Math::Vector<T, N>
 */
#if !defined(MSGPACK_USE_CPP03)

            template<std::size_t N, typename T>
            struct as<Magnum::Math::Vector<N, T>, typename std::enable_if<msgpack::has_as<T>::value>::type> {
                Magnum::Math::Vector<N, T> operator()(msgpack::object const &o) const {
                    if (o.type != msgpack::type::ARRAY) throw msgpack::type_error();
                    std::size_t num_elements = o.via.array.size;
                    if (N > 0) {
                        if (num_elements != N) throw msgpack::type_error();
                    }
                    Magnum::Math::Vector<N, T> result{Magnum::Math::ZeroInit};
                    for (std::size_t i = 0; i < num_elements; ++i) {
                        result[i] = o.via.array.ptr[i].as<T>();
                    }
                    return result;
                }
            };

#endif // !defined(MSGPACK_USE_CPP03)

            template<std::size_t N, typename T>
            struct convert<Magnum::Math::Vector<N, T> > {
                msgpack::object const &operator()(msgpack::object const &o, Magnum::Math::Vector<N, T> &v) const {
                    if (o.type != msgpack::type::ARRAY) throw msgpack::type_error();
                    std::size_t num_elements = o.via.array.size;
                    if (N > 0) {
                        if (num_elements != N) throw msgpack::type_error();
                    }
                    for (std::size_t i = 0; i < num_elements; ++i) {
                        msgpack::adaptor::convert<T>()(o.via.array.ptr[i], v[i]);
                        //v(i) = o.via.array.ptr[i].as<T>();
                    }
                    return o;
                }
            };

            template<std::size_t N, typename T>
            struct pack<Magnum::Math::Vector<N, T> > {
                template<typename Stream>
                msgpack::packer<Stream> &
                operator()(msgpack::packer<Stream> &o, const Magnum::Math::Vector<N, T> &v) const {
                    std::size_t num_elements = N;
                    o.pack_array(static_cast<uint32_t>(num_elements));
                    for (std::size_t i = 0; i < num_elements; ++i) {
                        o.pack(v[i]);
                    }
                    return o;
                }
            };

            template<std::size_t N, typename T>
            struct object_with_zone<Magnum::Math::Vector<N, T> > {
                void operator()(msgpack::object::with_zone &o, const Magnum::Math::Vector<N, T> &v) const {
                    std::size_t num_elements = N;
                    o.type = type::ARRAY;
                    o.via.array.size = static_cast<uint32_t>(num_elements);
                    o.via.array.ptr = static_cast<msgpack::object *>(
                            o.zone.allocate_align(sizeof(msgpack::object) * o.via.array.size));
                    for (std::size_t i = 0; i < num_elements; ++i) {
                        o.via.array.ptr[0] = msgpack::object(v[i], o.zone);
                    }
                }
            };


/*
 * Ubitrack::Math::Matrix<T, M, N>
 */
#if !defined(MSGPACK_USE_CPP03)

            template<std::size_t M, std::size_t N, typename T>
            struct as<Magnum::Math::RectangularMatrix<M, N, T>, typename std::enable_if<msgpack::has_as<T>::value>::type> {
                Magnum::Math::RectangularMatrix<M, N, T> operator()(msgpack::object const &o) const {
                    if (o.type != msgpack::type::ARRAY) throw msgpack::type_error();
                    std::size_t num_elements = o.via.array.size;
                    if ((M > 0) && (N > 0)) {
                        if (num_elements != M * N) throw msgpack::type_error();
                    } else {
                        // cannot unpack dynamically sized matrix without knowing dimensions ...
                        throw msgpack::type_error();
                    }
                    Magnum::Math::RectangularMatrix<M, N, T> result{Magnum::Math::ZeroInit};
                    for (std::size_t i = 0; i < M; ++i) {
                        for (std::size_t j = 0; j < N; ++j) {
                            std::size_t idx = i * M + j;
                            // implicit conversion row-major to column-major
                            result[j][i] = o.via.array.ptr[idx].as<T>();
                        }
                    }
                    return result;
                }
            };

#endif // !defined(MSGPACK_USE_CPP03)

            template<std::size_t M, std::size_t N, typename T>
            struct convert<Magnum::Math::RectangularMatrix<M, N, T> > {
                msgpack::object const &
                operator()(msgpack::object const &o, Magnum::Math::RectangularMatrix<M, N, T> &v) const {
                    if (o.type != msgpack::type::ARRAY) throw msgpack::type_error();
                    std::size_t num_elements = o.via.array.size;
                    if ((M > 0) && (N > 0)) {
                        if (num_elements != M * N) throw msgpack::type_error();
                    } else {
                        // cannot unpack dynamically sized matrix without knowing dimensions ...
                        throw msgpack::type_error();
                    }
                    for (std::size_t i = 0; i < M; ++i) {
                        for (std::size_t j = 0; j < N; ++j) {
                            std::size_t idx = i * M + j;
                            // implicit conversion row-major to column-major
                            msgpack::adaptor::convert<T>()(o.via.array.ptr[idx], v[j][i]);
                            //v(i,j) = o.via.array.ptr[idx].as<T>();
                        }
                    }
                    return o;
                }
            };

            template<std::size_t M, std::size_t N, typename T>
            struct pack<Magnum::Math::RectangularMatrix<M, N, T> > {
                template<typename Stream>
                msgpack::packer<Stream> &
                operator()(msgpack::packer<Stream> &o, const Magnum::Math::RectangularMatrix<M, N, T> &v) const {
                    std::size_t num_elements = M * N;
                    if (num_elements == 0) {
                        // cannot pack dynamically sized matrix without also serializing dimensions ...
                        throw msgpack::type_error();
                    }
                    o.pack_array(static_cast<uint32_t>(num_elements));
                    for (std::size_t i = 0; i < M; ++i) {
                        for (std::size_t j = 0; j < N; ++j) {
//                        std::size_t idx = i * M + j;
                            // implicit conversion column-major to row-major
                            o.pack(v[j][i]);
                        }
                    }
                    return o;
                }
            };

            template<std::size_t M, std::size_t N, typename T>
            struct object_with_zone<Magnum::Math::RectangularMatrix<M, N, T> > {
                void
                operator()(msgpack::object::with_zone &o, const Magnum::Math::RectangularMatrix<M, N, T> &v) const {
                    std::size_t num_elements = M * N;
                    if (num_elements == 0) {
                        // cannot pack dynamically sized matrix without also serializing dimensions ...
                        throw msgpack::type_error();
                    }
                    o.type = type::ARRAY;
                    o.via.array.size = static_cast<uint32_t>(num_elements);
                    o.via.array.ptr = static_cast<msgpack::object *>(
                            o.zone.allocate_align(sizeof(msgpack::object) * o.via.array.size));
                    for (std::size_t i = 0; i < M; ++i) {
                        for (std::size_t j = 0; j < N; ++j) {
                            std::size_t idx = i * M + j;
                            // implicit conversion column-major to row-major
                            o.via.array.ptr[idx] = msgpack::object(v[j][i], o.zone);
                        }
                    }
                }
            };




/*
 * Ubitrack::Math::Quaternion
 */
#if !defined(MSGPACK_USE_CPP03)

            template<typename T>
            struct as<Magnum::Math::Quaternion<T> > {
                Magnum::Math::Quaternion<T> operator()(msgpack::object const &o) const {
                    Magnum::Math::Vector<3, T> vec{Magnum::Math::ZeroInit};
                    vec[0] = o.via.array.ptr[0].as<T>();
                    vec[1] = o.via.array.ptr[1].as<T>();
                    vec[2] = o.via.array.ptr[2].as<T>();
                    T scalar = o.via.array.ptr[3].as<T>();
                    return Magnum::Math::Quaternion<T>(vec, scalar);;
                }
            };

#endif

            template<typename T>
            struct convert<Magnum::Math::Quaternion<T> > {
                msgpack::object const &operator()(msgpack::object const &o, Magnum::Math::Quaternion<T> &v) const {

                    Magnum::Math::Vector<3, T> vec{Magnum::Math::ZeroInit};
                    msgpack::adaptor::convert<T>()(o.via.array.ptr[0], vec[0]);
                    msgpack::adaptor::convert<T>()(o.via.array.ptr[1], vec[1]);
                    msgpack::adaptor::convert<T>()(o.via.array.ptr[2], vec[2]);
                    T scalar;
                    msgpack::adaptor::convert<T>()(o.via.array.ptr[3], scalar);
                    v = Magnum::Math::Quaternion<T>(vec, scalar);
                    return o;
                }
            };

            template<typename T>
            struct pack<Magnum::Math::Quaternion<T> > {
                template<typename Stream>
                msgpack::packer<Stream> &
                operator()(msgpack::packer<Stream> &o, const Magnum::Math::Quaternion<T> &v) const {
                    Magnum::Math::Vector<4, T> vec{Magnum::Math::ZeroInit};
                    vec[0] = v.vector().x();
                    vec[1] = v.vector().y();
                    vec[2] = v.vector().z();
                    vec[3] = v.scalar();
                    o.pack(vec);
                    return o;
                }
            };

            template<typename T>
            struct object_with_zone<Magnum::Math::Quaternion<T> > {
                void operator()(msgpack::object::with_zone &o, const Magnum::Math::Quaternion<T> &v) const {
                    o.type = type::ARRAY;
                    o.via.array.size = 4;
                    o.via.array.ptr = static_cast<msgpack::object *>(
                            o.zone.allocate_align(sizeof(msgpack::object) * o.via.array.size));
                    o.via.array.ptr[0] = msgpack::object(v.vector().x(), o.zone);
                    o.via.array.ptr[1] = msgpack::object(v.vector().y(), o.zone);
                    o.via.array.ptr[2] = msgpack::object(v.vector().z(), o.zone);
                    o.via.array.ptr[3] = msgpack::object(v.scalar(), o.zone);
                }
            };




/*
 * Ubitrack::Math::Pose (pack only)
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
                    // convert matrix to quaternion
                    Magnum::Matrix3x3 rotation{Magnum::Math::ZeroInit};
                    rotation[0][0] = v.rotation[0];
                    rotation[0][1] = v.rotation[1];
                    rotation[0][2] = v.rotation[2];
                    rotation[1][0] = v.rotation[3];
                    rotation[1][1] = v.rotation[4];
                    rotation[1][2] = v.rotation[5];
                    rotation[2][0] = v.rotation[6];
                    rotation[2][1] = v.rotation[7];
                    rotation[2][2] = v.rotation[8];

                    auto q = Magnum::Quaternion::fromMatrix(rotation).normalized();

                    // from float to double
                    Magnum::Math::Vector<3, double> translation{Magnum::Math::ZeroInit};
                    for (int i = 0; i < 3; i++) {
                        translation[i] = static_cast<double>(v.translation[i]);
                    }

                    auto qd = Magnum::Quaterniond{{static_cast<double>(q.vector().x()),
                                                          static_cast<double>(q.vector().y()),
                                                          static_cast<double>(q.vector().z())},
                                                  static_cast<double>(q.scalar())};
                    o.pack_array(2);
                    o.pack(qd);
                    o.pack(translation);
                    return o;
                }
            };

            template<>
            struct object_with_zone<k4a_calibration_extrinsics_t> {
                void operator()(msgpack::object::with_zone &o, const k4a_calibration_extrinsics_t &v) const {
                    // convert matrix to quaternion
                    Magnum::Matrix3x3 rotation{Magnum::Math::ZeroInit};
                    rotation[0][0] = v.rotation[0];
                    rotation[0][1] = v.rotation[1];
                    rotation[0][2] = v.rotation[2];
                    rotation[1][0] = v.rotation[3];
                    rotation[1][1] = v.rotation[4];
                    rotation[1][2] = v.rotation[5];
                    rotation[2][0] = v.rotation[6];
                    rotation[2][1] = v.rotation[7];
                    rotation[2][2] = v.rotation[8];

                    auto q = Magnum::Quaternion::fromMatrix(rotation).normalized();

                    // from float to double
                    Magnum::Math::Vector<3, double> translation{Magnum::Math::ZeroInit};
                    for (int i = 0; i < 3; i++) {
                        translation[i] = static_cast<double>(v.translation[i]);
                    }

                    auto qd = Magnum::Quaterniond{{static_cast<double>(q.vector().x()),
                                                   static_cast<double>(q.vector().y()),
                                                   static_cast<double>(q.vector().z())},
                                                  static_cast<double>(q.scalar())};

                    o.type = type::ARRAY;
                    o.via.array.size = 2;
                    o.via.array.ptr = static_cast<msgpack::object *>(
                            o.zone.allocate_align(sizeof(msgpack::object) * o.via.array.size));
                    o.via.array.ptr[0] = msgpack::object(qd, o.zone);
                    o.via.array.ptr[1] = msgpack::object(translation, o.zone);
                }
            };



/*
 * Ubitrack::Math::CameraIntrinsics<T> (pack only)
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

                    Magnum::Math::RectangularMatrix<3, 3, double> mat{Magnum::Math::ZeroInit};
                    mat[0][0] = static_cast<double>(p.param.fx);
                    mat[1][1] = static_cast<double>(p.param.fy);
                    mat[2][0] = -static_cast<double>(p.param.cx);
                    mat[2][1] = -static_cast<double>(p.param.cy);
                    mat[2][2] = -1.0;

                    Magnum::Math::Vector<2, Magnum::Int> dimension{v.resolution_width, v.resolution_height};

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
                    o.pack(mat);
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

                    Magnum::Math::RectangularMatrix<3, 3, double> mat{Magnum::Math::ZeroInit};
                    mat[0][0] = static_cast<double>(p.param.fx);
                    mat[1][1] = static_cast<double>(p.param.fy);
                    mat[2][0] = -static_cast<double>(p.param.cx);
                    mat[2][1] = -static_cast<double>(p.param.cy);
                    mat[2][2] = -1.0;

                    Magnum::Math::Vector<2, Magnum::Int> dimension{v.resolution_width, v.resolution_height};

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
                    o.via.array.ptr[2] = msgpack::object(mat, o.zone);
                    o.via.array.ptr[3] = msgpack::object(6, o.zone);
                    o.via.array.ptr[4] = msgpack::object(radial, o.zone);
                    o.via.array.ptr[5] = msgpack::object(tangential, o.zone);
                }
            };

        }
    }
}


#endif //MYAPPLICATION_MSGPACKSERIALIZATION_H
