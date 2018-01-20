//
//  se3.h
//
//  Created by KUAN LU on 2017/5/12.
//  Copyright © 2017年 KUAN LU. All rights reserved.
//

#ifndef _SE3_H_
#define _SE3_H_

#include "Matrix.h"
#include "helper_math.h"
#include <opencv2/core.hpp>

namespace rmd
{

    template<typename Type>
    class SE3
    {
    public:
        SE3()
        {
            /* TODO: initialize to [eye(3) [0 0 0]']*/
        }

        /// Constructor from a normalized quaternion and a translation vector
        SE3(Type qw, Type qx, Type qy, Type qz, Type tx, Type ty, Type tz)
        {
            const Type x  = 2*qx;
            const Type y  = 2*qy;
            const Type z  = 2*qz;
            const Type wx = x*qw;
            const Type wy = y*qw;
            const Type wz = z*qw;
            const Type xx = x*qx;
            const Type xy = y*qx;
            const Type xz = z*qx;
            const Type yy = y*qy;
            const Type yz = z*qy;
            const Type zz = z*qz;

            data(0, 0) = 1-(yy+zz);
            data(0, 1) = xy-wz;
            data(0, 2) = xz+wy;
            data(1, 0) = xy+wz;
            data(1, 1) = 1-(xx+zz);
            data(1, 2) = yz-wx;
            data(2, 0) = xz-wy;
            data(2, 1) = yz+wx;
            data(2, 2) = 1-(xx+yy);

            data(0, 3) = tx;
            data(1, 3) = ty;
            data(2, 3) = tz;
        }

        /// Construct from C arrays
        /// r is rotation matrix row major
        /// t is the translation vector (x y z)
        SE3(Type *r, Type *t)
        {
            data[0]=r[0]; data[1]=r[1]; data[2] =r[2]; data[3] =t[0];
            data[4]=r[3]; data[5]=r[4]; data[6] =r[5]; data[7] =t[1];
            data[8]=r[6]; data[9]=r[7]; data[10]=r[8]; data[11]=t[2];
        }

        SE3(const cv::Matx33f &r, const cv::Vec3f &t)
        {
            data[0]=r(0,0); data[1]=r(0,1); data[2] =r(0,2); data[3] =t[0];
            data[4]=r(1,0); data[5]=r(1,1); data[6] =r(1,2); data[7] =t[1];
            data[8]=r(2,0); data[9]=r(2,1); data[10]=r(2,2); data[11]=t[2];
        }

        SE3<Type> inv() const
        {
            SE3<Type> result;
            result.data[0]  = data[0];
            result.data[1]  = data[4];
            result.data[2]  = data[8];
            result.data[4]  = data[1];
            result.data[5]  = data[5];
            result.data[6]  = data[9];
            result.data[8]  = data[2];
            result.data[9]  = data[6];
            result.data[10] = data[10];
            result.data[3]  = -data[0]*data[3] -data[4]*data[7] -data[8] *data[11];
            result.data[7]  = -data[1]*data[3] -data[5]*data[7] -data[9] *data[11];
            result.data[11] = -data[2]*data[3] -data[6]*data[7] -data[10]*data[11];
            return result;
        }

        cv::Vec3f getTranslation() const
        {
            return cv::Vec3f(data[3],data[7],data[11]);
        }

        Type operator()(int r, int c) const
        {
            return data(r, c);
        }

        Type & operator()(int r, int c)
        {
            return data(r, c);
        }

        friend std::ostream & operator<<(std::ostream &out, const SE3 &m)
        {
            out << m.data;
            return out;
        }

        float3 rotate(const float3 &p) const
        {
            return make_float3(data(0,0)*p.x + data(0,1)*p.y + data(0,2)*p.z,
                               data(1,0)*p.x + data(1,1)*p.y + data(1,2)*p.z,
                               data(2,0)*p.x + data(2,1)*p.y + data(2,2)*p.z);
        }

        float3 translate(const float3 &p) const
        {
            return make_float3(p.x + data(0,3),
                               p.y + data(1,3),
                               p.z + data(2,3));
        }

        Matrix<Type, 3, 4> data;
    };

    template<typename Type>
    SE3<Type> operator*(const SE3<Type> &lhs, const SE3<Type> &rhs)
    {
        SE3<Type> result;
        result.data[0]  = lhs.data[0]*rhs.data[0] + lhs.data[1]*rhs.data[4] + lhs.data[2]*rhs.data[8];
        result.data[1]  = lhs.data[0]*rhs.data[1] + lhs.data[1]*rhs.data[5] + lhs.data[2]*rhs.data[9];
        result.data[2]  = lhs.data[0]*rhs.data[2] + lhs.data[1]*rhs.data[6] + lhs.data[2]*rhs.data[10];
        result.data[3]  = lhs.data[3] + lhs.data[0]*rhs.data[3] + lhs.data[1]*rhs.data[7] + lhs.data[2]*rhs.data[11];
        result.data[4]  = lhs.data[4]*rhs.data[0] + lhs.data[5]*rhs.data[4] + lhs.data[6]*rhs.data[8];
        result.data[5]  = lhs.data[4]*rhs.data[1] + lhs.data[5]*rhs.data[5] + lhs.data[6]*rhs.data[9];
        result.data[6]  = lhs.data[4]*rhs.data[2] + lhs.data[5]*rhs.data[6] + lhs.data[6]*rhs.data[10];
        result.data[7]  = lhs.data[7] + lhs.data[4]*rhs.data[3] + lhs.data[5]*rhs.data[7] + lhs.data[6]*rhs.data[11];
        result.data[8]  = lhs.data[8]*rhs.data[0] + lhs.data[9]*rhs.data[4] + lhs.data[10]*rhs.data[8];
        result.data[9]  = lhs.data[8]*rhs.data[1] + lhs.data[9]*rhs.data[5] + lhs.data[10]*rhs.data[9];
        result.data[10] = lhs.data[8]*rhs.data[2] + lhs.data[9]*rhs.data[6] + lhs.data[10]*rhs.data[10];
        result.data[11] = lhs.data[11] + lhs.data[8]*rhs.data[3] + lhs.data[9]*rhs.data[7] + lhs.data[10]*rhs.data[11];
        return result;
    }

} // namespace rmd

#endif /* _SE3_H_ */