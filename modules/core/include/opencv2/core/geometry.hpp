/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#ifndef __OPENCV_CORE_GEOMETRY_HPP__
#define __OPENCV_CORE_GEOMETRY_HPP__

#ifndef __cplusplus
#  error matx.hpp header must be compiled as C++
#endif
#include "opencv2/core/cvdef.h"
#include "opencv2/core/types.hpp"
#include "opencv2/core/matx.hpp"

namespace cv
{
    // Planar triangle in Nd space
    template<typename _Tp, int cn> class Triangle
    {
    public:
        template<typename T> Triangle(const Vec<T, cn>& _A, const Vec<T, cn>& _B, const Vec<T, cn>& _C);
        Vec<_Tp, cn> norm() const;

        template<typename T> bool contains(const Vec<T, cn>& pt) const;
        template<typename T> bool contains(const Point3_<T>& pt) const;
        _Tp area() const;

        Vec<_Tp, cn> vertices[3];
    };

    // Initial implementation is just 3d, meant to be expanded further.
    template<typename _Tp, int cn> class Ray
    {
    public:
        template<typename _Tp2> Ray(const Vec<_Tp2, cn>& _origin, const Vec<_Tp2, cn>& _direction);
        template<typename _Tp2> Ray(const Point3_<_Tp2>& _origin, const Vec<_Tp2, cn>& _direction);
        template<typename _Tp2> bool intersects(const Triangle<_Tp2, cn>& triangle, Point3d<_Tp2>* intersection_point = NULL) const;
        Vec<_Tp, cn> extrapolate(_Tp dist) const;

        Vec<_Tp, cn> origin;
        Vec<_Tp, cn> direction;
    };

    class CV_EXPORTS TriangleMesh
    {
    public:
        TriangleMesh();
        TriangleMesh(InputArray _vertices, InputArray _indecies = noArray(), InputArray _normals = noArray());
        template<typename T> bool intersects(const Ray<T, 3>& ray, Point3_<T>* intersection_point = NULL);

    protected:
        Mat vertices;
        Mat normals;
        Mat indecies;
    };

    ////////////////////////////////// Ray //////////////////////////////////////////////
    template<typename _Tp, int cn> template<typename _Tp2> inline
        Ray<_Tp, cn>::Ray(const Vec<_Tp2, cn>& _origin, const Vec<_Tp2, cn>& _direction)
        :origin(_origin), direction(_direction) {}
    template<typename _Tp, int cn> template<typename _Tp2> inline
        Ray<_Tp, cn>::Ray(const Point3_<_Tp2>& _origin, const Vec<_Tp2, cn>& _direction)
        :origin(_origin), direction(_direction) {}

    template<typename _Tp, int cn> template<typename _Tp2> inline
        bool Ray<_Tp, cn>::intersects(const Triangle<_Tp2, cn>& triangle) const
    {
        Vec<_Tp, cn> norm = triangle.norm();
        if (direction.dot(norm) == 0)
            return false;

        _Tp D = norm.dot(triangle.vertices[0]);

        _Tp t = (norm.dot(origin) + D) / (norm.dot(direction));

        Vec<_Tp, cn> pt = extrapolate(t);
        return triangle.contains(pt);
    }

    template<typename _Tp, int cn> inline
    Vec<_Tp, cn> Ray<_Tp, cn>::extrapolate(_Tp dist) const
    {
        return origin + dist * direction;
    }

    ////////////////////////////////// Triangle /////////////////////////////////////////
    template<typename _Tp, int cn> template<typename T> inline
        Triangle<_Tp, cn>::Triangle(const Vec<T, cn>& _A, const Vec<T, cn>& _B, const Vec<T, cn>& _C)
    {
        vertices[0] = _A;
        vertices[1] = _B;
        vertices[2] = _C;
    }

    template<typename _Tp, int cn> inline
        Vec<_Tp, cn> Triangle<_Tp, cn>::norm() const
    {
        Vec<_Tp, cn> AB(vertices[0], vertices[1]);
        Vec<_Tp, cn> AC(vertices[0], vertices[2]);
        return AB.cross(AC);
    }

    template<typename _Tp, int cn> template<typename T> inline
        bool Triangle<_Tp, cn>::contains(const Vec<T, cn>& pt) const
    {
        Vec<_Tp, cn> AB(vertices[0], vertices[1]);
        Vec<_Tp, cn> BC(vertices[1], vertices[2]);
        Vec<_Tp, cn> CA(vertices[2], vertices[0]);
        Vec<_Tp, cn> C0(vertices[0], pt);
        Vec<_Tp, cn> C1(vertices[1], pt);
        Vec<_Tp, cn> C2(vertices[2], pt);

        Vec<_Tp, cn> norm_ = norm();
        // Inside-outside test
        // If the value == 0, then the point lies on the edge, positive is inside, negative is outside.
        return norm_.dot(AB.cross(C0)) >= 0 && norm_.dot(BC.cross(C1)) >= 0 && norm_.dot(CA.cross(C2)) >= 0;
    }

    template<typename _Tp, int cn> template<typename T> inline
        bool Triangle<_Tp, cn>::contains(const Point3_<T>& pt) const
    {
        CV_StaticAssert(cn == 3, "Can only compare 3d point to 3d triangle")
        Vec<_Tp, cn> AB(vertices[0], vertices[1]);
        Vec<_Tp, cn> BC(vertices[1], vertices[2]);
        Vec<_Tp, cn> CA(vertices[2], vertices[0]);
        Vec<_Tp, cn> C0(vertices[0], pt);
        Vec<_Tp, cn> C1(vertices[1], pt);
        Vec<_Tp, cn> C2(vertices[2], pt);

        Vec<_Tp, cn> norm_ = norm();
        // Inside-outside test
        // If the value == 0, then the point lies on the edge, positive is inside, negative is outside.
        return norm_.dot(AB.cross(C0)) >= 0 && norm_.dot(BC.cross(C1)) >= 0 && norm_.dot(CA.cross(C2)) >= 0;
    }

    template<typename _Tp, int cn> inline
        _Tp Triangle<_Tp, cn>::area() const
    {
        return 0.5 * norm(Vec<_Tp, cn>(vertices[0], vertices[1]).cross(Vec<_Tp, cn>(vertices[0], vertices[2])));
    }
} // cv

#endif // __OPENCV_CORE_GEOMETRY_HPP__