#include "opencv2/core/geometry.hpp"
using namespace cv;

TriangleMesh::TriangleMesh()
{

}

TriangleMesh::TriangleMesh(InputArray _vertices, InputArray _indecies, InputArray _normals)
{
}

template<typename T>
bool TriangleMesh::intersects(const Ray<T, 3>& ray, cv::Point3_<T>* intersection_point)
{

}

template TriangleMesh::intersects<float>(const Ray<float, 3>& ray, cv::Point3_<float>* intersection_point);
template TriangleMesh::intersects<double>(const Ray<double, 3>& ray, cv::Point3_<double>* intersection_point);