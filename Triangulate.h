#pragma once

#include <vector>
#include <opencv2/core/core.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>

#include "common.h"

using namespace std;

bool getTriangles(string depthpath, float focal_length, vector<Triangle2D> &triangles2D, vector<Triangle3D> &triangles3D, const float imgWidth, const float imgHeight);
pcl::PolygonMesh::Ptr GenTriangles(pcl::PointCloud<pcl::PointXYZ>::Ptr &tmp_xyz);
pcl::PointCloud<pcl::PointXYZ>::Ptr GenPointCloudFromMat(const cv::Mat& depth_, vector<pcl::PointXY>& points2d, float depth_constant_);
