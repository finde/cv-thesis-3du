
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>

#include <pcl/common/time.h>
#include <pcl/common/angles.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/features/normal_3d_omp.h>

#include "Triangulate.h"

using namespace std;
using namespace cv;
using namespace pcl;

bool getTriangles(string depthpath, float focal_length, vector<Triangle2D> &triangles2D, vector<Triangle3D> &triangles3D, const float imgWidth, const float imgHeight) {
	Mat depth;
	FileStorage fs(depthpath, FileStorage::READ);
	fs["depth"] >> depth;
	fs.release();
	if (depth.empty())
		return false;

	vector<PointXY> point2ds;
	PointCloud<PointXYZ>::Ptr pointCloud = GenPointCloudFromMat(depth, point2ds, focal_length);
	PolygonMesh::Ptr triangles = GenTriangles(pointCloud);

	triangles2D.reserve(triangles->polygons.size());
	triangles3D.reserve(triangles->polygons.size());

	//for (auto triangle : triangles->polygons) {
	BOOST_FOREACH(Vertices &triangle, triangles->polygons ) {
		Triangle3D triangle3D;
		Triangle2D triangle2D;

		for (int j = 0; j < 3; j++) {
			const int index = triangle.vertices[j];
			PointXYZ &pxyz = pointCloud->points[index];
			PointXY &pxy = point2ds[index];

			triangle3D.set(j, pxyz);
			triangle2D.set(j, pxy);
		}
		triangles3D.push_back(triangle3D);
		triangles2D.push_back(triangle2D);
	}
	return true;
}

PolygonMesh::Ptr GenTriangles(PointCloud<PointXYZ>::Ptr &tmp_xyz) {
	PointCloud<PointNormal>::Ptr combined_ptr_ = PointCloud<PointNormal>::Ptr(new PointCloud<PointNormal>);

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(tmp_xyz);
	n.setInputCloud(tmp_xyz);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);
	pcl::concatenateFields(*tmp_xyz, *normals, *combined_ptr_);
	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(combined_ptr_);
	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);
	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);
	PolygonMesh::Ptr triangles_ptr_(new PolygonMesh());
	// Get result
	gp3.setInputCloud(combined_ptr_);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(*triangles_ptr_);

	return triangles_ptr_;
}

PointCloud<PointXYZ>::Ptr GenPointCloudFromMat(const cv::Mat& depth_, vector<PointXY>& points2d, float depth_constant_)
{
	PointCloud<PointXYZ>::Ptr points_ = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);
	int width = depth_.cols;
	int height = depth_.rows;
	register int centerX = (width >> 1);
	int centerY = (height >> 1);
	register int depth_idx = 0;
	points_->points.clear();
	points2d.clear();
	float z;
	for (int v = -centerY; v < centerY; ++v)
	{
		for (register int u = -centerX; u < centerX; ++u, ++depth_idx)
		{
			pcl::PointXYZ p1;
			pcl::PointXY p2d;
			z = depth_.at<ushort>(depth_idx) / 1000.f;
			if (z > 0)
			{
				p1.z = z;
				p1.x = static_cast<float> (u)* z / depth_constant_;
				p1.y = static_cast<float> (v)* z / depth_constant_;
				points_->points.push_back(p1);
				p2d.x = u + centerX;
				p2d.y = v + centerY;
				points2d.push_back(p2d);
			}
		}
	}

	return points_;
}
