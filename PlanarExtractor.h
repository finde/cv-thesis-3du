#pragma once

#include <opencv2/core/core.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace cv;
using namespace pcl;

#define USE_NORMAL_PLANE 1
#define MIN_INLIERS 10

class PlanarExtractor
{
public:
	int stride				= 4;
	float radius			= 0.05;
	float distanceWeight	= 0.13f;
	int maxIterations		= 1000;
	float distanceThreshold = USE_NORMAL_PLANE ? 0.04 : 0.005;

	PlanarExtractor();
	PlanarExtractor(int argc, char *argv[]);
	~PlanarExtractor();

	void print();
	
	PointCloud<PointXYZI>::Ptr getPointCloud(Mat &depth, vector<Point> &position);
	PointCloud<PointXYZRGB>::Ptr getRGBCloud(Mat &depth);
	vector<vector<PointXYZI> > getPlanar(Mat &depth, Mat rgb);
};

