#include <iostream>
#include <boost/thread/thread.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d_omp.h>

#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>

#include "PlanarExtractor.h"

using namespace std;
using namespace cv;
using namespace pcl;

/* SUPPORT FUNCTIONS */

/* Compute the surface area of a polygon given its vertexes (Green's theorem).
* Input	: A vector of points, There are multiple types of points, but they should at least have an x and y variable.
* Output	: Surface area of the given polygon
*/
template <class T> double getArea(vector<T> &r) {
	double value = 0;
	T prev = r.back();

	for (size_t i = 0, iend = r.size(); i < iend; i++) { // TODO use c++11 loop? :: c++11 does not work with PCL At this moment...
		T &p = r[i];
		value += prev.x*p.y - prev.y*p.x;
		prev = p;
	}

	return 0.5 * abs(value);
}

/* Since the intensity-dimension of the PointXYZI is used as an index, it's possible to extract the original position
* Input		: A vector of points where the index-value represents the index of the point in the point-cloud
* Output	: vector of Points
*/
template <class T, class B> vector<T> fetchFromIndex(vector<PointXYZI> &indexes, B &points) {
	const size_t size = indexes.size();
	vector<T> ret(size);

	for (size_t i = 0; i < size; i++)
		ret[i] = points[(int)indexes[i].intensity];

	return ret;
}

/* 'Rotate' Point 45 degrees around z, it doesn't actually rotate, but in this case it's unnesesary to do full rotation by multiplying with sin(45) and cos(45), while the implementation below is faster.
*/
template <class T> inline T rotate45(T &p) {
	T ret = p;
	ret.x += p.y;
	ret.y -= p.x;
	return ret;
}

/* Removes a set of indexes from the input vector (vec). The input vector (vec) will be modified
*  INPUT	: vector of some type, and a vector of indexes
*/
template <class T, class B> void removeIndexes(T &vec, B &indexes) {
	const size_t vec_size = vec.size();
	const size_t ind_size = indexes.size();
	bool *boollist = (bool *)calloc(vec_size, sizeof(bool));

	for (size_t i = 0; i < ind_size; ++i)
		boollist[indexes[i]] = true;

	for (size_t i = vec_size; i > 0; --i) {
		if (boollist[i]) {
			vec[i] = vec.back();
			vec.pop_back();
		}
	}

	free(boollist);
}


/* BEGINNING OF CLASS PLANAREXTRACTOR */

PlanarExtractor::~PlanarExtractor() {
}

PlanarExtractor::PlanarExtractor() {
}

PlanarExtractor::PlanarExtractor(int argc, char *argv[]) {
	for (int i = 1; i < (argc - 1); i++) {
		cout << "arg[" << i << "] = " << argv[i] << endl;
		char *arg = argv[i];

		if (strcmp(arg, "-stride") == 0) 		stride = atoi(argv[++i]);
		else if (strcmp(arg, "-radius") == 0) 	radius = atoi(argv[++i]);
		else if (strcmp(arg, "-dw") == 0) 		distanceWeight = atof(argv[++i]);
		else if (strcmp(arg, "-mi") == 0) 		maxIterations = atoi(argv[++i]);
		else if (strcmp(arg, "-dt") == 0) 		distanceThreshold = atof(argv[++i]);
		else cerr << "undefined argument " << arg << endl;
	}
}

void PlanarExtractor::print() {
	printf("%15s = %d\n", "stride", stride);
	printf("%15s = %f\n", "radius", radius);
	printf("%15s = %f\n", "distanceWeight", distanceWeight);
	printf("%15s = %d\n", "maxIterations", maxIterations);
	printf("%15s = %f\n", "distanceThreshold", distanceThreshold);
}


/* Convert 16-bit (CV_16U) depth image to a PCL point cloud
*/
PointCloud<PointXYZI>::Ptr PlanarExtractor::getPointCloud(Mat &depth, vector<Point> &position) {
	PointCloud<PointXYZI>::Ptr pointcloud(new PointCloud<PointXYZI>);
	pointcloud->points.resize(depth.total());
	position.resize(depth.total());

	const register float constant = 1.0f / 536.36839530;
	const register int centerX = depth.cols >> 1;
	const int centerY = depth.rows >> 1;
	register int depth_idx = 0;

	for (int v = -centerY; v < centerY; v += stride) {
		for (register int u = -centerX; u < centerX; u += stride) {
			const Point p((u + centerX), (v + centerY));
			const unsigned short depthp = depth.at<unsigned short>(p);
			if (depthp) {
				PointXYZI& pt = pointcloud->points[depth_idx];
				position[depth_idx] = p;
				pt.z = depthp * 0.0005f;			// depth_data[depth_idx * stride_f] * 0.001f;
				pt.x = static_cast<float> (u)* -pt.z * constant;
				pt.y = static_cast<float> (v)* -pt.z * constant;
				pt.intensity = depth_idx;			// NOTE: I (as in intensity) is being treated as INDEX here, the index is being used to convert the points back to their position on the screen
				++depth_idx;
			}
		}
	}

	pointcloud->height = 1;
	pointcloud->width = depth_idx;
	pointcloud->points.resize(depth_idx);
	position.resize(depth_idx);

	return pointcloud;
}

/* Convert 16-bit (CV_16U) depth image to RGB point cloud (depth-values are used as gray-RGB values)
*/
PointCloud<PointXYZRGB>::Ptr PlanarExtractor::getRGBCloud(Mat &depth) {
	PointCloud<PointXYZRGB>::Ptr pointcloud(new PointCloud<PointXYZRGB>);
	pointcloud->points.resize(depth.total());

	const register float constant = 1.0f / 536.36839530;
	const register int centerX = depth.cols >> 1;
	const int centerY = depth.rows >> 1;
	register int depth_idx = 0;

	for (int v = -centerY; v < centerY; v += stride) {
		for (register int u = -centerX; u < centerX; u += stride) {
			Point p((u + centerX), (v + centerY));
			const unsigned short depthp = depth.at<unsigned short>(p);
			if (depthp > 0) {
				PointXYZRGB& pt = pointcloud->points[depth_idx];
				pt.z = depthp * 0.0005f;			// depth_data[depth_idx * stride_f] * 0.001f;
				pt.x = static_cast<float> (u)* -pt.z * constant;
				pt.y = static_cast<float> (v)* -pt.z * constant;
				pt.r = depthp / 16;
				pt.g = depthp / 16;
				pt.b = depthp / 16;
				++depth_idx;
			}
		}
	}
	pointcloud->height = 1;
	pointcloud->width = depth_idx;
	pointcloud->points.resize(depth_idx);

	return pointcloud;
}

/* Compute the corner points of the biggest planar in the area (based on depth image)
* Input 	: Depth image (CV_16U), and rgb image, which is used for display
* Output	: vector of planes, where each plane is represented by 4 Corner points on the planar surface
*/
vector<vector<PointXYZI> > PlanarExtractor::getPlanar(Mat &depth, Mat rgb) {
	vector<vector<PointXYZI> > allCorners;
	PointCloud<PointXYZI>::Ptr cloud_inliers(new PointCloud<PointXYZI>);
	PointCloud<PointXYZI>::Ptr cloud_projection(new PointCloud<PointXYZI>);
	PointIndices::Ptr inliers(new PointIndices);
	ModelCoefficients::Ptr coefficients(new ModelCoefficients);
	vector<Point> positions;

	// *Extract Point cloud from depth image*
	PointCloud<PointXYZI>::Ptr cloud = getPointCloud(depth, positions);

	for (int plannr = 1;; plannr++) {
		// *Compute the planar*
#if USE_NORMAL_PLANE
		PointCloud<Normal>::Ptr normals_out(new PointCloud<Normal>);
		NormalEstimationOMP<PointXYZI, Normal> norm_est;
		norm_est.setSearchMethod(search::KdTree<PointXYZI>::Ptr(new search::KdTree<PointXYZI>));
		norm_est.setRadiusSearch(radius);
		norm_est.setInputCloud(cloud);
		norm_est.compute(*normals_out);

		SACSegmentationFromNormals<PointXYZI, Normal> segNormal;
		segNormal.setOptimizeCoefficients(true);
		segNormal.setModelType(SACMODEL_NORMAL_PLANE);
		segNormal.setNormalDistanceWeight(distanceWeight);
		segNormal.setMethodType(SAC_RANSAC);
		segNormal.setMaxIterations(maxIterations);
		segNormal.setDistanceThreshold(distanceThreshold);
		segNormal.setInputCloud(cloud);
		segNormal.setInputNormals(normals_out);
		segNormal.segment(*inliers, *coefficients);
#else
		SACSegmentation<PointXYZI> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(SACMODEL_PLANE);
		seg.setMethodType(SAC_RANSAC);
		seg.setDistanceThreshold(params.distanceThreshold); 							// WARNING : non normalized kinect input data, possible max: 4096 (needs verfification), how does this threshold compare to other devices???
		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);
#endif

		ExtractIndices<PointXYZI> extract_inliers;
		extract_inliers.setInputCloud(cloud);
		extract_inliers.setIndices(inliers);
		extract_inliers.setNegative(false);
		extract_inliers.filter(*cloud_inliers);
		const size_t numInliers = inliers->indices.size();

		if (numInliers < MIN_INLIERS) break;

		// *Place inlier overlay over rgb image*
		const int channel = plannr % 3;
		for (int i = 0, iend = numInliers; i < iend; i++) {
			Point &p = positions[inliers->indices[i]];
			for (int x = p.x, xend = min(int(p.x + stride), rgb.cols - 1); x < xend; x++) {
				for (int y = p.y, yend = min(int(p.y + stride), rgb.rows - 1); y < yend; y++) {
					Vec3b &col = rgb.at<Vec3b>(y, x);
					col[channel] = (col[channel] + 255) / 2;
				}
			}
		}

		float a = coefficients->values[0];
		float b = coefficients->values[1];
		float c = coefficients->values[2];

		Eigen::Vector3f x_axis(b / sqrt(a * a + b * b), -a / sqrt(a * a + b * b), 0);
		Eigen::Vector3f y_direction(a, b, c);
		Eigen::Affine3f rotation = getTransFromUnitVectorsXY(x_axis, y_direction);

		for (int x = 0; x < 3; x++) // The rotation actually flattens the XZ axis, instead of XY, since using XY variables is much more convienient, swap the rotation of the Y and Z axis around
			swap(rotation(1, x), rotation(2, x));

		PointCloud<PointXYZI>::Ptr rotated(new PointCloud<PointXYZI>);
		transformPointCloud(*cloud_inliers, *rotated, rotation);

#if 0
		// viewing purposes:: view rotated inlier plane
		for (int i = 0; i < 10; i++) {
			cout << rotated->points[i] << endl;
		}
		visualization::CloudViewer viewer("Simple Cloud Viewer");
		viewer.showCloud(rotated);
		while (!viewer.wasStopped()) sleep(500);
#endif

		vector<PointXYZI> rect1(4);					// Corner points in projection-space
		vector<PointXYZI> rect2(4);					// Corner points in projection-space, but rotated 45 degrees
		for (int i = 0; i < 4; i++) {				// Initialize corner points to the first point
			rect1[i] = rotated->points[0];
			rect2[i] = rotate45(rotated->points[0]);
		}

		// *Compute corner points of planar*
		// Do estimation of rectangle, in order to really compute the corners compute for each point the point that has the LARGEST distance (farthest neighbor?). Keep track how many times each point has been listed as farthest, and create histogram.
		// For estimation, make 2 rectangles, first extract the 4 extrema (top/right/bottom/left), do the same but then rotate 45 degrees. Take the rectangle with the largest surface area.
		for (size_t i = 1; i < numInliers; i++) { 		// start from 1, since 0 is already set
			PointXYZI p = rotated->points[i];
			PointXYZI p45 = rotate45(p);

			if (p.y > rect1[0].y) rect1[0] = p; 	// TOP
			if (p.x > rect1[1].x) rect1[1] = p; 	// RIGHT
			if (p.y < rect1[2].y) rect1[2] = p; 	// BOTTOM
			if (p.x < rect1[3].x) rect1[3] = p; 	// LEFT

			if (p45.y > rect2[0].y) rect2[0] = p45; // TOP-LEFT
			if (p45.x > rect2[1].x) rect2[1] = p45; // TOP-RIGHT
			if (p45.y < rect2[2].y) rect2[2] = p45; // BOTTOM-RIGHT
			if (p45.x < rect2[3].x) rect2[3] = p45; // BOTTOM-LEFT
		}

		// Measure rectangle surface area for both suggestions, and use the largest one
		double area1 = getArea(rect1);
		double area2 = getArea(rect2);
		vector<PointXYZI> corners = fetchFromIndex<PointXYZI>(area1 > area2 ? rect1 : rect2, cloud->points);
		vector<Point> cvCorners = fetchFromIndex<Point>(corners, positions);
		allCorners.push_back(corners);

		// compute rectangle
		Point prev = cvCorners.back();
		for (int i = 0; i < 4; i++) {
			Point &p = cvCorners[i];
			circle(rgb, p, 5, cvScalar(0, 0, 255), -1);
			line(rgb, prev, p, cvScalar(0, 0, 255), 1);
			prev = p;
		}

		removeIndexes(cloud->points, inliers->indices);
		cloud->width = cloud->points.size();

		cout << "Corners: " << corners[0] << " " << corners[1] << " " << corners[2] << " " << corners[3] << std::endl;
		imshow("planar", rgb);
		if (waitKey() == 27) break;
	}

	return allCorners;
}
