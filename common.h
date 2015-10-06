#pragma once
#pragma GCC diagnostic ignored "-Warray-bounds"

#undef Success	/*Dirty workaround for redefinition of Success in X11.h on Linux*/
#include <pcl/point_types.h>

struct Triangle3D
{
	float p1[3];
	float p2[3];
	float p3[3];

	void set(int index, pcl::PointXYZ &point) {
		float *p = (*this)[index];
		p[0] = point.x;
		p[1] = point.y;
		p[2] = point.z;
	}

	// This will allow you to select points by id, instead of .p1/.p2/.p3
	inline float* operator[](const int idx) {
		return (p1 + idx * sizeof(p1) / sizeof(*p1));
	}
};

struct Triangle2D
{
	float p1[2];
	float p2[2];
	float p3[2];

	void set(int index, pcl::PointXY &point) {
		float *p = (*this)[index];
		p[0] = point.x;
		p[1] = point.y;
	}

	// This will allow you to select points by id, instead of .p1/.p2/.p3
	inline float* operator[](const int idx) {
		return (p1 + idx * sizeof(p1) / sizeof(*p1));
	}
};
