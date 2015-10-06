#pragma once

#ifdef __WXMAC__
	#include "OpenGL/glu.h"
	#include "OpenGL/gl.h"
#else
	#ifdef WIN32
		#include <windows.h>
    #endif
    #define GLEW_STATIC
    #include <GL/glew.h>
    #include <GLFW/glfw3.h>
#endif

#include <vector>
#include <stdio.h>
#include <math.h>
#include <opencv2/core/core.hpp>

#include "common.h"


using namespace std;

class Model
{
public:
	int size;
	vector<Triangle3D> triangles3D;
	vector<Triangle2D> triangles2D;
	cv::Mat image;
	GLuint texture;
	int isReady;
	bool succes;

	bool loadPoints(string path);
	void savePointData(string path);
	void readPointData(string path);
	bool loadImage(string path, cv::Mat &image, GLuint &texture);

	Model();
	~Model();

	void draw();
	void load(string imagePath);
};
