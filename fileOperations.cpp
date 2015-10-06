/*
 * fileOperations.cpp
 *
 *  Created on: May 26, 2014
 *      Author: morris
 */

#include "fileOperations.h"

#include <opencv2/core/mat.hpp>
#include <stdlib.h>
#include <sys/stat.h>
#include <iostream>
#include <iterator>

using namespace std;
using namespace cv;

string readFile(string path) {
	ifstream in(&path[0], ios::binary);
	string out((istreambuf_iterator<char>(in)), istreambuf_iterator<char>());
	in.close();

	return out;
}

char *readFile(char *filename, int &size) {
	ifstream file (filename, ios::in|ios::binary|ios::ate); //|ios::ate
	size = (int)file.tellg()+1;
	file.seekg (0, ios::beg);
	char *txt = (char *)malloc(size);
	file.read (txt,size-1); // or read size will include 0?
	file.close();
	txt[size-1] = 0;

	return txt;
}

char *readFile(char *filename) {
	ifstream file (filename, ios::in|ios::binary|ios::ate); //|ios::ate
	int size = (int)file.tellg()+1;
	file.seekg (0, ios::beg);
	char *txt = (char *)malloc(size);
	file.read (txt,size-1); // or read size will include 0?
	file.close();
	txt[size-1] = 0;

	return txt;
}

/* Save a cv::Mat to a file, such that it can be retrieved later
 * INPUT	: Path to the designated file
 * 			  cv::Mat
 */
void saveMat(string path, Mat &m) {
	cout << "saving mat..." << flush;
	ofstream os(&path[0], ios::binary);
	writeMat(os, m);
	os.close();
	cout << "done!" << endl;
}

/* Loads a cv::Mat (matrix) that has been saved using the saveMat function
 * INPUT	: path to the file (string)
 * OUTPUT	: Matrix that has been stored in the file
 */
Mat loadMat(string path) {
	int size;
	char *data = readFile(&path[0],size);
	Mat m;
	if (size > 0)
		m = readMat(data);
	else
		cerr << "could not open file: " << path << endl;

	return m;
}

bool isFile(string path) {
	struct stat s;
	if( stat(&path[0], &s) == 0 ) {
		return s.st_mode & S_IFREG;
	}
	return false;
}
