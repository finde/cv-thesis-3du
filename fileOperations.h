/*
 * fileOperations.h
 *
 *  Created on: May 26, 2014
 *      Author: morris
 */

#ifndef FILEOPERATIONS_H_
#define FILEOPERATIONS_H_

#include <string>
#include <fstream>
#include <opencv2/core/core.hpp>

char *readFile(char *filename, int &size);
char *readFile(char *filename);
std::string readFile(std::string path);
void saveMat(std::string path, cv::Mat &m);
cv::Mat loadMat(std::string path);
bool isFile(std::string);

/* WARNING The following functions can be used to store data in binary format, therefore, they are possibly NOT interchangable on different platforms (especially 32 vs 64 bits)
*/
template <class T> inline void writeNum(std::ofstream &of, T num) {
	of.write((char *)&num, sizeof(T));
}

template <class T> inline T readNum(char* &data) {
	T num = *((T *)data);
	data += sizeof(T);
	return num;
}

inline void writeStr(std::ofstream &os, std::string &str) {
	os << str << '\0';
}

inline std::string readStr(char* &data) {
	std::string str = std::string(data);
	data += str.length() + 1;
	return str;
}

/* Append a cv::Mat to an existing output stream,
 */
inline void writeMat(std::ofstream &os, cv::Mat &mat) {
	int type = mat.type();
	int rows = mat.rows;
	int cols = mat.cols;
	int sizeofMat = (mat.dataend - mat.data);

	writeNum(os, rows);
	writeNum(os, cols);
	writeNum(os, type);
	os.write((char *)mat.data, sizeofMat); // RGB
}

/* Extract a cv::Mat from an input datastream.
 * INPUT	: contents of file (NOT FILE PATH!!! -> use loadMat when loading single matrix from file)
 */
inline cv::Mat readMat(char* &data) {
	int rows = readNum<int>(data);
	int cols = readNum<int>(data);
	int type = readNum<int>(data);

	cv::Mat mat(rows,cols,type,data);
	data = (char *)mat.dataend;

	return mat;
}

/* add vector of SERIAL data to file stream (vector elements may NOT contain any pointers)
 * INPUT	: file ouput stream
			  Vector containing any data-type that does not contain any pointers in itself.
*/
template <class T> inline void writeVec(std::ofstream &os, std::vector<T> &vec) {
	int size = vec.size();
	writeNum(os, size);
	os.write((char *)&vec[0], size * sizeof(T));
}

/* Read vector from input datastream
 * INPUT	: contents of file (NOT FILE PATH!!!)
   OUTPUT	: Vector of type T, 
*/
template <class T> inline std::vector<T> readVec(char* &data) {
	int size = readNum<int>(data);
	std::vector<T> out;
	T *pd = (T *)data;
	out.assign(pd, pd + size);
	data += size * sizeof(T);
	return out;
}

template <class T> inline void writeArray(std::ofstream &os, T *data, int size) {
	writeNum(os, size);
	os.write((char *)data, size * sizeof(T));
}

template <class T> inline T* readArray(char* &data, int &size) {
	size = readNum<int>(data);
	if (size == 0) return NULL;
	T *out = (T *)data;
	data += size * sizeof(T);
	return out;
}

#endif /* FILEOPERATIONS_H_ */
