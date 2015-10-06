#include <iostream>
#include <fstream>

//#include <wx/wxprec.h>
//#ifndef WX_PRECOMP
//#include <wx/wx.h>
//#endif

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "fileOperations.h"
#include "Triangulate.h"
#include "common.h"
#include "Model.h"

using namespace std;
using namespace cv;

Model::Model() : size(0), texture(0), isReady(0), succes(false) {

}

Model::~Model() {
    // TODO cleanup openGL textures etc..? -> Texture paint needs major revision anyway....
}

bool showingMessage = false; // horrible workaround to prevent infinity messages when unable to load the model
void Model::load(string imagePath) {
    cout << "loading :: " << imagePath << "..." << flush;
    triangles2D.clear();
    triangles3D.clear();
    int dot = imagePath.find_last_of('.');
    string dataPath = imagePath.substr(0, dot);
    isReady++;

    succes = loadImage(imagePath, image, texture);
    if (succes) {
        succes = loadPoints(dataPath);
    }

    showingMessage = false;
    if (!succes) {
        if (!showingMessage) {
//			wxMessageBox("Cannot load " + imagePath, "Error loading model, wxOK | wxICON_ERROR);
            cout << "Cannot load " << imagePath << "Error loading model" << endl;
            showingMessage = true;
        }
    }
    cout << "Done!" << endl;
}


bool Model::loadImage(string path, Mat &image, GLuint &texture) {
    image = imread(path, IMREAD_COLOR);
    if (!image.empty()) {
        cvtColor(image, image, CV_BGR2RGB);
        glPixelStorei(GL_UNPACK_ALIGNMENT,(image.step & 3) ? 1 : 4);               // use fast 4-byte alignment (default anyway) if possible
        glPixelStorei(GL_UNPACK_ROW_LENGTH, image.step / image.elemSize());        // set length of one complete row in data (doesn't need to equal image.cols)

        glGenTextures(1, &texture);                    // Create The Texture
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.cols, image.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, image.ptr());
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glEnable(GL_TEXTURE_2D);
        glShadeModel(GL_SMOOTH);
        return true;
    }
    cerr << "image loading failed! could not open " << path << endl;
    return false;
}

bool Model::loadPoints(string path) {
    ifstream ifs((path + ".tri").c_str());
    if (ifs.is_open()) {
        readPointData(path + ".tri");
    }
    else { // TODO should probably do this on a different thread instead of the main thread... future work!
        float focalLength = 510.75759609098617;        // TODO make focal length part of the file, at least avoid hardcoding it like this!
        bool success = getTriangles(path + ".mat", focalLength, triangles2D, triangles3D, image.cols, image.rows);
        if (!success) return success;
        const float divwidth = 1 / (float) image.cols;
        const float divheight = 1 / (float) image.rows;

        // fix triangles to correct format
        double totalz = 0;
        for (float *data3D = &(triangles3D[0].p1[0]), *data_end =
                data3D + triangles3D.size() * sizeof(Triangle3D) / sizeof(float); data3D < data_end; data3D += 3) {
            data3D[1] *= -1;
            data3D[2] *= -1;
            totalz += data3D[2];
        }
        double multiplier = totalz / ((double) ((triangles3D.size() * sizeof(Triangle3D) / sizeof(float)) / 3));
        for (float *data3D = &(triangles3D[0].p1[2]), *data_end =
                data3D + triangles3D.size() * sizeof(Triangle3D) / sizeof(float); data3D < data_end; data3D += 3) {
            data3D[0] -= multiplier;    // Center object
        }

        // convert texture coordinates to proportional mapping (required by OpenGL)
        for (float *data2D = &(triangles2D[0].p1[0]), *data_end =
                data2D + triangles2D.size() * sizeof(Triangle2D) / sizeof(float); data2D < data_end; data2D += 2) {
            data2D[0] *= divwidth;
            data2D[1] *= divheight;
        }

        savePointData(path + ".tri");
    }

    size = triangles3D.size();
    return true;
}


void Model::savePointData(string path) {
    ofstream os(path.c_str(), ios::binary);
    writeVec(os, triangles2D);
    writeVec(os, triangles3D);
    os.close();
}

void Model::readPointData(string path) {
    char *data = readFile(&path[0]);
    triangles2D = readVec<Triangle2D>(data);
    triangles3D = readVec<Triangle3D>(data);
}

// TODO Revise entire draw method to new OpenGL standards (with shaders and stuff)	--> currently not well support in combination with wxWidgets, and default Linux drivers
void Model::draw() {
    if (!succes) return;        // some error happened while loading, do not proceed
    glBindTexture(GL_TEXTURE_2D, texture);

    for (int i = 0; i < size; i++) {
        Triangle2D &t2D = triangles2D[i];
        Triangle3D &t3D = triangles3D[i];

        glBegin(GL_POLYGON);
        glTexCoord2fv(t2D.p1);
        glVertex3fv(t3D.p1);

        glTexCoord2fv(t2D.p2);
        glVertex3fv(t3D.p2);

        glTexCoord2fv(t2D.p3);
        glVertex3fv(t3D.p3);
        glEnd();
    }
}
