#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/stitching.hpp"

using namespace std;
using namespace cv;

bool try_use_gpu = true;
vector<Mat> imgs;
string result_name = "result.jpg";
int jumpSubsample = 2; // 2, 4, 8

int main(int argc, char *argv[]) {

    string folder = "imgsPointC";

    // append all images
    for (int i = 14; i <= 15; ++i) {
        stringstream ss;
        ss << setw(5) << setfill('0') << i;

        string path = "data/" + folder + "/" + ss.str() + ".png";
        cout << path << endl;
        Mat img = imread(path);
        if (img.empty()) {
            cout << "Can't read image '" << path << "'\n";
            return -1;
        }
        imgs.push_back(img);
    }

    Mat pano;
    Stitcher stitcher = Stitcher::createDefault(false);
    Stitcher::Status status = stitcher.stitch(imgs, pano);

    if (status != Stitcher::OK) {
        cout << "Can't stitch images, error code = " << int(status) << endl;
        return -1;
    }

    imwrite(folder + ".png", pano);

    imshow("Pano", pano);

    waitKey(-1);
    return 0;
}