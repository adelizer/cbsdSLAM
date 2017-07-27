#ifndef VO_UTILS_H
#define VO_UTILS_H

#include <iostream>
#include <fstream>

#include "opencv2/calib3d.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <list>
#include <vector>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

void drawMatchesRelative(const vector<KeyPoint>& train, const vector<KeyPoint>& query,
                         std::vector<cv::DMatch>& matches, Mat& img,
                         const vector<unsigned char>& mask = vector<unsigned char> ());

void keypoints2points(const vector<KeyPoint>& in, vector<Point2f>& out);
void points2keypoints(const vector<Point2f>& in, vector<KeyPoint>& out);
void warpKeypoints(const Mat& H, const vector<KeyPoint>& in, vector<KeyPoint>& out);

void matches2points(const vector<KeyPoint>& train, const vector<KeyPoint>& query,
                    const std::vector<cv::DMatch>& matches, std::vector<cv::Point2f>& pts_train,
                    std::vector<Point2f>& pts_query);
void resetH(Mat&H);
void cameraPoseFromHomography(const Mat& H, Mat& pose);
Vec3f rotationMatrixToEulerAngles(Mat &R);
bool isRotationMatrix(Mat &R);


#endif // VO_UTILS_H

