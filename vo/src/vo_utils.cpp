#include "vo_utils.h"


void drawMatchesRelative(const vector<KeyPoint> &train, const vector<KeyPoint> &query, std::vector<DMatch> &matches, Mat &img, const vector<unsigned char> &mask){
    for (int i = 0; i < (int)matches.size(); i++){
        if (mask.empty() || mask[i]){
            Point2f pt_new = query[matches[i].queryIdx].pt;
            Point2f pt_old = train[matches[i].trainIdx].pt;

            cv::line(img, pt_new, pt_old, Scalar(125, 255, 125), 1);
            cv::circle(img, pt_new, 2, Scalar(255, 0, 125), 1);
        }
    }
}

void keypoints2points(const vector<KeyPoint> &in, vector<Point2f> &out){
    out.clear();
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i){
        out.push_back(in[i].pt);
    }
}

void points2keypoints(const vector<Point2f> &in, vector<KeyPoint> &out){
    out.clear();
    out.reserve(in.size());
    for (size_t i = 0; i < in.size(); ++i){
        out.push_back(KeyPoint(in[i], 1));
    }
}

void warpKeypoints(const Mat &H, const vector<KeyPoint> &in, vector<KeyPoint> &out){
    vector<Point2f> pts;
    keypoints2points(in, pts);
    vector<Point2f> pts_w(pts.size());
    Mat m_pts_w(pts_w);
    perspectiveTransform(Mat(pts), m_pts_w, H);
    points2keypoints(pts_w, out);
}


void matches2points(const vector<KeyPoint> &train, const vector<KeyPoint> &query, const std::vector<DMatch> &matches, std::vector<Point2f> &pts_train, std::vector<Point2f> &pts_query){
    pts_train.clear();
    pts_query.clear();
    pts_train.reserve(matches.size());
    pts_query.reserve(matches.size());

    size_t i = 0;

    for (; i < matches.size(); i++){
        const DMatch & dmatch = matches[i];
        pts_query.push_back(query[dmatch.queryIdx].pt);
        pts_train.push_back(train[dmatch.trainIdx].pt);
    }
}

void resetH(Mat &H){ // set H matrix to eye
    H = Mat::eye(3, 3, CV_32FC1);
}

void cameraPoseFromHomography(const Mat& H, Mat& pose){
    pose = Mat::eye(3, 4, CV_64FC1); //3x4 matrix
    float norm1 = (float)norm(H.col(0));
    float norm2 = (float)norm(H.col(1));
    float tnorm = (norm1 + norm2) / 2.0f;

    Mat v1 = H.col(0);
    Mat v2 = pose.col(0);

    cv::normalize(v1, v2); // Normalize the rotation

    v1 = H.col(1);
    v2 = pose.col(1);

    cv::normalize(v1, v2);

    v1 = pose.col(0);
    v2 = pose.col(1);

    Mat v3 = v1.cross(v2);  //Computes the cross-product of v1 and v2
    Mat c2 = pose.col(2);
    v3.copyTo(c2);

    pose.col(3) = H.col(2) / tnorm; //vector t [R|t]
}

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;

    return  fabs(determinant(shouldBeIdentity) - 1) < 0.1;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3f rotationMatrixToEulerAngles(Mat &R)
{
    float x, y, z;

    if(isRotationMatrix(R)){
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );
    bool singular = sy < 1e-6; // If
    if (!singular) {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else{
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    }else{
        cout <<  "R is not a rotation matrix" << endl;
        x = y = z = 0;
    }
    return Vec3f(x, y, z);
}
