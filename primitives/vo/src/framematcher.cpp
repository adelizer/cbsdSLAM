#include "framematcher.h"

FrameMatcher::FrameMatcher(){
    desc_matcher = BFMatcher((brief->defaultNorm()));
    detector = FastFeatureDetector::create(10, true);
    pathFrame = Mat(500, 500, CV_8UC3, Scalar(255,255,255));
    H_prev = Mat::eye(3, 3, CV_32FC1);
    kill = false;
    initDone = false;
    displayImage = false;
    config();
}

FrameMatcher::~FrameMatcher(){
    // save path to txt file
    ofstream f ("./../logging/path.txt");
    for(int i = 0; i < pathX.size(); i++) {
        f << pathX[i] << ", " << pathY[i] << ", " << pathTheta[i] << '\n';
    }
    f.close();
}

void FrameMatcher::config(){
    configFileName = "../config/config.yaml";
    intrinsicParams = Mat::zeros(3, 3, CV_32F);
    distortionCoeff = Mat::zeros(4,1,CV_32F);
    FileStorage fs;
    fs.open(configFileName, FileStorage::READ);
    if (!fs.isOpened()){
        cerr << "Failed to open " << configFileName << endl;
        cerr << "Proceeding with kinect rgb camera default values " << endl;
        intrinsicParams.at<float>(0,0) = 534.745346 ; // fx
        intrinsicParams.at<float>(0,1) = 0.0;
        intrinsicParams.at<float>(0,2) =  311.883701 ; // cx
        intrinsicParams.at<float>(1,0) = 0.0;
        intrinsicParams.at<float>(1,1) = 534.464055; // fy
        intrinsicParams.at<float>(1,2) =  255.480002; // cy
        intrinsicParams.at<float>(2,2) = 1;
    }else{
        fs["cameraMatrix"] >> intrinsicParams;
        FileNode n = fs["Extra"];
        int tempDisp = (int)(n["display"]);
        displayImage = (bool)(tempDisp);
        cout << "Configuration file opened successfully " << endl;
        cout << intrinsicParams << endl;
    }
    fs.release();
    // reset pose variables
    x = 0; y = 0; z = 0; theta = 0;
}

void FrameMatcher::setCurrentFrame(const Mat &frame){
    Mat gray;
    if(frame.channels() > 2)
        cvtColor(frame, gray, CV_BGR2GRAY);
    else
        gray = frame.clone();

    undistort(gray, currentFrame, intrinsicParams, distortionCoeff);
    //    currentFrame = gray.clone();
    dispFrame = currentFrame.clone();
}

void FrameMatcher::runVO(){
    detector->detect(currentFrame, query_kpts); //Find interest points using FAST
    brief->compute(currentFrame, query_kpts, query_desc); // compute descriptors
    if (!train_kpts.empty()){ // if previous keypoints exist
        vector<KeyPoint> test_kpts;
        warpKeypoints(H_prev.inv(), query_kpts, test_kpts);
        desc_matcher.match(query_desc, train_desc, matches, Mat());
        //drawKeypoints(dispFrame, test_kpts, dispFrame, Scalar(255, 0, 0), DrawMatchesFlags::DRAW_OVER_OUTIMG);
        matches2points(train_kpts, query_kpts, matches, train_pts, query_pts);
        if(matches.size() > 5){
            Mat H = findHomography(train_pts, query_pts, RANSAC, 4, match_mask);
            if (countNonZero(Mat(match_mask)) < 15){
                H_prev = H;
            }else{
                resetH(H_prev);
                drawMatchesRelative(train_kpts, query_kpts, matches, dispFrame, match_mask);
                Mat pose;
                cameraPoseFromHomography(H, pose);

                cv::Mat submat = cv::Mat(H, cv::Rect(0, 0, 3, 3));
                Vec3f angles = rotationMatrixToEulerAngles(submat);
                theta = theta + angles[2];
                x = x + cos(theta) * pose.at<double>(0,3);
                y = y + sin(theta) * pose.at<double>(1,3);
                z = z + pose.at<double>(2,3);
                // *3.5 / 990.0 * 50
                int px,py;
                px =round(x * 50 * 3.5 / 990.0) + 600;
                py = round(y * 50 * 3.5 / 990.0) + 250;
                cout << px << "," << py << endl;
                pathFrame.at<Vec3f>(py,px)[0] = 0;
                pathFrame.at<Vec3f>(py,px)[1] = 0;
                pathFrame.at<Vec3f>(py,px)[2] = 255;
                pathX.push_back(x);
                pathY.push_back(y);
                pathTheta.push_back(theta);
            }
        }else{
            resetH(H_prev);
        }
    }else{ // no previous data
        H_prev = Mat::eye(3, 3, CV_32FC1);
        Mat out;
        drawKeypoints(currentFrame, query_kpts, out);
        dispFrame = out;
    }
    train_kpts = query_kpts;
    query_desc.copyTo(train_desc);
    resetH(H_prev);

}

void FrameMatcher::printInfo(){
    cout << " X: " << x << ", ";
    cout << " Y: "<< y  << ", ";
    cout << " Theta: " << theta * 180.0 / M_PI << endl;

}

void FrameMatcher::visualize(){
    if(!displayImage)
        return;
    imshow("", dispFrame);
    imshow("path", pathFrame);
    char c = waitKey(1);
    if((int)c == 27) // Esc
        kill = true;
}


