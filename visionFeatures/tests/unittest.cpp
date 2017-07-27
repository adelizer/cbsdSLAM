#include <unistd.h>
#include "arucodetectorcv2.h"

#include <gtest/gtest.h>

#include <math.h>

using namespace std;

TEST(ArucoMarkersDetector, Constructor){
    arucoDetectorCV2 detector;
    EXPECT_EQ(0, detector.kill);
}

TEST(ArucoMarkersDetector, intrinsics){
    arucoDetectorCV2 detector;
    Mat params = detector.getIntrinsicParams();
    EXPECT_EQ(3, params.rows);
    EXPECT_EQ(3, params.cols);
}

TEST(ArucoMarkers, FrameData){
    arucoDetectorCV2 detector;
    Mat frame = imread("/home/adel/workspace/components/primitives/visionFeatures/tests/data/markers/marker.png");
    detector.setCurrentFrame(frame);
    EXPECT_LT(0, detector.getCurrentFrame().rows);
}

TEST(ArucoMarkersDetector, Detections){
    arucoDetectorCV2 detector;
    Mat frame = imread("/home/adel/workspace/components/primitives/visionFeatures/tests/data/markers/marker.png");
    detector.setCurrentFrame(frame);
    detector.detectMarkers();
    vector<double> x, z;
    vector<int> ids;
    detector.getDetections(x, z, ids);
    EXPECT_EQ(1, x.size());
    EXPECT_EQ(4, ids[0]);
}


int main(int argc, char **argv){
    cout << "Running unit tests for vision feature extraction" << endl;
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    return 0;
}
