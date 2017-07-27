//MRPT headers had to be included before other headers to avoid compilation errors
#include "mrptcoordinator.h"
#include "visionfeaturescoordinator.h"

#include <ros/ros.h>
#include <stdio.h>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_geometry/laser_geometry.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/ChannelFloat32.h"
#include <tf/transform_datatypes.h>

#include<math.h>
#include<cmath>
#include <iostream>
#include <vector>
#include <string>

using namespace std;

class PFNode
{
    ros::NodeHandle nh_;
    ros::Subscriber vo_sub_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber arucoCam_;

    visionFeaturesCoordinator vfCoordinator;
    mrptCoordinator ekf;

double velx, velz;
double poseX, poseY, poseTheta;
vector<double> x, y, z;
vector<int> ids;

public:
    bool kill;
    PFNode() : it_(nh_)
    {
kill = false;
         vo_sub_ = nh_.subscribe<geometry_msgs::Pose2D>("/vo_pose",1,&PFNode::voCallback, this);
	arucoCam_ = it_.subscribe("/camera/rgb/image_raw",1, &PFNode::arucoCamCallback, this);
    }
    ~PFNode(){}

    void voCallback(const geometry_msgs::Pose2D::ConstPtr& vel){
	cout << "\n Received visual odometry " << vel->x << ", " << vel->theta <<  endl;
           velx = vel->x ;
           velz = vel->theta;
      }


void arucoCamCallback(const sensor_msgs::ImageConstPtr& msg){

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        Mat img = cv_ptr->image;
vfCoordinator.setCurrentFrame(img);
vfCoordinator.runOnce();
x.clear(); x.shrink_to_fit();
z.clear(); z.shrink_to_fit();
ids.clear(); ids.shrink_to_fit();
vfCoordinator.getDetections(x, z, ids);
//vfCoordinator.visualize();
ekf.setObservations(x,y,z,ids);

}

void update(){
        ekf.setInput(velx, 0.0 , velz);
	velx = 0; velz = 0;
	ekf.runOnce();
	ekf.getPose(poseX, poseY, poseTheta);
	kill = ekf.kill;	
}
   

};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "PF_Node");
    PFNode p;
    ros::Rate loop_rate(20);
    while (ros::ok() && !p.kill){
        p.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    cout << "Exited cleanly" << endl;

    return 0;
}

