//MRPT headers had to be included before other headers to avoid compilation errors
#include "gridmapcoordinator.h"

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

class GridNode
{
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;


    GridMapCoordinator c;

public:
    bool kill;
    GridNode() {
        scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan",1,&GridNode::scanCallback, this);
    }
    ~GridNode(){}


 void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {

        int n = scan->ranges.size();
        vector<float> ranges, angles;
        vector<char> valid;
        double currentAngle = scan->angle_min;
        double anglInc = scan->angle_increment;
        for(int i = 0; i < n; i++){
            ranges.push_back(scan->ranges[i]);
            angles.push_back(currentAngle);
            currentAngle+= anglInc;
            if(scan->ranges[i] > scan->range_max || scan->ranges[i] < scan->range_min){
                valid.push_back(0);
            }else{
                valid.push_back(1);
            }
        }

        double xS, yS, thetaS;
	c.setScan(ranges, valid);
    }

void update(){
        c.runOnce();
	kill = c.kill;	
}
   

};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "Grid_Node");
    GridNode g;
    ros::Rate loop_rate(20);
    while (ros::ok() && !g.kill){
        g.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    cout << "Exited cleanly" << endl;

    return 0;
}

