#include <ros/ros.h>
#include <tf/transform_listener.h>

using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "robot_location");

    ros::NodeHandle node;
    tf::TransformListener listener;
    ros::Rate rate(2.0);

    listener.waitForTransform("/base_footprint", "/odom", ros::Time(0),ros::Duration(10.0));

    while (ros::ok()){
        tf::StampedTransform transform;
        try {
            listener.lookupTransform("/base_footprint", "/odom", ros::Time(0),transform);
            double x = transform.getOrigin().x();
            double y = transform.getOrigin().y();
            cout << "Current position: (" << x << "," << y << ")" << endl;
        } catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
        }
        rate.sleep();
    }
    return 0;
}