#include <manipulation.h>
#include <ros/console.h>
#include <tf_conversions/tf_eigen.h>

using namespace kinova;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_place_demo");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    kinova::Manipulation pick_place;

    geometry_msgs::PoseStamped tmp;
    tmp.pose.position.x = 0.2;
    tmp.pose.position.y = -0.4;
    tmp.pose.position.z = -0.05;

    pick_place.define_pick_pose(tmp, "HORZ", -0, 0.05);

    // pick process
    bool result = false;
    result = pick_place.pick();

    ros::spinOnce();
    return 0;
}
