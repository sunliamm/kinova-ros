#include <manipulation.h>
#include <ros/console.h>
#include "kinova_manipulation/Pick_Place.h"


bool pick_place(kinova_manipulation::Pick_Place::Request  &req, kinova_manipulation::Pick_Place::Response &res)
{

	kinova::Manipulation pick_place;

	req.pick_pose.pose.position.z += req.offset_plus;
	req.pick_pose.pose.position.z -= req.offset_minus;

	pick_place.define_pick_pose(req.pick_pose, req.pick_type, req.gripper_direction, req.offset_plus);

	// pick process
    bool result = false;
    result = pick_place.pick();

	ROS_INFO("request grasp pose:");
	std::cout << req.pick_pose.pose.position << std::endl;
	ROS_INFO("request grasp type:");
	std::cout << req.pick_type << std::endl;
	ROS_INFO("request gripper direction:");
	std::cout << (req.gripper_direction/M_PI) << " pi" << std::endl;

	res.result = result;
	ROS_INFO("sending back response:");
	std::cout << (result ? "true" : "false") << std::endl;

	if(req.is_place)
	{
		pick_place.define_place_pose(req.place_pose, req.pick_type, req.gripper_direction);

		result = false;
		result = pick_place.place();
	}

	if(req.is_home)
	{
		pick_place.send_home();
	}

	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Pick_Place_server");
  ros::NodeHandle n;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::ServiceServer service = n.advertiseService("/kinova_manipulation/pick_place", pick_place);
  ROS_INFO("Ready to grasp and place an object.");
  ros::waitForShutdown();

  return 0;
}