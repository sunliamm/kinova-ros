#include <manipulation.h>
#include <ros/console.h>

#include <tf_conversions/tf_eigen.h>

const double FINGER_MAX = 6400;

using namespace kinova;


tf::Quaternion EulerZYZ_to_Quaternion(double tz1, double ty, double tz2)
{
    tf::Quaternion q;
    tf::Matrix3x3 rot;
    tf::Matrix3x3 rot_temp;
    rot.setIdentity();

    rot_temp.setEulerYPR(tz1, 0.0, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerYPR(0.0, ty, 0.0);
    rot *= rot_temp;
    rot_temp.setEulerYPR(tz2, 0.0, 0.0);
    rot *= rot_temp;
    rot.getRotation(q);
    return q;
}

Manipulation::Manipulation(ros::NodeHandle &nh):nh_(nh)
{
    ros::NodeHandle pn("~");

    nh_.param<std::string>("/robot_type",robot_type_,"j2n6s300");
    nh_.param<bool>("/robot_connected",robot_connected_,true);

    if (robot_connected_)
    {
        //sub_joint_ = nh_.subscribe<sensor_msgs::JointState>("/j2s7s300_driver/out/joint_state", 1, Manipulation::get_current_state, this);
        sub_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("/" + robot_type_ +"_driver/out/tool_pose", 1, &Manipulation::get_current_pose, this);
    }

    // Before we can load the planner, we need two objects, a RobotModel and a PlanningScene.
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model_ = robot_model_loader.getModel();

    // construct a `PlanningScene` that maintains the state of the world (including the robot).
    planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

    //    //  every time need retrive current robot state, do the following.
    //    robot_state::RobotState& robot_state = planning_scene_->getCurrentStateNonConst();
    //    const robot_state::JointModelGroup *joint_model_group = robot_state.getJointModelGroup("arm");

    group_ = new moveit::planning_interface::MoveGroupInterface("arm");
    gripper_group_ = new moveit::planning_interface::MoveGroupInterface("gripper");

    group_->setEndEffectorLink(robot_type_ + "_end_effector");

    finger_client_ = new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>
            ("/" + robot_type_ + "_driver/fingers_action/finger_positions", false);
    while(robot_connected_ && !finger_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the finger action server to come up");
    }

    pub_co_ = nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);
    pub_aco_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>("/attached_collision_object", 10);
    pub_planning_scene_diff_ = nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    int arm_joint_num = robot_type_[3]-'0';
    joint_names_.resize(arm_joint_num);
    joint_values_.resize(joint_names_.size());
    for (uint i = 0; i<joint_names_.size(); i++)
    {
        joint_names_[i] = robot_type_ + "_joint_" + boost::lexical_cast<std::string>(i+1);
    }
}

Manipulation::~Manipulation()
{
    // shut down pub and subs
    //sub_joint_.shutdown();
    //sub_pose_.shutdown();
    pub_co_.shutdown();
    pub_aco_.shutdown();
    pub_planning_scene_diff_.shutdown();

    // release memory
    delete group_;
    delete gripper_group_;
    delete finger_client_;
}


void Manipulation::get_current_state(const sensor_msgs::JointStateConstPtr &msg)
{
    boost::mutex::scoped_lock lock(mutex_state_);
    current_state_ = *msg;
}

void Manipulation::get_current_pose(const geometry_msgs::PoseStampedConstPtr &msg)
{
    boost::mutex::scoped_lock lock(mutex_pose_);
    current_pose_ = *msg;
}

/**
 * @brief Manipulation::gripper_action
 * @param gripper_rad close for 6400 and open for 0.0
 * @return true is gripper motion reaches the goal
 */
bool Manipulation::gripper_action(std::string command)
{
    double finger_turn;

    if(command == "open")
        finger_turn = 0.0;
    else if (command == "close")
        finger_turn = 0.75*FINGER_MAX;
    else
        std::cout << "Unknown gripper command!" << std::endl;

    if(robot_connected_ == false)
    {
        if (finger_turn>0.5*FINGER_MAX)
        {
          gripper_group_->setNamedTarget("Close");
        }
        else
        {
          gripper_group_->setNamedTarget("Open");
        }
        gripper_group_->move();
        return true;
    }

    if (finger_turn < 0)
    {
        finger_turn = 0.0;
    }
    else
    {
        finger_turn = std::min(finger_turn, FINGER_MAX);
    }

    kinova_msgs::SetFingersPositionGoal goal;
    goal.fingers.finger1 = finger_turn;
    goal.fingers.finger2 = goal.fingers.finger1;
    goal.fingers.finger3 = goal.fingers.finger1;
    finger_client_->sendGoal(goal);

    if (finger_client_->waitForResult(ros::Duration(5.0)))
    {
        finger_client_->getResult();
        return true;
    }
    else
    {
        finger_client_->cancelAllGoals();
        ROS_WARN_STREAM("The gripper action timed-out");
        return false;
    }
}

void Manipulation::build_workscene()
{
    co_.header.frame_id = "root";
    co_.header.stamp = ros::Time::now();

    // remove table
    co_.id = "table";
    co_.operation = moveit_msgs::CollisionObject::REMOVE;
    pub_co_.publish(co_);

    // add table
    co_.primitives.resize(1);
    co_.primitive_poses.resize(1);
    co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
    co_.operation = moveit_msgs::CollisionObject::ADD;

    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 2.4;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 2.4;
    co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.03;
    co_.primitive_poses[0].position.x = 0;
    co_.primitive_poses[0].position.y = 0.0;
    co_.primitive_poses[0].position.z = -0.03/2.0;
    pub_co_.publish(co_);
    planning_scene_msg_.world.collision_objects.push_back(co_);
    planning_scene_msg_.is_diff = true;
    pub_planning_scene_diff_.publish(planning_scene_msg_);
    ros::WallDuration(0.1).sleep();}


void Manipulation::define_cartesian_pose(geometry_msgs::PoseStamped request_pose, std::string grasp_type, double grasp_angle)
{
    tf::Quaternion q;

    // define grasp pose
    grasp_pose_.header.frame_id = "root";
    grasp_pose_.header.stamp  = ros::Time::now();

    // Euler_ZYZ (-M_PI/4, M_PI/2, M_PI/2)
    grasp_pose_.pose.position.x = request_pose.pose.position.x;
    grasp_pose_.pose.position.y = request_pose.pose.position.y;
    grasp_pose_.pose.position.z = request_pose.pose.position.z;

    double eular_x, eular_y, eular_z;

    if(grasp_type == "VERT")
        eular_x = M_PI;
    else if(grasp_type == "HORZ")
        eular_x = M_PI/2;
    else
        std::cout << "Incorrect grasp type!" << std::endl;

    // the inplane rotation of gripper
    eular_z = grasp_angle;

    eular_y = M_PI;

    q = EulerZYZ_to_Quaternion(eular_x, eular_y, eular_z);
    grasp_pose_.pose.orientation.x = q.x();
    grasp_pose_.pose.orientation.y = q.y();
    grasp_pose_.pose.orientation.z = q.z();
    grasp_pose_.pose.orientation.w = q.w();

    // generate_pregrasp_pose(double dist, double azimuth, double polar, double rot_gripper_z)
    grasp_pose_= generate_gripper_align_pose(grasp_pose_, 0.03999, eular_z, eular_x, eular_y);
    pregrasp_pose_ = generate_gripper_align_pose(grasp_pose_, 0.1, eular_z, eular_x, eular_y);
    postgrasp_pose_ = grasp_pose_;
    postgrasp_pose_.pose.position.z = grasp_pose_.pose.position.z + 0.05;
}


/**
 * @brief Manipulation::generate_gripper_align_pose
 * @param targetpose_msg pick/place pose (object location): where gripper close/open the fingers (grasp/release the object). Only position information is used.
 * @param dist distance of returned pose to targetpose
 * @param azimuth an angle measured from the x-axis in the xy-plane in spherical coordinates, denoted theta (0<= theta < 2pi ).
 * @param polar also named zenith, colatitude, denoted phi (0<=phi<=pi). It is the angle from the positive z-axis to the vector.  phi= pi/2 - delta where delta is the latitude.
 * @param rot_gripper_z rotation along the z axis of the gripper reference frame (last joint rotation)
 * @return a pose defined in a spherical coordinates where origin is located at the target pose. Normally it is a pre_grasp/post_realease pose, where gripper axis (last joint axis) is pointing to the object (target_pose).
 */
geometry_msgs::PoseStamped Manipulation::generate_gripper_align_pose(geometry_msgs::PoseStamped targetpose_msg, double dist, double azimuth, double polar, double rot_gripper_z)
{
    geometry_msgs::PoseStamped pose_msg;

    pose_msg.header.frame_id = "root";

    // computer pregrasp position w.r.t. location of grasp_pose in spherical coordinate. Orientation is w.r.t. fixed world (root) reference frame.
    double delta_x = -dist * cos(azimuth) * sin(polar);
    double delta_y = -dist * sin(azimuth) * sin(polar);
    double delta_z = -dist * cos(polar);

    // computer the orientation of gripper w.r.t. fixed world (root) reference frame. The gripper (z axis) should point(open) to the grasp_pose.
    tf::Quaternion q = EulerZYZ_to_Quaternion(azimuth, polar, rot_gripper_z);

    pose_msg.pose.position.x = targetpose_msg.pose.position.x + delta_x;
    pose_msg.pose.position.y = targetpose_msg.pose.position.y + delta_y;
    pose_msg.pose.position.z = targetpose_msg.pose.position.z + delta_z;
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    ROS_DEBUG_STREAM(__PRETTY_FUNCTION__ << ": LINE: " << __LINE__ << ": " << "pose_msg: x " << pose_msg.pose.position.x  << ", y " << pose_msg.pose.position.y  << ", z " << pose_msg.pose.position.z  << ", qx " << pose_msg.pose.orientation.x  << ", qy " << pose_msg.pose.orientation.y  << ", qz " << pose_msg.pose.orientation.z  << ", qw " << pose_msg.pose.orientation.w );

    return pose_msg;

}

void Manipulation::evaluate_plan(moveit::planning_interface::MoveGroupInterface &group)
{
    bool replan = true;
    int count = 0;

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    while (replan == true && ros::ok())
    {
        // reset flag for replan
        count = 0;
        result_ = false;

        // try to find a success plan.
        double plan_time;
        while (result_ == false && count < 5)
        {
            count++;
            plan_time = 20+count*10;
            ROS_INFO("Setting plan time to %f sec", plan_time);
            group.setPlanningTime(plan_time);

            moveit::planning_interface::MoveItErrorCode ret = group.plan(my_plan);
            result_ = static_cast<bool>(ret);
            std::cout << "at attemp: " << count << std::endl;
            ros::WallDuration(0.1).sleep();
        }

        // found a plan
        if (result_ == true)
        {
            std::cout << "plan success at attemp: " << count << std::endl;

            replan = false;
            std::cout << "please input e to execute the plan, r to replan, others to skip: ";

            ros::WallDuration(0.5).sleep();

            replan = false;
        }
        else // not found
        {
            std::cout << "Exit since plan failed until reach maximum attemp: " << count << std::endl;
            replan = false;
            break;
        }
    }

    if(result_ == true)
    {

        group.execute(my_plan);
  
    }
    ros::WallDuration(1.0).sleep();
}

bool Manipulation::send_home()
{
    ROS_INFO_STREAM("Send robot to home position ...");
    //std::cin >> pause_;
    group_->clearPathConstraints();
    group_->setNamedTarget("Home");
    evaluate_plan(*group_);

    return true;
}

bool Manipulation::my_pick()
{
    ros::WallDuration(1.0).sleep();
    ///////////////////////////////////////////////////////////
    //// Cartesian space without obstacle
    ///////////////////////////////////////////////////////////
    ROS_INFO_STREAM("*************************");
    ROS_INFO_STREAM("*************************");
    ROS_INFO_STREAM("*************************");
    ROS_INFO_STREAM("Motion planning in cartesian space without obstacles ...");
    //clear_workscene();
    //build_workscene();
    ros::WallDuration(0.1).sleep();

    ROS_INFO_STREAM("Planning to go to pre-grasp position ...");
    group_->setPoseTarget(pregrasp_pose_);
    evaluate_plan(*group_);

    gripper_action("open");

    ROS_INFO_STREAM("Approaching grasp position ...");
    group_->setPoseTarget(grasp_pose_);
    evaluate_plan(*group_);

    ROS_INFO_STREAM("Grasping ...");
    gripper_action("close"); // partially close


    group_->setNamedTarget("Home");
    evaluate_plan(*group_);


    ROS_INFO_STREAM("Releasing gripper ...");

    //gripper_action(0.0); // full open

    //clear_workscene();
    ROS_INFO_STREAM("Press any key to quit ...");
   // std::cin >> pause_;
    return true;
}


void Manipulation::getInvK(geometry_msgs::Pose &eef_pose, std::vector<double> &joint_value)
{
    // TODO: transform cartesian command to joint space, and alway motion plan in joint space.
}


/*int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_place_demo");
    ros::NodeHandle node;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    kinova: Manipulation pick_place(node);

    ros::spinOnce();
    return 0;
}*/
