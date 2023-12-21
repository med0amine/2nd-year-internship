#include "projectrobot_drive.h"

ProjectrobotDrive::ProjectrobotDrive()
  : nh_priv_("~")
{
  auto ret = init();
  ROS_ASSERT(ret);
}

ProjectrobotDrive::~ProjectrobotDrive()
{
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}


bool ProjectrobotDrive::init()
{
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");

  escape_range_       = 40.0 * DEG2RAD;
  check_forward_dist_ = 0.6;
  check_side_dist_    = 0.5;

  tb3_pose_ = 0.0;
  prev_tb3_pose_ = 0.0;

  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

  laser_scan_sub_  = nh_.subscribe("scan", 10, &ProjectrobotDrive::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &ProjectrobotDrive::odomMsgCallBack, this);

  return true;
}

void ProjectrobotDrive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);  

	tb3_pose_ = atan2(siny, cosy);
}

void ProjectrobotDrive::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  uint16_t scan_angle[4] = {0, 40, 350};

  for (int num = 0; num < 4; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      scan_data_[num] = msg->range_max;
    }
    else
    {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}

void ProjectrobotDrive::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}

bool ProjectrobotDrive::controlLoop()
{
  static uint8_t robot_state_num = 0;

  switch(robot_state_num)
  {
    case GET_TB3_DIRECTION:
      if (scan_data_[CENTER] > check_forward_dist_)
      {
        if (scan_data_[LEFT] < check_side_dist_)
        {
          prev_tb3_pose_ = tb3_pose_;
          robot_state_num = TB3_RIGHT_TURN;
        }
        else if (scan_data_[RIGHT] < check_side_dist_)
        {
          prev_tb3_pose_ = tb3_pose_;
          robot_state_num = TB3_LEFT_TURN;
        }
        else
        {
          robot_state_num = TB3_DRIVE_FORWARD;
        }
      }

      if (scan_data_[CENTER] < check_forward_dist_)
      {
        prev_tb3_pose_ = tb3_pose_;
        robot_state_num = TB3_RIGHT_TURN;
      }
      break;

    case TB3_DRIVE_FORWARD:
      updatecommandVelocity(LINEAR_VELOCITY, 0.0);
      robot_state_num = GET_TB3_DIRECTION;
      break;

    case TB3_RIGHT_TURN:
      if (fabs(prev_tb3_pose_ - tb3_pose_) >= escape_range_)
        robot_state_num = GET_TB3_DIRECTION;
      else
        updatecommandVelocity(0.0, -1 * ANGULAR_VELOCITY);
      break;

    case TB3_LEFT_TURN:
      if (fabs(prev_tb3_pose_ - tb3_pose_) >= escape_range_)
        robot_state_num = GET_TB3_DIRECTION;
      else
        updatecommandVelocity(0.0, ANGULAR_VELOCITY);
      break;

    default:
      robot_state_num = GET_TB3_DIRECTION;
      break;
  }

  return true;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "turtlebot3_drive");
  ProjectrobotDrive projectrobot_drive;

  ros::Rate loop_rate(150);

  while (ros::ok())
  {
    projectrobot_drive.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
