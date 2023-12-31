#include <projectrobot_sim.h>

ProjectrobotSim::ProjectrobotSim()
: nh_priv_("~")
{
  bool init_result = init();
  ROS_ASSERT(init_result);
}

ProjectrobotSim: ~ProjectrobotSim()
{
}


bool ProjectrobotSim::init()
{
  wheel_seperation_ = 0;
  turning_radius_   = 0;
  robot_radius_     = 0;

  nh_.param("wheel_left_joint_name", joint_states_name_[LEFT],  std::string("wheel_left_joint"));
  nh_.param("wheel_right_joint_name", joint_states_name_[RIGHT],  std::string("wheel_right_joint"));
  nh_.param("joint_states_frame", joint_states_.header.frame_id, std::string("base_footprint"));
  nh_.param("odom_frame", odom_.header.frame_id, std::string("odom"));
  nh_.param("base_frame", odom_.child_frame_id, std::string("base_footprint"));


  wheel_speed_cmd_[LEFT]  = 0.0;
  wheel_speed_cmd_[RIGHT] = 0.0;
  goal_linear_velocity_   = 0.0;
  goal_angular_velocity_  = 0.0;
  cmd_vel_timeout_        = 1.0;
  last_position_[LEFT]    = 0.0;
  last_position_[RIGHT]   = 0.0;
  last_velocity_[LEFT]    = 0.0;
  last_velocity_[RIGHT]   = 0.0;

  double pcov[36] = {   0,   0,   0,   0,   0,   0,
                        0,   0,   0,   0,   0,   0,
                        0,   0,   0,   0,   0,   0,
                        0,   0,   0,   0,   0,   0,
                        0,   0,   0,   0,   0,   0,
                        0,   0,   0,   0,   0,   0};
  memcpy(&(odom_.pose.covariance),pcov,sizeof(double)*36);
  memcpy(&(odom_.twist.covariance),pcov,sizeof(double)*36);

  odom_pose_[0] = 0.0;
  odom_pose_[1] = 0.0;
  odom_pose_[2] = 0.0;

  odom_vel_[0] = 0.0;
  odom_vel_[1] = 0.0;
  odom_vel_[2] = 0.0;

  joint_states_.name.push_back(joint_states_name_[LEFT]);
  joint_states_.name.push_back(joint_states_name_[RIGHT]);
  joint_states_.position.resize(2,0.0);
  joint_states_.velocity.resize(2,0.0);
  joint_states_.effort.resize(2,0.0);

  joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 100);
  odom_pub_         = nh_.advertise<nav_msgs::Odometry>("odom", 100);

  cmd_vel_sub_  = nh_.subscribe("cmd_vel", 100,  &ProjectrobotSim::commandVelocityCallback, this);

  prev_update_time_ = ros::Time::now();

  return true;
}


void ProjectrobotSim::commandVelocityCallback(const geometry_msgs::TwistConstPtr cmd_vel_msg)
{
  last_cmd_vel_time_ = ros::Time::now();

  goal_linear_velocity_  = cmd_vel_msg->linear.x;
  goal_angular_velocity_ = cmd_vel_msg->angular.z;

  wheel_speed_cmd_[LEFT]  = goal_linear_velocity_ - (goal_angular_velocity_ * wheel_seperation_ / 2);
  wheel_speed_cmd_[RIGHT] = goal_linear_velocity_ + (goal_angular_velocity_ * wheel_seperation_ / 2);
}


bool ProjectrobotSim::updateOdometry(ros::Duration diff_time)
{
  double wheel_l, wheel_r; 
  double delta_s, delta_theta;
  double v[2], w[2];

  wheel_l = wheel_r     = 0.0;
  delta_s = delta_theta = 0.0;

  v[LEFT]  = wheel_speed_cmd_[LEFT];
  w[LEFT]  = v[LEFT] / WHEEL_RADIUS;  // w = v / r
  v[RIGHT] = wheel_speed_cmd_[RIGHT];
  w[RIGHT] = v[RIGHT] / WHEEL_RADIUS;

  last_velocity_[LEFT]  = w[LEFT];
  last_velocity_[RIGHT] = w[RIGHT];

  wheel_l = w[LEFT]  * diff_time.toSec();
  wheel_r = w[RIGHT] * diff_time.toSec();

  if(isnan(wheel_l))
  {
    wheel_l = 0.0;
  }

  if(isnan(wheel_r))
  {
    wheel_r = 0.0;
  }

  last_position_[LEFT]  += wheel_l;
  last_position_[RIGHT] += wheel_r;

  delta_s     = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0;
  delta_theta = WHEEL_RADIUS * (wheel_r - wheel_l) / wheel_seperation_;

  odom_pose_[0] += delta_s * cos(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[1] += delta_s * sin(odom_pose_[2] + (delta_theta / 2.0));
  odom_pose_[2] += delta_theta;

  odom_vel_[0] = delta_s / diff_time.toSec();     // v
  odom_vel_[1] = 0.0;
  odom_vel_[2] = delta_theta / diff_time.toSec(); // w

  odom_.pose.pose.position.x = odom_pose_[0];
  odom_.pose.pose.position.y = odom_pose_[1];
  odom_.pose.pose.position.z = 0;
  odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom_pose_[2]);

  odom_.twist.twist.linear.x  = odom_vel_[0];
  odom_.twist.twist.angular.z = odom_vel_[2];

  return true;
}


void ProjectrobotSim::updateJoint(void)
{
  joint_states_.position[LEFT]  = last_position_[LEFT];
  joint_states_.position[RIGHT] = last_position_[RIGHT];
  joint_states_.velocity[LEFT]  = last_velocity_[LEFT];
  joint_states_.velocity[RIGHT] = last_velocity_[RIGHT];
}

void ProjectrobotSim::updateTF(geometry_msgs::TransformStamped& odom_tf)
{
  odom_tf.header = odom_.header;
  odom_tf.child_frame_id = odom_.child_frame_id;
  odom_tf.transform.translation.x = odom_.pose.pose.position.x;
  odom_tf.transform.translation.y = odom_.pose.pose.position.y;
  odom_tf.transform.translation.z = odom_.pose.pose.position.z;
  odom_tf.transform.rotation = odom_.pose.pose.orientation;
}

bool ProjectrobotSim::update()
{
  ros::Time time_now = ros::Time::now();
  ros::Duration step_time = time_now - prev_update_time_;
  prev_update_time_ = time_now;

  if((time_now - last_cmd_vel_time_).toSec() > cmd_vel_timeout_)
  {
    wheel_speed_cmd_[LEFT]  = 0.0;
    wheel_speed_cmd_[RIGHT] = 0.0;
  }

  updateOdometry(step_time);
  odom_.header.stamp = time_now;
  odom_pub_.publish(odom_);

  updateJoint();
  joint_states_.header.stamp = time_now;
  joint_states_pub_.publish(joint_states_);

  geometry_msgs::TransformStamped odom_tf;
  updateTF(odom_tf);
  tf_broadcaster_.sendTransform(odom_tf);

  return true;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "robot_sim_node");
  Projectrobot3Sim r3sim;

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    r3sim.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
