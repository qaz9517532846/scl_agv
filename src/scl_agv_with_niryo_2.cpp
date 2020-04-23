#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <boost/thread.hpp>
#include <scl_agv/agv_niryo.h>
#include <scl_agv/vision_feedback.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

double feedback_x, feedback_y, feedback_theta;
ros::Publisher change_local;

class agv_niryo_service {
public:
    ros::NodeHandle n;
    ros::ServiceClient client; 

    void service_client(int agv_command);
};

void change_localization(int method)
{
  agv_niryo_service service;
  std_msgs::Int32 localization;
  localization.data = method;
  ros::Rate loop_rate(10);
  for(int i = 0; i < 3; i++)
  {
     change_local.publish(localization);
     loop_rate.sleep();
  }
}

void agv_move_map(double x, double y, double theta)
{
  MoveBaseClient ac("move_base");

  //give some time for connections to register
  sleep(2.0);

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 2 meters forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base move.");
  else
    ROS_INFO("The base failed to move.");
}

void agv_move_baselink(double x, double y, double theta)
{
  MoveBaseClient ac("move_base");

  //give some time for connections to register
  sleep(2.0);

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 2 meters forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved.");
  else
    ROS_INFO("The base failed to move.");
}

void chatterCallback(const scl_agv::vision_feedback::ConstPtr& msg)
{
  feedback_x = msg->x - 0.5;
  feedback_y = msg->y + 0.1;
  feedback_theta = msg->theta;
}

void agv_niryo_service::service_client(int agv_command)
{
  agv_niryo_service service;
  scl_agv::agv_niryo srv;
  srv.request.agv_command = agv_command;
  if(client.call(srv))
  {
    ROS_INFO("respone number: %ld", (long int)srv.response.niryo_command);
    if((long int)srv.response.niryo_command == 1)
    {
      change_localization(1);
      ROS_INFO("first Localization.");
      ros::Subscriber sub = service.n.subscribe("feedback", 1000, chatterCallback);
      ros::Rate loop_rate(1);
      for(int i = 0; i < 3; i++)
      {
         ros::spinOnce();
         loop_rate.sleep();
      }
      ROS_INFO("I heard feedback x = %f", feedback_x);
      ROS_INFO("I heard feedback y = %f", feedback_y);
      ROS_INFO("I heard feedback theta = %f", feedback_theta);
      sleep(1.0);
      agv_move_baselink(feedback_x, feedback_y, feedback_theta);
    }
    else if((long int)srv.response.niryo_command == 2)
    {
      ROS_INFO("second Localization.");
    }
  }
  else
  {
    ROS_ERROR("Failed to call service, check niryo robot hardware.");
  }
}

void spinThread(){
  ros::spin();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "scl_agv_with_niryo_2");  
  agv_niryo_service service;

  boost::thread spin_thread = boost::thread(boost::bind(&spinThread));

  service.client = service.n.serviceClient<scl_agv::agv_niryo>("agv_connect_niryo");
  change_local = service.n.advertise<std_msgs::Int32>("change_localization", 1000);

  //agv_move_map(-2.8, -23, 0);

  sleep(1.0);
  
  service.service_client(2);
  //service.service_client(2);

  return 0;
}
