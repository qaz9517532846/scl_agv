#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <boost/thread.hpp>
#include <agv_scl/agv_niryo.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void spinThread(){
  ros::spin();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "scl_agv_with_niryo");

  ros::NodeHandle n;

  boost::thread spin_thread = boost::thread(boost::bind(&spinThread));

  MoveBaseClient ac("move_base");
  ros::ServiceClient client = n.serviceClient<agv_scl::agv_niryo>("agv_connect_niryo");
  agv_scl::agv_niryo srv;

  //give some time for connections to register
  sleep(2.0);

  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 2 meters forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 0.5;
  goal.target_pose.pose.position.y = 0.5;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(1);

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 2 meters forward");
  else
    ROS_INFO("The base failed to move forward 2 meters for some reason");

  sleep(1.0);

  /*
  srv.request.agv_command = 1;

  if(client.call(srv))
  {
    ROS_INFO("respone number: %ld", (long int)srv.response.niryo_command);
    if((long int)srv.response.niryo_command == 1)
    {
      ROS_INFO("niryo and IDS camera finished.");
    }
  }
  else
  {
    ROS_ERROR("Failed to call service, check niryo robot hardware.");
    return 1;
  }
  */

  return 0;
}
