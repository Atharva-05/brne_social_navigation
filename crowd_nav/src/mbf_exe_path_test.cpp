#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/GetPathAction.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

typedef actionlib::SimpleActionClient<mbf_msgs::ExePathAction> PathClient; // A client of Exe Path Action Server

nav_msgs::Path path_;
geometry_msgs::Pose current_pose;

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg){
  path_.header.frame_id = msg->header.frame_id;
  current_pose = msg->pose.pose;
  ROS_INFO_THROTTLE(1, "%f", msg->pose.pose.position.x);
}

int main(int argc, char** argv){

    ros::init(argc, argv, "brne_waypoint_generator");
    ros::NodeHandle nh;
    current_pose.position.x = 10;
    current_pose.position.y = 10;
    current_pose.position.z = 0;
    ros::Subscriber odom_subsrciber = nh.subscribe("/jackal/odom", 10, odom_callback);

    nav_msgs::Odometry::ConstPtr wait_ptr = ros::topic::waitForMessage<nav_msgs::Odometry>("/jackal/odom", ros::Duration(5.0));
    if(wait_ptr == NULL){
      ROS_ERROR("No odometry received.");
    } else{
      ROS_INFO("Received odometry.");
    }
    PathClient pc("/jackal/move_base_flex/exe_path", true); // true doesnt need ros::spin

    while(!pc.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for Move Base server to come up");
    }

    mbf_msgs::ExePathGoal target_path_;

    int no_of_poses_in_path = 4;       // Number of poses to compose the path

    std::vector<geometry_msgs::PoseStamped> poses_(no_of_poses_in_path);

    // Fill the pose vector with zeroes 
     for (int i=0; i<no_of_poses_in_path; i++){
        memset(&poses_[i].pose.position, 0, 3);
        memset(&poses_[i].pose.orientation, 0, 4);

    }

    // Insert your series of poses here using your logic 
    // Here I am using 4 poses 10 cm apart
     for (int i=0; i<no_of_poses_in_path; i++){
        poses_[i].header.frame_id = "jackal/odom";
        poses_[i].pose.position.x = current_pose.position.x + 2*i;
        ROS_INFO("Pose populated %f %f %f", current_pose.position.x, current_pose.position.y, current_pose.position.z);
        poses_[i].pose.position.y = current_pose.position.y;
        poses_[i].pose.orientation.z = current_pose.position.z; 
        poses_[i].pose.orientation.w = 1;    
        poses_[i].header.stamp = ros::Time::now();

     }

   // Populate the Path Message
    path_.header.frame_id = "jackal/odom";
    path_.poses = poses_;
    path_.header.stamp = ros::Time::now();

    ROS_INFO("Goal Path Planned %f %f %f %f", path_.poses[0].pose.position.x, path_.poses[1].pose.position.x, 
                                         path_.poses[2].pose.position.x, path_.poses[3].pose.position.x);

   // Populate the controller and path fields (Refer to ExePath.Action)

    target_path_.controller = "TebLocalPlannerROS"; // As defined in list of controllers in your yaml

    target_path_.path = path_;

  // Interact with Action Server

  pc.sendGoal(target_path_);

  pc.waitForResult();
  if(pc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Base moved %s", pc.getState().toString().c_str());

  else if(pc.getState() == actionlib::SimpleClientGoalState::ABORTED)  
    ROS_INFO("Goal aborted");

  else 
    ROS_INFO("Base failed to move for some reason %s, ", pc.getState().toString().c_str());


  ros::spin();
}