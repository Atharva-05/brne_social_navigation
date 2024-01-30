#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <crowd_nav_interfaces/GoalReq.h>

using namespace std::chrono_literals;

class PubGoal
{
public:
  PubGoal(ros::NodeHandle *nh)
  {
    goal_pose_pub_ = nh->advertise<geometry_msgs::PoseStamped>("goal_pose", 10);

    set_goal_pose_srv_ =  nh->advertiseService("set_goal_pose", &PubGoal::set_goal_pose_cb, this);
  }

private:
  ros::Publisher goal_pose_pub_;
  ros::ServiceServer set_goal_pose_srv_;

  bool set_goal_pose_cb(crowd_nav_interfaces::GoalReq::Request &req, crowd_nav_interfaces::GoalReq::Response &res)
  {
    geometry_msgs::PoseStamped gp;
    gp.header.stamp = ros::Time::now();
    gp.header.frame_id = "odom";
    gp.pose.position.x = req.x;
    gp.pose.position.y = req.y;
    goal_pose_pub_.publish(gp);
    return true;
  }
};

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "pub_goal");
  ros::NodeHandle nh;
  PubGoal pg = PubGoal(&nh);
  ros::spin();
}
