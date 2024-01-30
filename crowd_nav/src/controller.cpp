#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include "crowd_nav_interfaces/TwistArray.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class Controller
{
public:
  Controller(ros::NodeHandle *nh) : current_idx{0}
  {
    nh->setParam("dt", 0.1);
    nh->setParam("n_steps", 10);

    nh->getParam("dt", dt);
    nh->getParam("n_steps", n_steps);

    // Why is this necessary?
    // std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000. * dt));

    cmd_buf_sub_ = nh->subscribe("cmd_buf", 10, &Controller::cmd_buf_cb, this);

    cmd_vel_pub_ = nh->advertise<geometry_msgs::Twist>("cmd_vel", 10);

    timer_ = nh->createTimer(ros::Duration(0.1), &Controller::timer_callback, this);
  }

private:
  double dt;
  int n_steps;
  int current_idx;
  crowd_nav_interfaces::TwistArray cmd_buff;
  ros::Timer timer_;
  ros::Subscriber cmd_buf_sub_;
  ros::Publisher cmd_vel_pub_;

  void cmd_buf_cb(const crowd_nav_interfaces::TwistArray::ConstPtr& msg)
  {
    // RCLCPP_INFO_STREAM(get_logger(), "Received buffer. Size="<<msg.twists.size());
    cmd_buff.twists = msg->twists;
    current_idx = 0;
  }

  void timer_callback(const ros::TimerEvent& event)
  {
    geometry_msgs::Twist vel;
    if ((cmd_buff.twists.size() > 0) && (current_idx < n_steps)) {
      vel = cmd_buff.twists.at(current_idx);
      current_idx += 1;
    }
    cmd_vel_pub_.publish(vel);
  }
};

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;
  Controller controller = Controller(&nh);
  ros::spin();
}