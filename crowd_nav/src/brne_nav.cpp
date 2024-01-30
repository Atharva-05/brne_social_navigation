#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <armadillo>
#include <std_msgs/String.h>
#include <crowd_nav_interfaces/PedestrianArray.h>
#include <crowd_nav_interfaces/TwistArray.h>
#include <crowd_nav_interfaces/GoalReq.h>
#include <pedsim_msgs/SemanticData.h>
#include <brnelib/brne.hpp>
#include <nav_msgs/Path.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

// MBF integration
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/GetPathAction.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <nav_msgs/Path.h>
typedef actionlib::SimpleActionClient<mbf_msgs::ExePathAction> PathClient; // A client of Exe Path Action Server

using namespace std::chrono_literals;

struct RobotPose
{
  double x;
  double y;
  double theta;
  arma::rowvec toVec()
  {
    return arma::rowvec(std::vector<double>{x, y, theta});
  }
};

double dist(double x1, double y1, double x2, double y2)
{
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

class PathPlan{
  public:
    PathPlan(ros::NodeHandle *nh): goal_set{false}, walls_generated{false}
    {
      //->get parameters
      ros::param::param("dt", dt, 0.1);
      ros::param::param("maximum_agents", maximum_agents, 8);
      ros::param::param("replan_freq", replan_freq, 10.0);
      ros::param::param("n_samples", n_samples, 196);
      ros::param::param("n_steps", n_steps, 15);
      ros::param::param("cost_a1", cost_a1, 4.0);
      ros::param::param("cost_a2", cost_a2, 1.0);
      ros::param::param("cost_a3", cost_a3, 80.0);
      ros::param::param("kernel_a1", kernel_a1, 0.2);
      ros::param::param("kernel_a2", kernel_a2, 0.2);
      ros::param::param("y_min", y_min, -5.0);
      ros::param::param("y_max", y_max, 5.0);
      ros::param::param("people_timeout", people_timeout, 5.0);
      ros::param::param("goal_threshold", goal_threshold, 0.5);
      ros::param::param("brne_activate_threshold", brne_activate_threshold, 3.5);
      ros::param::param("max_lin_vel", max_lin_vel, 0.6);
      ros::param::param("nominal_lin_vel", nominal_lin_vel, 0.4);
      ros::param::param("max_ang_vel", max_ang_vel, 0.5);
      ros::param::param("close_stop_threshold", close_stop_threshold, 0.5);  
      ros::param::param("offset_unitree_vel", offset_unitree_vel, false);
      ros::param::param<std::string>("pedestrian_position_topic", _pedestrian_position_topic, "/pedsim_agents/semantic/pedestrian");
      ros::param::param<std::string>("odom_topic", _odom_topic, "/jackal/odom");
      ros::param::param<std::string>("goal_topic", _goal_topic, "/jackal/current_goal");
      ros::param::param<std::string>("planning_frame", _planning_frame, "map");

      brne = brne::BRNE{kernel_a1, kernel_a2,
        cost_a1, cost_a2, cost_a3,
        dt, n_steps, n_samples,
        y_min, y_max};

      trajgen = brne::TrajGen{max_lin_vel, max_ang_vel, n_samples, n_steps, dt};
      ROS_INFO("Y-Axis wall bounds: %f, %f", y_min, y_max);

      // Define publishers and subscribers
      goal_sub_ = nh->subscribe(_goal_topic, 10, &PathPlan::goal_cb, this);
      // gazebo_model_states_sub_ = nh->subscribe("gazebo/model_states_throttle", 33, &PathPlan::gazebo_model_states_cb, this);
      pedestrians_sub_ = nh->subscribe(_pedestrian_position_topic, 33, &PathPlan::pedestrian_pos_cb, this);
      odom_sub_ = nh->subscribe(_odom_topic, 10, &PathPlan::odom_cb, this);
        
      cmd_buf_pub_ = nh->advertise<crowd_nav_interfaces::TwistArray>("cmd_buf", 10);
      path_pub_ = nh->advertise<nav_msgs::Path>("optimal_path", 10);
      walls_pub_ = nh->advertise<visualization_msgs::MarkerArray>("walls", 10);


      set_goal_pose_srv_ =  nh->advertiseService("set_goal_pose", &PathPlan::set_goal_pose_cb, this);

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      // Create a timer that executes at replan_freq
      std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000. / replan_freq));
      timer_ = nh->createTimer(ros::Duration(1.0 / replan_freq), &PathPlan::timer_callback, this);
      
      // optimal_path.header.frame_id = _planning_frame;
      optimal_path.header.frame_id = _planning_frame;
    }

  private:
    double replan_freq, kernel_a1, kernel_a2, cost_a1, cost_a2, cost_a3, y_min, y_max, dt,
      max_ang_vel, max_lin_vel, people_timeout, goal_threshold, brne_activate_threshold,
      nominal_lin_vel, close_stop_threshold;
    int maximum_agents, n_samples, n_steps;
    std::string _odom_topic, _planning_frame, _pedestrian_position_topic, _goal_topic;

    brne::BRNE brne{};
    brne::TrajGen trajgen{};

    ros::Timer timer_;

    crowd_nav_interfaces::PedestrianArray ped_buffer;
    crowd_nav_interfaces::PedestrianArray selected_peds;

    crowd_nav_interfaces::TwistArray robot_cmds;

    nav_msgs::Path optimal_path;
    bool estop_triggered;

    RobotPose robot_pose;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    ros::Publisher cmd_buf_pub_;
    ros::Publisher path_pub_;
    ros::Publisher walls_pub_;
    ros::Subscriber goal_sub_ ;
    ros::Subscriber gazebo_model_states_sub_;
    ros::Subscriber pedestrians_sub_;
    ros::Subscriber odom_sub_;
    ros::ServiceServer set_goal_pose_srv_;
    visualization_msgs::MarkerArray wall_markers;

    bool goal_set;
    bool walls_generated;
    geometry_msgs::PoseStamped goal;

    // trial information
    double trial_closest_dst_to_ped;
    double trial_path_length;
    RobotPose trial_start_pose;
    double trial_straight_line_length;
    double trial_path_ratio;
    ros::Time trial_start;
    int trial_n_estops;

    bool offset_unitree_vel;

    std::vector<geometry_msgs::PoseStamped> goal_poses_;
    nav_msgs::Path path_;

    bool set_goal_pose_cb(crowd_nav_interfaces::GoalReq::Request &req, crowd_nav_interfaces::GoalReq::Response &res)
    {
      goal.header.stamp = ros::Time::now();
      goal.header.frame_id = _planning_frame;
      goal.pose.position.x = req.x;
      goal.pose.position.y = req.y;
      // goal_pose_pub_.publish(gp);
      ROS_INFO_STREAM("Goal Received: " << req.x << ", " << req.y);
      goal_set = true;
      check_goal();
      trial_start = ros::Time::now();
      trial_start_pose = robot_pose;
      trial_path_length = 0;
      trial_closest_dst_to_ped = 10000;
      trial_n_estops = 0;
      return true;
    }
    
    void pub_walls()
    {
      if (!walls_generated){
        const auto height = 1.0;
        const auto length = 20.0;
        const auto thickness = 0.01;
        const auto transparency = 0.2;
        for (int i = 0; i < 2; i++) {
          visualization_msgs::Marker wall;
          wall.header.frame_id = _planning_frame;
          wall.id = i;
          wall.type = 1;   // cube
          wall.action = 0;
          wall.pose.position.x = 0.5 * length - 1.0;
          wall.pose.position.z = 0.5 * height;
          wall.color.a = transparency;
          wall.color.b = 1.0;
          wall.scale.x = length;
          wall.scale.y = thickness;
          wall.scale.z = height;
          wall_markers.markers.push_back(wall);
        }
        wall_markers.markers.at(0).pose.position.y = y_min;
        wall_markers.markers.at(1).pose.position.y = y_max;
        walls_generated = false;    // Temporary setting since rviz may not be initialized before starting
      }
      const auto now = ros::Time::now();
      wall_markers.markers.at(0).header.stamp = now;
      wall_markers.markers.at(1).header.stamp = now;
      // Visualize goal
      visualization_msgs::Marker goal_marker;
      goal_marker.header.frame_id = _planning_frame;
      goal_marker.id = 2;
      goal_marker.action = 0;
      goal_marker.type = visualization_msgs::Marker::SPHERE;
      goal_marker.pose = goal.pose;
      goal_marker.color.a = 1.0;
      goal_marker.color.g = 1.0;
      goal_marker.scale.x = 0.4;
      goal_marker.scale.y = 0.4;
      goal_marker.scale.z = 0.1;
      wall_markers.markers.push_back(goal_marker);

      walls_pub_.publish(wall_markers);
    }
    
    void goal_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
      if (!check_new_goal(msg)){
        return;
      } else{
        ROS_INFO_STREAM("New Goal Received: " << msg->pose.position.x << ", " << msg->pose.position.y);
        goal_set = true;
        goal.pose = msg->pose;
        goal.pose.position.x = msg->pose.position.x;
        check_goal();
        trial_start = ros::Time::now();
        trial_start_pose = robot_pose;
        trial_path_length = 0;
        trial_closest_dst_to_ped = 10000;
        trial_n_estops = 0;
      }
    }

    bool check_new_goal(const geometry_msgs::PoseStamped::ConstPtr &msg){
      // TODO: orientation check
      double NEW_GOAL_TOLERANCE = 0.01;
      bool is_x_new = abs(msg->pose.position.x - goal.pose.position.x) > NEW_GOAL_TOLERANCE;
      bool is_y_new = abs(msg->pose.position.y - goal.pose.position.y) > NEW_GOAL_TOLERANCE;
      return is_x_new | is_y_new;

    }

    void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
    {
      if (goal_set){
        trial_path_length += dist(robot_pose.x, robot_pose.y, msg->pose.pose.position.x, msg->pose.pose.position.y);
      }
      // get the angle from the quaternion
      tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      robot_pose.x = msg->pose.pose.position.x;
      robot_pose.y = msg->pose.pose.position.y;
      robot_pose.theta = yaw;
      if (goal_set) {
        check_goal();
      }
    }

    void pedestrian_pos_cb(const pedsim_msgs::SemanticData::ConstPtr &msg){

      int n_peds = msg->points.size();
      if(n_peds < 1){
        ROS_WARN("No pedestrians found.");
        return;
      } else {
          // Pedestrian(s) found
          ROS_INFO_THROTTLE(5, "%d pedestrians detected.", n_peds);
          crowd_nav_interfaces::PedestrianArray ped_array;
          ped_array.header.frame_id = _planning_frame;
          ped_array.header.stamp = ros::Time::now();

          for (int i = 0; i < n_peds; i++) {
              crowd_nav_interfaces::Pedestrian ped_i;
              ped_i.id = i;
              ped_i.pose.position = msg->points[i].location;
              ped_array.pedestrians.push_back(ped_i);
          }

          for (int p = 0; p < n_peds; p++) {
            auto ped = ped_array.pedestrians.at(p);
            // add to pedestrian buffer
            // first have to do some error checking to make sure the index is in bounds
            // fix time synchronization issues
            ped.header.stamp = ped_array.header.stamp;
            while (static_cast<int>(ped_buffer.pedestrians.size()) < static_cast<int>(ped.id + 1)) {
              crowd_nav_interfaces::Pedestrian blank_ped;
              blank_ped.id = ped_buffer.pedestrians.size();
              ped_buffer.pedestrians.push_back(blank_ped);
            }
            ped_buffer.pedestrians.at(ped.id) = ped;
          }
      }

    }

    void check_goal()
    {
      const auto dist_to_goal =
        dist(robot_pose.x, robot_pose.y, goal.pose.position.x, goal.pose.position.y);
      if (dist_to_goal < goal_threshold) {
        const auto trial_end = ros::Time::now();;
        ROS_INFO_STREAM( "Goal Reached!");
        const auto trial_dt = trial_end - trial_start;
        trial_straight_line_length = dist(trial_start_pose.x, trial_start_pose.y, robot_pose.x, robot_pose.y);
        ROS_INFO_STREAM( "=========================================================");
        ROS_INFO_STREAM("Straight Line Path: "<<trial_straight_line_length << " m");
        ROS_INFO_STREAM( "Trial Path: " <<trial_path_length << " m");
        ROS_INFO_STREAM( "Path Ratio: " << trial_path_length/trial_straight_line_length);
        ROS_INFO_STREAM( "Closest Dist to Ped: " << trial_closest_dst_to_ped << " m");
        ROS_INFO_STREAM( "Number of E-STOPs: " << trial_n_estops);
        ROS_INFO_STREAM( "=========================================================");
        goal_set = false;
      }
    }

    void pedestrians_cb(const crowd_nav_interfaces::PedestrianArray::ConstPtr & msg)
    {
      const auto curr_ped_stamp = ros::Time::now();;
      const int n_peds = msg->pedestrians.size();
      // iterate through pedestrians
      for (int p = 0; p < n_peds; p++) {
        auto ped = msg->pedestrians.at(p);
        // add to pedestrian buffer
        // first have to do some error checking to make sure the index is in bounds
        // fix time synchronization issues
        ped.header.stamp = curr_ped_stamp;
        while (static_cast<int>(ped_buffer.pedestrians.size()) < static_cast<int>(ped.id + 1)) {
          crowd_nav_interfaces::Pedestrian blank_ped;
          blank_ped.id = ped_buffer.pedestrians.size();
          ped_buffer.pedestrians.push_back(blank_ped);
        }
        ped_buffer.pedestrians.at(ped.id) = ped;
      }
    }

    void timer_callback(const ros::TimerEvent& event){
      // ROS_INFO("Timer fired");
      // record time at start for benchmarking speed
      const auto start = ros::Time::now();
      
      // Flag to validate if valid path is found
      estop_triggered = true;

      // clear leftover messages
      robot_cmds.twists.clear();
      selected_peds.pedestrians.clear();

      // record time in seconds to compare to pedestrian stamp
      const ros::Time current_timestamp = ros::Time::now();;
      const auto current_time_sec = current_timestamp.sec + 1e-9 * current_timestamp.nsec;

      // find pedestrians to interact with from pedestrian buffer
      std::vector<double> dists_to_peds;
      for (auto p:ped_buffer.pedestrians) {
        auto ped_time_sec = p.header.stamp.sec + 1e-9 * p.header.stamp.nsec;
        auto dt = current_time_sec - ped_time_sec;
        // don't consider this pedestrian if it came in too long ago.
        if (dt > people_timeout) {
          continue;
        }
        // compute distance to the pedestrian from the robot
        auto dist_to_ped = dist(robot_pose.x, robot_pose.y, p.pose.position.x, p.pose.position.y);
        // dont' consider this pedestrian if it is too far away
        if (dist_to_ped > brne_activate_threshold) {
          continue;
        }
        // pedestrian has been selected
        dists_to_peds.push_back(dist_to_ped);
        selected_peds.pedestrians.push_back(p);
      }

      const auto n_peds = static_cast<int>(selected_peds.pedestrians.size());
      const auto n_agents = std::min(maximum_agents, n_peds + 1);

      // ROS_INFO("%d pedestrians detected", n_peds);
      // grab the goal. if there is no goal set, visualize a "fake" goal at (10,0) which 
      // should be a straight trajectory forward
      arma::rowvec goal_vec;
      if (goal_set) {
        goal_vec = arma::rowvec(
          std::vector<double>{goal.pose.position.x,
            goal.pose.position.y});
      } else {
        goal_vec = arma::rowvec(std::vector<double>{10.0, 0.0});
      }

      // get the controls to go to the goal assuming a diff drive model
      auto theta_a = robot_pose.theta;
      if (robot_pose.theta > 0.0) {
        theta_a -= M_PI_2;
      } else {
        theta_a += M_PI_2;
      }
      const arma::rowvec axis_vec(std::vector<double>{cos(theta_a), sin(theta_a)});
      const arma::rowvec pose_vec(std::vector<double>{robot_pose.x, robot_pose.y});
      const arma::rowvec vec_to_goal = goal_vec - pose_vec;
      const auto dist_to_goal = arma::norm(vec_to_goal);
      const auto proj_len =
        arma::dot(axis_vec, vec_to_goal) / arma::dot(vec_to_goal, vec_to_goal) * dist_to_goal;
      const auto radius = 0.5 * dist_to_goal / proj_len;
      // find nominal linear and angular velocity for diff drive model
      double nominal_ang_vel = 0;
      if (robot_pose.theta > 0.0) {
        nominal_ang_vel = -nominal_lin_vel / radius;
      } else {
        nominal_ang_vel = nominal_lin_vel / radius;
      }

      // do control-space sampling for the robot given these nominal commands
      const auto traj_samples = trajgen.traj_sample(nominal_lin_vel, nominal_ang_vel, robot_pose.toVec());

      if (n_agents > 1) {
        // create pedestrian samples
        // ROS_INFO("%d pedestrians detected", n_agents - 1);
        const auto x_pts = brne.mvn_sample_normal(n_agents - 1);
        const auto y_pts = brne.mvn_sample_normal(n_agents - 1);

        // Will need to fill these in with samples of the robot and pedestrian
        arma::mat xtraj_samples(n_agents * n_samples, n_steps, arma::fill::zeros);
        arma::mat ytraj_samples(n_agents * n_samples, n_steps, arma::fill::zeros);

        // pick only the closest pedestrians to interact with
        const auto closest_idxs =
          arma::conv_to<arma::vec>::from(arma::sort_index(arma::vec(dists_to_peds)));
        // iterate through pedestrians, closest to farthest
        for (int p = 0; p < (n_agents - 1); p++) {
          auto ped = selected_peds.pedestrians.at(closest_idxs.at(p));
          arma::vec ped_vel(std::vector<double>{ped.velocity.linear.x, ped.velocity.linear.y});
          auto speed_factor = arma::norm(ped_vel);
          arma::rowvec ped_xmean = arma::rowvec(n_steps, arma::fill::value(ped.pose.position.x)) +
            arma::linspace<arma::rowvec>(0, (n_steps - 1), n_steps) * dt * ped.velocity.linear.x;
          arma::rowvec ped_ymean = arma::rowvec(n_steps, arma::fill::value(ped.pose.position.y)) +
            arma::linspace<arma::rowvec>(0, (n_steps - 1), n_steps) * dt * ped.velocity.linear.y;
          arma::mat ped_xmean_mat(n_samples, n_steps, arma::fill::zeros);
          arma::mat ped_ymean_mat(n_samples, n_steps, arma::fill::zeros);
          ped_xmean_mat.each_row() = ped_xmean;
          ped_ymean_mat.each_row() = ped_ymean;
          // set submatrix in xtraj and ytraj samples.
          // submatrix ((p+1)*nsamples, (p+2)*nsamples) = xpoints(p*nsamples, (p+1)*nsamples) * speed_factor + ped_xmean
          xtraj_samples.submat((p + 1) * n_samples, 0, (p + 2) * n_samples - 1, n_steps - 1) =
            x_pts.submat(
            p * n_samples, 0, (p + 1) * n_samples - 1,
            n_steps - 1) * speed_factor + ped_xmean_mat;
          ytraj_samples.submat((p + 1) * n_samples, 0, (p + 2) * n_samples - 1, n_steps - 1) =
            y_pts.submat(
            p * n_samples, 0, (p + 1) * n_samples - 1,
            n_steps - 1) * speed_factor + ped_ymean_mat;
          // if the speed factor is 0 then this will just be equal to ped_xmean
        }
        // apply the robot's samples
        auto robot_xtraj_samples = trajgen.get_xtraj_samples();
        auto robot_ytraj_samples = trajgen.get_ytraj_samples();
        xtraj_samples.submat(0, 0, n_samples - 1, n_steps - 1) = robot_xtraj_samples;
        ytraj_samples.submat(0, 0, n_samples - 1, n_steps - 1) = robot_ytraj_samples;
        // after this xtraj and ytraj samples are fully filled in!

        // Safety mask calculation
        // the idea is to see if the distance to the closest pedestrian
        // in any of the robot samples is less than the close stop threshold
        const auto closest_ped = selected_peds.pedestrians.at(closest_idxs.at(0));
        const arma::mat robot_samples_to_ped = arma::sqrt(
          arma::pow(
            robot_xtraj_samples -
            closest_ped.pose.position.x, 2) +
          arma::pow(
            robot_ytraj_samples -
            closest_ped.pose.position.y, 2));
        const auto closest_to_ped = arma::conv_to<arma::vec>::from(arma::min(robot_samples_to_ped, 1));
        const auto safety_mask = arma::conv_to<arma::rowvec>::from(closest_to_ped > close_stop_threshold);

        // also update the closest pedestrian to the robot for trial statistics
        if (goal_set){
          const auto dst_to_closest_ped = dist(robot_pose.x, robot_pose.y, 
            closest_ped.pose.position.x, closest_ped.pose.position.y);
          if (dst_to_closest_ped < trial_closest_dst_to_ped){
            trial_closest_dst_to_ped = dst_to_closest_ped;
          }
        }

        // brne optimization
        auto weights = brne.brne_nav(xtraj_samples, ytraj_samples);

        // check if a solution was not found for the robot (agent 0). 
        // This means we are going out of bounds and should stop before we hit the wall
        if (weights.row(0).is_zero()){
          if (goal_set){
            ROS_WARN_STREAM("No path found -- stopping navigation to this goal.");
            goal_set = false;
            estop_triggered = true;
          }
          // return;
        } else {
          // apply the safety mask to the weights for the robot to stop if a pedestrian is too close
          weights.row(0) %= safety_mask;
          const double mean_weights = arma::mean(weights.row(0));
          if (mean_weights != 0) {
            weights.row(0) /= mean_weights;
          } else {
            if (goal_set) {
              ROS_WARN_STREAM("E-STOP: Pedestrian too close!");
              estop_triggered = true;
              trial_n_estops += 1;
            }
          }
        }

        // compute the optimal commands using the BRNE weights
        const auto ulist = trajgen.get_ulist();
        const auto ulist_lin = arma::conv_to<arma::rowvec>::from(ulist.col(0));
        const auto ulist_ang = arma::conv_to<arma::rowvec>::from(ulist.col(1));
        const auto opt_cmds_lin = arma::mean(ulist_lin % weights.row(0));
        const auto opt_cmds_ang = arma::mean(ulist_ang % weights.row(0));

        // publish the appropriate series of commands
        if (goal_set) {
          for (int i = 0; i < n_steps; i++) {
            geometry_msgs::Twist tw;
            tw.linear.x = opt_cmds_lin;
            tw.angular.z = opt_cmds_ang;
            // manually adjust for dog's drift (discovered these values experimentally)
            if (offset_unitree_vel){
              if ((opt_cmds_lin > 0.1) && (opt_cmds_lin < 0.3)){
                tw.angular.z -= 0.04;
              } else if ((opt_cmds_lin >= 0.3) && (opt_cmds_lin < 0.5)){
                tw.angular.z -= 0.05;
              } else if (opt_cmds_lin >= 0.5){
                tw.angular.z -= 0.06;
              }
            }
            robot_cmds.twists.push_back(tw);
          }
          // Path is valid
          estop_triggered = false;
        }

        arma::mat opt_cmds(n_steps, 2, arma::fill::zeros);
        opt_cmds.col(0) = arma::vec(n_steps, arma::fill::value(opt_cmds_lin));
        opt_cmds.col(1) = arma::vec(n_steps, arma::fill::value(opt_cmds_ang));

        // compute the optimal path for visualization
        const auto opt_traj = trajgen.sim_traj(robot_pose.toVec(), opt_cmds);
        optimal_path.header.stamp = current_timestamp;
        optimal_path.poses.clear();
        for (int i = 0; i < n_steps; i++) {
          geometry_msgs::PoseStamped ps;
          ps.header.stamp = current_timestamp;
          ps.header.frame_id = _planning_frame;
          ps.pose.position.x = opt_traj.at(i, 0);
          ps.pose.position.y = opt_traj.at(i, 1);
          optimal_path.poses.push_back(ps);
        }


      } else {
        // No pedestrian
        estop_triggered = false;

        // go straight to the goal
        const auto opt_cmds = trajgen.opt_controls(goal_vec);
        if (goal_set) {
          for (int i = 0; i < n_steps; i++) {
            geometry_msgs::Twist tw;
            const auto opt_cmds_lin = opt_cmds.at(i, 0);
            const auto opt_cmds_ang = opt_cmds.at(i, 1);
            tw.linear.x = opt_cmds_lin;
            tw.angular.z = opt_cmds_ang;
            // manually adjust for dog's drift (discovered these values experimentally)
            if (offset_unitree_vel){
              if ((opt_cmds_lin > 0.1) && (opt_cmds_lin < 0.3)){
                tw.angular.z -= 0.04;
              } else if ((opt_cmds_lin >= 0.3) && (opt_cmds_lin < 0.5)){
                tw.angular.z -= 0.05;
              } else if (opt_cmds_lin >= 0.5){
                tw.angular.z -= 0.06;
              }
            }
            robot_cmds.twists.push_back(tw);
          }
        }
        // compute the optimal path
        const auto opt_traj = trajgen.sim_traj(robot_pose.toVec(), opt_cmds);
        optimal_path.header.stamp = current_timestamp;
        optimal_path.poses.clear();
        for (int i = 0; i < n_steps; i++) {
          geometry_msgs::PoseStamped ps;
          ps.header.stamp = current_timestamp;
          ps.header.frame_id = _planning_frame;
          ps.pose.position.x = opt_traj.at(i, 0);
          ps.pose.position.y = opt_traj.at(i, 1);
          optimal_path.poses.push_back(ps);
        }
      }
      // publish controls
      cmd_buf_pub_.publish(robot_cmds);
      
      // publish optimal path
      if(!estop_triggered){
        path_pub_.publish(optimal_path);
      } else{
        // If estop is triggered, publish current robot pose
        optimal_path.poses.clear();
        geometry_msgs::PoseStamped pose_stamped_temp;
        pose_stamped_temp.pose.position.x = robot_pose.x;
        pose_stamped_temp.pose.position.y = robot_pose.y;
        optimal_path.poses.push_back(pose_stamped_temp);
        path_pub_.publish(optimal_path);
      }

      // publish the walls
      pub_walls();

      const auto end = ros::Time::now();
      const auto diff = end - start;
    }
};

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "arena_crowd_nav");
  ros::NodeHandle nh;
  PathPlan pp = PathPlan(&nh);
  ros::spin();
}