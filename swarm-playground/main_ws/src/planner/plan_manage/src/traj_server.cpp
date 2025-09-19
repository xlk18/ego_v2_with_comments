#include <nav_msgs/Odometry.h>
#include <traj_utils/PolyTraj.h>
#include <optimizer/poly_traj_utils.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include "PointMassPathSearching.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <deque>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
using namespace Eigen;

constexpr double PM_TRAJ_STEP = 0.01;

ros::Publisher pos_cmd_pub;
ros::Publisher point_mass_cmd_pub;
quadrotor_msgs::PositionCommand cmd;

bool receive_traj_ = false;
boost::shared_ptr<poly_traj::Trajectory> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;
ros::Time heartbeat_time_(0);
Eigen::Vector3d last_pos_;
Eigen::Vector3d last_pm_pos_;

// yaw control
double last_yaw_, last_yawdot_;
double last_pm_yaw_, last_pm_yawdot_;
double time_forward_;

// 新增：point_mass_trajectory_generation相关变量
size_t num;
bool pub_origin_cmd_;
bool use_time_reallocation_ ;                    // 是否使用新的质点模型轨迹
std::unique_ptr<PointMassPathSearching> point_mass_planner_;  // 质点规划器
Trajectories::Trajectories point_mass_trajectory_;     // 存储质点轨迹
bool point_mass_trajectory_ready_ = false;             // 质点轨迹是否准备好
std::vector<Eigen::Vector3d> point_mass_positions_;          
std::vector<Eigen::Vector3d> point_mass_velocities_; 
// 航点相关参数
double waypoint_spacing_;                       // 航点间距（米）
int min_waypoints_ = 3;                               // 最小航点数
int max_waypoints_ = 15;                              // 最大航点数
double max_acc_;                               // 最大加速度
double max_vel_;                               // 最大速度

void heartbeatCallback(std_msgs::EmptyPtr msg)
{
  heartbeat_time_ = ros::Time::now();
}

// 根据轨迹长度计算航点数量 
int calculateOptimalWaypointCount(const double &traj_len)
{
    // 根据间距计算需要的航点数量
    int waypoint_count = static_cast<int>(std::ceil(traj_len / waypoint_spacing_)) + 1; 
    // 限制在合理范围内
    waypoint_count = std::max(min_waypoints_, std::min(max_waypoints_, waypoint_count));
    return waypoint_count;
}

// 根据距离返回在轨迹上对应的时刻 
double findTimeAtDistance(const boost::shared_ptr<poly_traj::Trajectory> &trajectory, double target_dist, double total_len)
{
    double total_duration = trajectory->getTotalDuration();
    // 假设飞机匀速飞行，按比例估算一个初始时间 t_guess
    double t_guess = (target_dist / total_len) * total_duration;
    for (int i = 0; i < 5; ++i) 
    {
      // 计算在当前猜测时间点 t_guess 处的实际距离
      double current_dist = trajectory->getLength(t_guess);
      // 计算误差离目标距离还差多远
      double error = target_dist - current_dist;
      if (std::abs(error) < 0.05) 
      {
          break;
      }
      // 计算当前点的速度大小
      double speed_at_t = trajectory->getVel(t_guess).norm();
      if (speed_at_t < 1e-4)
      {
          break;
      }
      t_guess += error / speed_at_t;
    }
    if (t_guess > total_duration) t_guess = total_duration;
    if (t_guess < 0.0) t_guess = 0.0;
    
    return t_guess;
}

// 从原始轨迹采样航点 
std::deque<waypoint> sampleTrajectoryWaypoints(const boost::shared_ptr<poly_traj::Trajectory> &trajectory, int num_samples)
{
  std::deque<waypoint> point_mass_waypoints_; 
  double total_duration = trajectory->getTotalDuration();
  double length = trajectory->getLength(total_duration);
  if (num_samples == 3)
  {
    point_mass_waypoints_.emplace_back(trajectory->getPos(0.0), trajectory->getVel(0.0), 0);
    point_mass_waypoints_.emplace_back(trajectory->getPos(total_duration / 2), trajectory->getVel(total_duration / 2).normalized(), 1);
    point_mass_waypoints_.emplace_back(trajectory->getPos(total_duration), trajectory->getVel(total_duration), 2);
  }else if (num_samples == 15 )
  {
    point_mass_waypoints_.emplace_back(trajectory->getPos(0.0), trajectory->getVel(0.0), 0);
    for (size_t i = 1; i < 14; i++)
    {
      double target_dist = i * length / 14;
      double t = findTimeAtDistance(trajectory, target_dist, length);
      Eigen::Vector3d pos = trajectory->getPos(t);
      Eigen::Vector3d vel = trajectory->getVel(t);
      Eigen::Vector3d vel_dir = vel.normalized();
      point_mass_waypoints_.emplace_back(pos, vel_dir, i);
    }
    point_mass_waypoints_.emplace_back(trajectory->getPos(total_duration), trajectory->getVel(total_duration), 14);
  }else{
    point_mass_waypoints_.emplace_back(trajectory->getPos(0.0), trajectory->getVel(0.0), 0);
    for (size_t i = 1; i < num_samples - 1; i++)  
    {
      double target_dist = i * waypoint_spacing_;
      double t = findTimeAtDistance(trajectory, target_dist, length);
      Eigen::Vector3d pos = trajectory->getPos(t);
      Eigen::Vector3d vel = trajectory->getVel(t);
      Eigen::Vector3d vel_dir = vel.normalized();
      point_mass_waypoints_.emplace_back(pos, vel_dir, i);
    }
    point_mass_waypoints_.emplace_back(trajectory->getPos(total_duration), trajectory->getVel(total_duration), num_samples-1);
  }
  return point_mass_waypoints_;
}

// 从质点轨迹得到控制命令
void getpointMassCmd(const std::deque<std::vector<double>> &trajectory)
{
  point_mass_positions_.clear();
  point_mass_velocities_.clear();
  num = trajectory.size();
  point_mass_positions_.reserve(num); 
  point_mass_velocities_.reserve(num);
  for (const auto& point : trajectory) {
    point_mass_positions_.emplace_back(Eigen::Vector3d{point.at(0), point.at(1), point.at(2)});
    point_mass_velocities_.emplace_back(Eigen::Vector3d{point.at(3), point.at(4), point.at(5)});
  }
}

void polyTrajCallback(traj_utils::PolyTrajPtr msg)
{
  if (msg->order != 5)
  {
    ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
  {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }

  traj_.reset(new poly_traj::Trajectory(dura, cMats));
  traj_duration_ = traj_->getTotalDuration();
  
  //---------根据use_time_reallocation_标志决定是否使用质点轨迹优化------//
  if (use_time_reallocation_) {
    ros::Time time_1 = ros::Time::now();
    double traj_length_ = traj_->getLength(traj_duration_);
    int waypoint_num = calculateOptimalWaypointCount(traj_length_);
    try {
      point_mass_planner_ = std::make_unique<PointMassPathSearching>(
        max_acc_, max_acc_, max_acc_,
        -max_acc_, -max_acc_, -max_acc_,
        waypoint_num
      );
      std::deque<waypoint> point_mass_waypoints_ = sampleTrajectoryWaypoints(traj_, waypoint_num);  
      Eigen::Vector3d start_pos = traj_->getPos(0.0);
      Eigen::Vector3d start_vel = traj_->getVel(0.0);
      Eigen::Vector4d start_attitude(1, 0, 0, 0);
      point_mass_planner_->solve(point_mass_waypoints_, start_pos, start_vel, start_attitude, false);
      if (!point_mass_planner_->getwaypointOptimalVel().empty())
      {
        point_mass_planner_->drawTrajectory(PM_TRAJ_STEP);
        point_mass_planner_->getPointMassTrajectory(&point_mass_trajectory_);
        std::deque<std::vector<double>> trajectory = point_mass_trajectory_.get_full_trajectories_vec();
        getpointMassCmd(trajectory);
        point_mass_trajectory_ready_ = true;
      }else{
        point_mass_trajectory_ready_ = false;
      }
      ros::Time time_2 = ros::Time::now();
      double execution_time = (time_2 - time_1).toSec();
      ROS_INFO("[Point Mass Planning] Execution time: %.6f seconds", execution_time);
    }catch(const std::exception& e){
      std::cerr << e.what() << '\n';
      use_time_reallocation_ = false;
    }
  }else {
    point_mass_trajectory_ready_ = false;
  }
  //---------------------------------------//
  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;
  receive_traj_ = true;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt)
{
  constexpr double YAW_DOT_MAX_PER_SEC = 2 * M_PI;
  constexpr double YAW_DOT_DOT_MAX_PER_SEC = 5 * M_PI;
  std::pair<double, double> yaw_yawdot(0, 0);

  Eigen::Vector3d dir;

  dir = t_cur + time_forward_ <= traj_duration_
            ? traj_->getPos(t_cur + time_forward_) - pos
            : traj_->getPos(traj_duration_) - pos;
  
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;

  double yawdot = 0;
  double d_yaw = yaw_temp - last_yaw_;
  if (d_yaw >= M_PI)
  {
    d_yaw -= 2 * M_PI;
  }
  if (d_yaw <= -M_PI)
  {
    d_yaw += 2 * M_PI;
  }

  const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
  const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
  double d_yaw_max;
  if (fabs(last_yawdot_ + dt * YDDM) <= fabs(YDM))
  {
    d_yaw_max = last_yawdot_ * dt + 0.5 * YDDM * dt * dt;
  }
  else
  {
    double t1 = (YDM - last_yawdot_) / YDDM;
    d_yaw_max = ((dt - t1) + dt) * (YDM - last_yawdot_) / 2.0;
  }

  if (fabs(d_yaw) > fabs(d_yaw_max))
  {
    d_yaw = d_yaw_max;
  }
  yawdot = d_yaw / dt;

  double yaw = last_yaw_ + d_yaw;
  if (yaw > M_PI)
    yaw -= 2 * M_PI;
  if (yaw < -M_PI)
    yaw += 2 * M_PI;
  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  last_yaw_ = yaw_yawdot.first;
  last_yawdot_ = yaw_yawdot.second;

  yaw_yawdot.second = yaw_temp;

  return yaw_yawdot;
}

std::pair<double, double> calculate_pm_yaw(double t_cur, Eigen::Vector3d &pos, double dt)
{
  constexpr double YAW_DOT_MAX_PER_SEC = 2 * M_PI;
  constexpr double YAW_DOT_DOT_MAX_PER_SEC = 5 * M_PI;
  std::pair<double, double> yaw_yawdot(0, 0);

  Eigen::Vector3d dir;
  
  double t_forward = t_cur + time_forward_;
  int t_pm = static_cast<int>(t_forward / PM_TRAJ_STEP); 
  dir = t_forward <= num * PM_TRAJ_STEP
            ? point_mass_positions_.at(t_pm) - pos
            : point_mass_positions_.back() - pos;
  
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_pm_yaw_;

  double yawdot = 0;
  double d_yaw = yaw_temp - last_pm_yaw_;
  if (d_yaw >= M_PI)
  {
    d_yaw -= 2 * M_PI;
  }
  if (d_yaw <= -M_PI)
  {
    d_yaw += 2 * M_PI;
  }

  const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
  const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
  double d_yaw_max;
  if (fabs(last_pm_yawdot_ + dt * YDDM) <= fabs(YDM))
  {
    d_yaw_max = last_pm_yawdot_ * dt + 0.5 * YDDM * dt * dt;
  }
  else
  {
    double t1 = (YDM - last_pm_yawdot_) / YDDM;
    d_yaw_max = ((dt - t1) + dt) * (YDM - last_pm_yawdot_) / 2.0;
  }

  if (fabs(d_yaw) > fabs(d_yaw_max))
  {
    d_yaw = d_yaw_max;
  }
  yawdot = d_yaw / dt;

  double yaw = last_pm_yaw_ + d_yaw;
  if (yaw > M_PI)
    yaw -= 2 * M_PI;
  if (yaw < -M_PI)
    yaw += 2 * M_PI;
  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  last_pm_yaw_ = yaw_yawdot.first;
  last_pm_yawdot_ = yaw_yawdot.second;

  yaw_yawdot.second = yaw_temp;

  return yaw_yawdot;
}

void publish_cmd(Vector3d p, Vector3d v, Vector3d a, Vector3d j, double y, double yd)
{
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.jerk.x = j(0);
  cmd.jerk.y = j(1);
  cmd.jerk.z = j(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  pos_cmd_pub.publish(cmd);

  last_pos_ = p;
}

void publish_point_mass_cmd(Vector3d p, Vector3d v, Vector3d a, Vector3d j, double y, double yd)
{
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.jerk.x = j(0);
  cmd.jerk.y = j(1);
  cmd.jerk.z = j(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  point_mass_cmd_pub.publish(cmd);

  last_pm_pos_ = p;
}


void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ and have heartbeat */
  if (heartbeat_time_.toSec() <= 1e-5)
  {
    return;
  }
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();

  if ((time_now - heartbeat_time_).toSec() > 0.5)
  {
    ROS_ERROR("[traj_server] Lost heartbeat from the planner, is it dead?");
    receive_traj_ = false;
    publish_cmd(last_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_yaw_, 0);
    return;
  }

  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();

  // 根据use_time_reallocation_标志选择不同的轨迹执行逻辑
  if (use_time_reallocation_ && point_mass_trajectory_ready_) {
      // 使用质点轨迹
      if (t_cur < num * PM_TRAJ_STEP && t_cur >= 0.0)
      {
        int T_pm = static_cast<int>(t_cur / PM_TRAJ_STEP);
        T_pm = std::min(T_pm, static_cast<int>(num) - 1);
        pos = point_mass_positions_.at(T_pm);
        vel = point_mass_velocities_.at(T_pm);
        /*** calculate yaw ***/
        yaw_yawdot = calculate_pm_yaw(t_cur, pos, (time_now - time_last).toSec());
        /*** calculate yaw ***/

        time_last = time_now;
        last_pm_yaw_ = yaw_yawdot.first;
        last_pm_pos_ = pos;

        // publish
        publish_point_mass_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
      }
  } 
  if (pub_origin_cmd_){
      // 使用原始轨迹逻辑
    if (t_cur < traj_duration_ && t_cur >= 0.0)
    {
      pos = traj_->getPos(t_cur);
      vel = traj_->getVel(t_cur);
      acc = traj_->getAcc(t_cur);
      jer = traj_->getJer(t_cur);

      /*** calculate yaw ***/
      yaw_yawdot = calculate_yaw(t_cur, pos, (time_now - time_last).toSec());
      /*** calculate yaw ***/

      time_last = time_now;
      last_yaw_ = yaw_yawdot.first;
      last_pos_ = pos;


      // publish
      publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle nh("~");

  ros::Subscriber poly_traj_sub = nh.subscribe("planning/trajectory", 10, polyTrajCallback);
  ros::Subscriber heartbeat_sub = nh.subscribe("heartbeat", 10, heartbeatCallback);

  pos_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);
  point_mass_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/point_mass_cmd", 50);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  //质点模型参数
  nh.param("traj_server/use_time_reallocation", use_time_reallocation_, false);
  nh.param("traj_server/waypoint_spacing", waypoint_spacing_, 0.5);
  nh.param("traj_server/max_acc", max_acc_, 10.0);
  nh.param("traj_server/max_vel", max_vel_, 10.0);
  nh.param("traj_server/pub_origin_cmd", pub_origin_cmd_, true);
  
  last_yaw_ = 0.0;
  last_yawdot_ = 0.0;
  last_pm_yaw_ = 0.0;
  last_pm_yawdot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_INFO("[Traj server]: ready with point mass trajectory optimization %s.", 
           use_time_reallocation_ ? "ENABLED" : "DISABLED");

  ros::spin();

  return 0;
}