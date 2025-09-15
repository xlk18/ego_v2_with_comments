//
// Created by zfg on 23-3-29.
//
#include <Eigen/Dense>
#include <utility>
#include <vector>
#include <queue>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Time.h>
#include "fstream"
#include "chrono"
//#include "utils/waypoint.h"
//#include "ReplanOptimal/SingleWaypointPlan.h"
#include "thread"
//#include "utils/Trajectories.h"
#ifndef SRC_CPCREPLAN_H
#define SRC_CPCREPLAN_H
#define INF 1000000.0
namespace Trajectories {
    using Vector3dList = std::vector<Eigen::Vector3d>;
    using Vector4dList = std::vector<Eigen::Vector4d>;
    using DoubleList = std::vector<double>;

    class StateBase {
    public:
        virtual DoubleList to_vector() = 0;
    };

    class States : public StateBase {
    private:
        Eigen::Vector3d pos, vel, ome;
        Eigen::Vector4d ori;
        double thr;
        double time_;
    public:
        States(double time, Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector4d at, Eigen::Vector3d ome, double th)
                :time_(time), pos(std::move(p)), vel(std::move(v)), ori(std::move(at)), ome(std::move(ome)), thr(th) {}

        DoubleList to_vector() override {
            std::vector<double> state{time_, pos(0), pos(1), pos(2), vel(0), vel(1), vel(2), ori(0), ori(1), ori(2), ori(3),
                                      ome(0), ome(1), ome(2), thr};
            return state;
        };
    };

    class PositionOnlyStates : public StateBase {
    private:
        Eigen::Vector3d pos;
    public:
        PositionOnlyStates(Eigen::Vector3d p)
                : pos(std::move(p)) {}

        DoubleList to_vector() override {
            std::vector<double> state{pos(0), pos(1), pos(2)};
            return state;
        };
    };

    class PointMassStates : public StateBase {
    private:
        Eigen::Vector3d pos, vel, acc;
    public:
        PointMassStates(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d a)
                : pos(std::move(p)), vel(std::move(v)), acc(std::move(a)) {}

        DoubleList to_vector() override {
            std::vector<double> state{pos(0), pos(1), pos(2), vel(0), vel(1), vel(2), acc(0), acc(1), acc(2)};
            return state;
        };
    };

    class PointMassWithYawStates : public StateBase {
    private:
        Eigen::Vector3d pos, vel, acc;
        double yaw;
    public:
        PointMassWithYawStates(Eigen::Vector3d p, Eigen::Vector3d v, Eigen::Vector3d a, double y)
                : pos(std::move(p)), vel(std::move(v)), acc(std::move(a)), yaw(y) {}

        DoubleList to_vector() override {
            Eigen::Vector3d eulerAngle(0, 0, -yaw);
            Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitX()));
            Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()));
            Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitZ()));
            Eigen::Quaterniond quaternion;
            quaternion = yawAngle * pitchAngle * rollAngle;

            std::vector<double> state{pos(0), pos(1), pos(2), vel(0), vel(1), vel(2), acc(0), acc(1), acc(2), quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
//            std::vector<double> state{pos(0), pos(1), pos(2), vel(0), vel(1), vel(2), acc(0), acc(1), acc(2), 1, 0, 0, 0};
            return state;
        };
    };

    class Trajectories {
    private:
        std::vector<Vector3dList> position_, velocity_, omega_, attitude_, accelerate_;
        std::vector<Vector4dList> orientation_;
        std::vector<DoubleList> thrust_;
        std::vector<DoubleList> time_;
        std::vector<std::shared_ptr<StateBase>> series_trajectory_;
        std::vector<DoubleList> yaw;
        int trajectory_size_;
        bool is_point_mass_ = false;
        bool with_yaw = false;
        bool position_only = false;
    public:
        explicit Trajectories(int segment_size=1) {
            trajectory_size_ = segment_size;
            for (int i = 0; i < segment_size; i++) {
                position_.emplace_back();
                velocity_.emplace_back();
                accelerate_.emplace_back();
                orientation_.emplace_back();
                omega_.emplace_back();
                thrust_.emplace_back();
                yaw.emplace_back();
                time_.emplace_back();
            }
        }

        void set_pointmass_trajectories(std::vector<Vector3dList> pos, std::vector<Vector3dList> vel, std::vector<Vector3dList> acc) {
            is_point_mass_ = true;
            reset();
            position_ = std::move(pos);
            velocity_ = std::move(vel);
            accelerate_ = std::move(acc);
            get_full_trajectories();
        }

        void
        set_full_state_trajectories(std::vector<Vector3dList> pos, std::vector<Vector3dList> vel, std::vector<Vector4dList> att, std::vector<Vector3dList> ome, std::vector<DoubleList> thrust) {
            is_point_mass_ = false;
            reset();
            position_ = std::move(pos);
            velocity_ = std::move(vel);
            orientation_ = std::move(att);
            omega_ = std::move(ome);
            thrust_ = std::move(thrust);
            get_full_trajectories();
        }

        void set_full_state_trajectory(int index, Vector3dList pos, Vector3dList vel, Vector4dList att, Vector3dList ome, DoubleList thrust) {
            is_point_mass_ = false;
            position_.at(index) = std::move(pos);
            velocity_.at(index) = std::move(vel);
            orientation_.at(index) = std::move(att);
            omega_.at(index) = std::move(ome);
            thrust_.at(index) = std::move(thrust);
        }

        void set_point_mass_state_trajectory(int index, Vector3dList pos, Vector3dList vel, Vector3dList acc) {
            is_point_mass_ = true;
            position_.at(index) = std::move(pos);
            velocity_.at(index) = std::move(vel);
            accelerate_.at(index) = std::move(acc);
        }

        void set_yaw(const std::vector<Eigen::MatrixXd> &yaw_traj_from_minimum_snap) {
            with_yaw = true;
            yaw.clear();
            for (auto item: yaw_traj_from_minimum_snap) {
                Eigen::MatrixXd yaw_angel = item.row(0);
                yaw.emplace_back(yaw_angel.data(), yaw_angel.data() + yaw_angel.cols());
            }
        }

    private:
        std::vector<std::shared_ptr<StateBase>> get_full_trajectories() {
            series_trajectory_.clear();
            for (int i = 0; i < position_.size(); i++) {
                for (int j = 0; j < position_.at(i).size(); j++) {
                    if (position_only) {
                        series_trajectory_.push_back(std::make_shared<PositionOnlyStates>(position_.at(i).at(j)));
                    } else {
                        if (is_point_mass_) {
                            if (with_yaw) {
                                series_trajectory_.push_back(std::make_shared<PointMassWithYawStates>(position_.at(i).at(j), velocity_.at(i).at(j), accelerate_.at(i).at(j), yaw.at(i).at(j)));
                            } else {
                                series_trajectory_.push_back(std::make_shared<PointMassStates>(position_.at(i).at(j), velocity_.at(i).at(j), accelerate_.at(i).at(j)));
                            }
                        } else {
                            series_trajectory_.push_back(std::make_shared<States>(time_.at(i).at(j), position_.at(i).at(j), velocity_.at(i).at(j), orientation_.at(i).at(j), omega_.at(i).at(j), thrust_.at(i).at(j)));
                        }
                    }

                }
            }
            return series_trajectory_;
        }

    public:
        std::deque<std::vector<double>> get_full_trajectories_vec() {
            std::deque<std::vector<double>> ftv;
            if (series_trajectory_.size() >= 21) {
                for (auto &i: series_trajectory_) {
                    ftv.push_back(i->to_vector());
                }
            } else {
                for (auto &i: series_trajectory_) {
                    ftv.push_back(i->to_vector());
                }
                for (int i = 0; i < 21 - series_trajectory_.size(); i++) {
                    ftv.push_back(series_trajectory_.at(series_trajectory_.size() - 1)->to_vector());
                }
            }
            return ftv;
        }

        std::vector<DoubleList> get_yaw() const {
            return yaw;
        }



    private:
        void reset() {
            position_.clear();
            velocity_.clear();
            orientation_.clear();
            omega_.clear();
            thrust_.clear();
            series_trajectory_.clear();
        }
    };
}

class waypoint {
public:
    Eigen::Vector3d position;
    Eigen::Vector3d velocity_direction;
    int index;
    explicit waypoint(Eigen::Vector3d pos, Eigen::Vector3d vel_d, int i = 0) : position(std::move(pos)), velocity_direction(std::move(vel_d)), index(i){};
};

struct node {
    bool is_arrived;
    double g_from_begin;
    Eigen::Vector3d Velocity_;
    node *father_node = nullptr;
    int preSample_vel_ratio = 0;

    int num_wp;
    std::string flag = "node";

    node(bool arr, double g, Eigen::Vector3d vel, std::string n, int n_wp) : is_arrived(arr), g_from_begin(g),
                                                                             Velocity_(std::move(vel)),
                                                                             flag(std::move(n)), num_wp(n_wp), preSample_vel_ratio(0) {};

    node(bool arr, double g, Eigen::Vector3d vel, std::string n, int n_wp, int ratio) : is_arrived(arr), g_from_begin(g),
                                                                                        Velocity_(std::move(vel)),
                                                                                        flag(std::move(n)), num_wp(n_wp), preSample_vel_ratio(ratio) {};

    node() : is_arrived(false), g_from_begin(INF), Velocity_(Eigen::Vector3d{0, 0, 0}) {};
};

struct OptimalNode : public node {
    // 上一个节点到当前节点的加速度缩放因子
    Eigen::Vector3d alpha_from_father_node{1, 1, 1};
    Eigen::Vector2d x_time{0, 0};
    Eigen::Vector2d y_time{0, 0};
    Eigen::Vector2d z_time{0, 0};
    // 上一个节点到当前节点的第一个加速度方向
    Eigen::Vector3d first_a{1, 1, 1};
    Eigen::Vector3d position_shift{0, 0, 0};
    Eigen::Vector3d velocity_shift{0, 0, 0};

    Eigen::Vector3d do_not_move_in_this_axis{0, 0, 0};

    explicit OptimalNode(node &n1) {
        is_arrived = n1.is_arrived;
        g_from_begin = n1.g_from_begin;
        Velocity_ = n1.Velocity_;
        father_node = n1.father_node;
        num_wp = n1.num_wp;
        flag = n1.flag;
    }

};

struct cmp {
    bool operator()(node *a, node *b) {
        return a->g_from_begin > b->g_from_begin;
    }
};

class PointMassPathSearching {
public:
    PointMassPathSearching(double ax_up, double ay_up, double az_up, double ax_low, double ay_low, double az_low, int num_of_full_waypoints_seg) : a_up_bond_x(
            ax_up), a_up_bond_y(ay_up), a_up_bond_z(az_up), a_low_bond_x(ax_low), a_low_bond_y(ay_low), a_low_bond_z(
            az_low), num_of_full_waypoints_seg_(num_of_full_waypoints_seg) {
        for(int i = 1; i < num_of_full_waypoints_seg; i++){
//            SingleWaypointPlan mr(i);
//            single_waypoint_plan_list_.push_back(mr);
            optimal_position_.emplace_back();
            optimal_velocity_.emplace_back();
            optimal_attitude_.emplace_back();
            optimal_omega_.emplace_back();
            optimal_thrust_.emplace_back();
            optimal_time.push_back(0);
        }
    };

    std::vector<OptimalNode> solve(std::deque<waypoint> waypointVec, const Eigen::Vector3d &currentPosition,
                                   const Eigen::Vector3d &currentVelcity, const Eigen::Vector4d &currentAttitude, bool is_fix=false);

    void drawTrajectory(double delta_t);

    void reset();

    std::vector<Eigen::Vector3d> getOptimalVelocity() const;

    std::vector<float> getOptimalTime() const;

    void getOptimalTrajectory(std::vector<std::vector<Eigen::Vector3d>> * Position, std::vector<std::vector<Eigen::Vector3d>> * Velocity, std::vector<std::vector<Eigen::Vector4d>> * Attitude, std::vector<std::vector<Eigen::Vector3d>> * Omega, std::vector<std::vector<double>> * Thrust) const;
    void getOptimalTrajectory(Trajectories::Trajectories* traj) const;
    void getPointMassTrajectory(Trajectories::Trajectories* traj) const;
    void getPointMassTrajectoryMPCC(double num_point, std::vector<Eigen::Vector3d> &position_list) const;

    void saveTrajectory2file();

private:
    void cleanPreSample();

    void getWaypointDistance();

    void sampleVelocity(Eigen::Vector3d curVelocity);

    void preSearch();

    Eigen::Matrix<float, 3, 21> normSample(Eigen::Vector3d cone_direction);

    static Eigen::Matrix<double, 40, 3> coneSample(Eigen::Vector3d conicalSpindle);
    static Eigen::Matrix<double, 20, 3> coneSample_old(Eigen::Vector3d conicalSpindle);

    bool shortestPathSearching();

    void reverse();

    double calculateJ(Eigen::Vector3d v_init, Eigen::Vector3d v_end, Eigen::Vector3d distance);

    double calculateDuritionTinSample(Eigen::Vector3d v_init, Eigen::Vector3d v_end, Eigen::Vector3d distance) const;

    void calculateDuritonT(OptimalNode &node, Eigen::Vector3d v_init, Eigen::Vector3d v_end, Eigen::Vector3d distance);

    static Eigen::Vector3d
    calculateTperAxis(double v_init, double v_end, double distance, double a_low_bond, double a_up_bond);

    static Eigen::Vector4d
    refineTperAxis(double v_init, double v_end, double distance, double t, double a_low_bond, double a_up_bond,
                   double a_dir);

    static Eigen::Vector2d solveQuadratic(double a, double b, double c);

    Eigen::Vector4d calculateAttitudefromAcce(Eigen::Vector3d acc, double yaw);

//    std::vector<SingleWaypointPlan> single_waypoint_plan_list_;

    // 第一维是wp序列，第二维是速度的采样值
    std::vector<std::vector<node>> Velocity_Network_;
    // 存放搜索到的最快路径每一段的三个时间
    std::vector<Eigen::Vector3d> wayPoint_Optimal_TimeVec_;
    // 存放搜索到的最快路径每一个waypoint处的速度
    std::vector<OptimalNode> wayPoint_Optimal_VelocityVec_;
    // 存放每一个wp之间的距离
    std::vector<Eigen::Vector3d> distance_between_waypoint_;
    std::vector<Eigen::Vector3d> velocity_direction_prior_from_EGOPLANNER_;
    std::deque<waypoint> waypoints_;
    std::vector<node *> close_list;

    std::deque<Eigen::Vector3d> preSampleVelocity;
    Eigen::Matrix3d time_dur;
    Eigen::Vector3d current_position_;
    Eigen::Vector3d current_velcity_;
    Eigen::Vector4d current_attitude_;

    int num_of_full_waypoints_seg_;
    double a_low_bond_x = 1;
    double a_low_bond_y = 1;
    double a_low_bond_z = 1;
    double a_up_bond_x = 1;
    double a_up_bond_y = 1;
    double a_up_bond_z = 1;


    std::vector<std::vector<Eigen::Vector3d>> optimal_position_, optimal_velocity_,  optimal_omega_;
    std::vector<std::vector<Eigen::Vector3d>> pointmass_position_, pointmass_velocity_,  pointmass_acc_;
    std::vector<std::vector<Eigen::Vector4d>> optimal_attitude_;
    std::vector<std::vector<double>> optimal_thrust_;
    std::vector<double> optimal_time;
};

#endif //SRC_CPCREPLAN_H
