//
// Created by zfg on 23-3-29.
//
//#include "include/CPCReplan.h"
#include <iostream>
#include <utility>
#include <future>
#include "PointMassPathSearching.h"

void PointMassPathSearching::calculateDuritonT(OptimalNode &node, Eigen::Vector3d v_init, Eigen::Vector3d v_end,
                                               Eigen::Vector3d distance) {

    Eigen::Vector3d x_time = calculateTperAxis(v_init[0], v_end[0], distance[0], a_low_bond_x, a_up_bond_x);
    Eigen::Vector3d Y_time = calculateTperAxis(v_init[1], v_end[1], distance[1], a_low_bond_y, a_up_bond_y);
    Eigen::Vector3d Z_time = calculateTperAxis(v_init[2], v_end[2], distance[2], a_low_bond_z, a_up_bond_z);
    double t_x = x_time(0) + x_time(1);
    double t_y = Y_time(0) + Y_time(1);
    double t_z = Z_time(0) + Z_time(1);
    double x_a_dir = x_time(2);
    double y_a_dir = Y_time(2);
    double z_a_dir = Z_time(2);
    if(t_x == 0){
        node.do_not_move_in_this_axis(0) = 1;
    }
    if(t_y == 0){
        node.do_not_move_in_this_axis(1) = 1;
    }
    if(t_z == 0){
        node.do_not_move_in_this_axis(2) = 1;
    }
    if (t_x > t_y && t_x > t_z) {
        node.x_time << x_time(0), x_time(1);
        node.alpha_from_father_node(0) = 1;
        node.first_a(0) = x_a_dir;
        if(node.do_not_move_in_this_axis(1) != 1){
            Eigen::Vector4d new_y_time;
            new_y_time = refineTperAxis(v_init[1], v_end[1], distance[1], t_x, a_low_bond_y, a_up_bond_y, y_a_dir);
            node.y_time << new_y_time(0), new_y_time(1);
            node.alpha_from_father_node(1) = new_y_time(2);
            node.first_a(1) = y_a_dir;

        }
        if(node.do_not_move_in_this_axis(2) != 1){
            Eigen::Vector4d new_z_time;
            new_z_time = refineTperAxis(v_init[2], v_end[2], distance[2], t_x, a_low_bond_z, a_up_bond_z, z_a_dir);
            node.z_time << new_z_time(0), new_z_time(1);
            node.alpha_from_father_node(2) = new_z_time(2);
            node.first_a(2) = z_a_dir;
        }
    }
    if (t_y > t_x && t_y > t_z) {
        node.y_time << Y_time(0), Y_time(1);
        node.alpha_from_father_node(1) = 1;
        node.first_a(1) = y_a_dir;
        if(node.do_not_move_in_this_axis(0) != 1){
            Eigen::Vector4d new_x_time;
            new_x_time = refineTperAxis(v_init[0], v_end[0], distance[0], t_y, a_low_bond_x, a_up_bond_x, x_a_dir);
            node.x_time << new_x_time(0), new_x_time(1);
            node.alpha_from_father_node(0) = new_x_time(2);
            node.first_a(0) = x_a_dir;
        }
        if(node.do_not_move_in_this_axis(2) != 1){
            Eigen::Vector4d new_z_time;
            new_z_time = refineTperAxis(v_init[2], v_end[2], distance[2], t_y, a_low_bond_z, a_up_bond_z, z_a_dir);
            node.z_time << new_z_time(0), new_z_time(1);
            node.alpha_from_father_node(2) = new_z_time(2);
            node.first_a(2) = z_a_dir;
        }
    }
    if (t_z > t_x && t_z > t_y) {
        node.z_time << Z_time(0), Z_time(1);
        node.alpha_from_father_node(2) = 1;
        node.first_a(2) = z_a_dir;
        if(node.do_not_move_in_this_axis(0) != 1){
            Eigen::Vector4d new_x_time;
            new_x_time = refineTperAxis(v_init[0], v_end[0], distance[0], t_z, a_low_bond_x, a_up_bond_x, x_a_dir);
            node.x_time << new_x_time(0), new_x_time(1);
            node.alpha_from_father_node(0) = new_x_time(2);
            node.first_a(0) = x_a_dir;
        }

        if(node.do_not_move_in_this_axis(1) != 1){
            Eigen::Vector4d new_y_time;
            new_y_time = refineTperAxis(v_init[1], v_end[1], distance[1], t_z, a_low_bond_y, a_up_bond_y, y_a_dir);
            node.y_time << new_y_time(0), new_y_time(1);
            node.alpha_from_father_node(1) = new_y_time(2);
            node.first_a(1) = y_a_dir;

        }
    }
    if(node.alpha_from_father_node(0) < -1 || node.alpha_from_father_node(0) > 1 || node.alpha_from_father_node(1) < -1 || node.alpha_from_father_node(1) > 1 || node.alpha_from_father_node(2) < -1 || node.alpha_from_father_node(2) > 1){
        std::cout<<" "<<std::endl;
    }
    if (node.first_a(0) < 0) {
        node.velocity_shift(0) = v_init(0) + node.alpha_from_father_node(0) * a_low_bond_x * node.x_time(0);
        node.position_shift(0) = waypoints_[node.num_wp - 1].position(0) + v_init(0) * node.x_time(0) +
                                 0.5 * node.alpha_from_father_node(0) * a_low_bond_x * node.x_time(0) * node.x_time(0);
    } else {
        node.velocity_shift(0) = v_init(0) + node.alpha_from_father_node(0) * a_up_bond_x * node.x_time(0);
        node.position_shift(0) = waypoints_[node.num_wp - 1].position(0) + v_init(0) * node.x_time(0) +
                                 0.5 * node.alpha_from_father_node(0) * a_up_bond_x * node.x_time(0) * node.x_time(0);

    }
    if (node.first_a(1) < 0) {
        node.velocity_shift(1) = v_init(1) + node.alpha_from_father_node(1) * a_low_bond_y * node.y_time(0);
        node.position_shift(1) = waypoints_[node.num_wp - 1].position(1) + v_init(1) * node.y_time(0) +
                                 0.5 * node.alpha_from_father_node(1) * a_low_bond_y * node.y_time(0) * node.y_time(0);
    } else {
        node.velocity_shift(1) = v_init(1) + node.alpha_from_father_node(1) * a_up_bond_y * node.y_time(0);
        node.position_shift(1) = waypoints_[node.num_wp - 1].position(1) + v_init(1) * node.y_time(0) +
                                 0.5 * node.alpha_from_father_node(1) * a_up_bond_y * node.y_time(0) * node.y_time(0);
    }
    if (node.first_a(2) < 0) {
        node.velocity_shift(2) = v_init(2) + node.alpha_from_father_node(2) * a_low_bond_z * node.z_time(0);
        node.position_shift(2) = waypoints_[node.num_wp - 1].position(2) + v_init(2) * node.z_time(0) +
                                 0.5 * node.alpha_from_father_node(2) * a_low_bond_z * node.z_time(0) * node.z_time(0);
    } else {
        node.velocity_shift(2) = v_init(2) + node.alpha_from_father_node(2) * a_up_bond_z * node.z_time(0);
        node.position_shift(2) = waypoints_[node.num_wp - 1].position(2) + v_init(2) * node.z_time(0) +
                                 0.5 * node.alpha_from_father_node(2) * a_up_bond_z * node.z_time(0) * node.z_time(0);
    }
    if(node.do_not_move_in_this_axis(0)){
        node.x_time << 0, 0;
        node.alpha_from_father_node(0) = 0;
        node.first_a(0) = 1;
        node.velocity_shift(0) = 0;
        node.position_shift(0) = 0;
    }
    if(node.do_not_move_in_this_axis(1)){
        node.y_time << 0, 0;
        node.alpha_from_father_node(1) = 0;
        node.first_a(1) = 1;
        node.velocity_shift(1) = 0;
        node.position_shift(1) = 0;
    }
    if(node.do_not_move_in_this_axis(2)){
        node.z_time << 0, 0;
        node.alpha_from_father_node(2) = 0;
        node.first_a(2) = 1;
        node.velocity_shift(2) = 0;
        node.position_shift(2) = 0;
    }

}

Eigen::Vector3d
PointMassPathSearching::calculateTperAxis(double v_init, double v_end, double distance, double a_low_bond, double a_up_bond) {

    Eigen::Vector3d time_durition;
    double a = a_up_bond / 2 - (a_up_bond * a_up_bond) / (2 * a_low_bond);
    double b = v_init - (v_init * a_up_bond) / a_low_bond;
    double c = -distance + (v_end - v_init) * (v_end + v_init) / (2 * a_low_bond);
    Eigen::Vector2d t1 = solveQuadratic(a, b, c);
    if(t1(0) == t1(1) && t1(0) == 0){
        return (Eigen::Vector3d ()<<0, 0, 1).finished();
    }
    double t20 = (v_end - v_init - a_up_bond * t1(0)) / a_low_bond;
    double t21 = (v_end - v_init - a_up_bond * t1(1)) / a_low_bond;


    double mid;
    mid = a_low_bond;
    a_low_bond = a_up_bond;
    a_up_bond = mid;

    double at = a_up_bond / 2 - (a_up_bond * a_up_bond) / (2 * a_low_bond);
    double bt = v_init - (v_init * a_up_bond) / a_low_bond;
    double ct = -distance + (v_end - v_init) * (v_end + v_init) / (2 * a_low_bond);
    Eigen::Vector2d t1t = solveQuadratic(at, bt, ct);
    if(t1t(0) == t1t(1) && t1t(0) == 0){
        return (Eigen::Vector3d ()<<0, 0, 1).finished();
    }
    double t20t = (v_end - v_init - a_up_bond * t1t(0)) / a_low_bond;
    double t21t = (v_end - v_init - a_up_bond * t1t(1)) / a_low_bond;

    if (t20 < 0 || t1(0) < 0){
        t20 = -INF;
        t1(0) = -INF;
    }
    if (t21 < 0 || t1(1) < 0){
        t21 = -INF;
        t1(1) = -INF;
    }
    double t1_e = -INF;
    double t2_e = -INF;
    if(t1(0) >= 0 && t20 >= 0 && t1(1) >= 0 && t21 >= 0){
        if(t1(0) + t20 <  t1(1) + t21){
            t1_e = t1(0);
            t2_e = t20;
        }
        else{
            t1_e = t1(1);
            t2_e = t21;
        }
    }
    else{
        if (t1(0) >= 0 && t20 >= 0) {
            t1_e = t1(0);
            t2_e = t20;
        } else if (t1(1) >= 0 && t21 >= 0) {
            t1_e = t1(1);
            t2_e = t21;
        }
    }


    double t1t_e = -INF;
    double t2t_e = -INF;
    if((t20t < 0.01 || t1t(0) < 0.01) && (t21t < 0.01 || t1t(1) < 0.01)){
//        std::cout<<" ";
    }
    if (t20t < 0 || t1t(0) < 0){
        t20t = -INF;
        t1t(0) = -INF;
    }
    if (t21t < 0 || t1t(1) < 0){
        t21t = -INF;
        t1t(1) = -INF;
    }
    if(t1t(0) >= 0 && t20t >= 0 && t1t(1) >= 0 && t21t >= 0){
        if(t1t(0) + t20t < t1t(1) + t21t){
            t1t_e = t1t(0);
            t2t_e = t20t;
        }
        else{
            t1t_e = t1t(1);
            t2t_e = t21t;
        }
    }
    else{
        if (t1t(0) >= 0 && t20t >= 0) {
            t1t_e = t1t(0);
            t2t_e = t20t;
        } else if (t1t(1) >= 0 && t21t >= 0) {
            t1t_e = t1t(1);
            t2t_e = t21t;
        }
    }

    if (t1_e >= 0 && t2_e >= 0 && t1t_e >= 0 && t2t_e >= 0) {
        if (t1_e + t2_e <= t1t_e + t2t_e) {
            time_durition << t1t_e, t2t_e, -1;
            return time_durition;

        } else if (t1_e + t2_e > t1t_e + t2t_e) {
            time_durition << t1_e, t2_e, 1;
            return time_durition;

        }
    } else {
        if (t1_e >= 0 && t2_e >= 0) {
            time_durition << t1_e, t2_e, 1;
            return time_durition;

        } else if (t1t_e >= 0 && t2t_e >= 0) {
            time_durition << t1t_e, t2t_e, -1;
            return time_durition;

        }
    }
    return time_durition;
}

Eigen::Vector4d
PointMassPathSearching::refineTperAxis(double v_init, double v_end, double distance, double t, double a_low_bond, double a_up_bond,
                                       double a_dir) {

    if (a_dir < 0) {
        double mid = a_low_bond;
        a_low_bond = a_up_bond;
        a_up_bond = mid;
    }
    if(fabs(v_init) < 1e-5){
        v_init = 0;
    }
    if(fabs(v_end) < 1e-5){
        v_end = 0;
    }
    Eigen::Vector4d time_durition;
    double a = v_init * a_up_bond - v_init * a_low_bond + (v_end - v_init) * a_up_bond / 2 - v_init * a_up_bond +
               v_init * a_low_bond - (v_end - v_init) * a_up_bond + (v_end - v_init) * a_low_bond / 2;
    double b = -distance * a_up_bond + distance * a_low_bond + v_init * a_low_bond * t + v_init * a_up_bond * t -
               v_init * a_low_bond * t + (v_end - v_init) * a_up_bond * t - v_init * a_low_bond * t -
               (v_end - v_init) * a_low_bond * t;
    double c = -distance * a_low_bond * t + v_init * a_low_bond * t * t + (v_end - v_init) * a_low_bond / 2 * t * t;

    double a_new = -(v_end - v_init) * a_up_bond;
    double b_new = 2 * (v_end - v_init) * a_up_bond * t + 2 * v_init * t * a_up_bond - 2 * distance * a_up_bond;
    double c_new = - a_up_bond * v_init * t * t - 0.5 * (v_end - v_init) * a_up_bond * t * t + a_up_bond * t * distance;

    Eigen::Vector2d t1 = solveQuadratic(a, b, c);


//    if (t1(0) < 0 && t1(1) < 0) {
//        a_dir = -1;
//        double mid = a_low_bond;
//        a_low_bond = a_up_bond;
//        a_up_bond = mid;
//        a = v_init * a_up_bond - v_init * a_low_bond + (v_end - v_init) * a_up_bond / 2 - v_init * a_up_bond +
//            v_init * a_low_bond - (v_end - v_init) * a_up_bond + (v_end - v_init) * a_low_bond / 2;
//        b = -distance * a_up_bond + distance * a_low_bond + v_init * a_low_bond * t + v_init * a_up_bond * t -
//            v_init * a_low_bond * t + (v_end - v_init) * a_up_bond * t - v_init * a_low_bond * t -
//            (v_end - v_init) * a_low_bond * t;
//        c = -distance * a_low_bond * t + v_init * a_low_bond * t * t + (v_end - v_init) * a_low_bond / 2 * t * t;
//        t1 = solveQuadratic(a, b, c);
//    }
    double t20 = t - t1(0);
    double t21 = t - t1(1);
    double alpha_1 = -INF;
    double alpha_2 = -INF;
    if(t1(0) >= 0 && t20 >= 0){
        if(v_end == v_init && (t1(0) - t20) < 1e-6){
            if(v_init == 0){
                alpha_1 = distance / (t1(0) * t1(0)) / a_up_bond;
            }
            else{
                alpha_1 = (2 * (- v_init) / t1(0)) / a_up_bond;
            }
        }
        else{
            alpha_1 = (v_end - v_init) / (a_up_bond * t1(0) + a_low_bond * t20);
        }
        time_durition << t1(0), t20, alpha_1, 1;

    }
    if(t1(1) >= 0 && t21 >= 0){
        if(v_end == v_init && (t1(1) - t21) < 1e-6){
            if(v_init == 0){
                alpha_2 = distance / (t1(1) * t1(1)) / a_up_bond;
            }
            else{
                alpha_2 = (2 * (- v_init) / t1(1)) / a_up_bond;
            }
        }
        else{
            alpha_2 = (v_end - v_init) / (a_up_bond * t1(1) + a_low_bond * t21);
        }
        time_durition << t1(1), t21, alpha_2, 1;
    }
//    if((alpha_1 > 0 && alpha_1 < 1) || (alpha_1 < 0 && alpha_1 > -1)){
//    }
//    if((alpha_2 > 0 && alpha_2 < 1) || (alpha_2 < 0 && alpha_2 > -1)){
//    }
//    if(v_end == v_init && distance == 0 && t1(0) == t20){
//        alpha_1 = (2 * (- v_init) / t1(0)) / a_up_bond;
//    }
    return time_durition;

//    if (a_dir < 0) {
//        double mid = a_low_bond;
//        a_low_bond = a_up_bond;
//        a_up_bond = mid;
//    }
//    // 先加再减计算时间
//    a = v_init * a_up_bond - v_init * a_low_bond + (v_end - v_init) * a_up_bond / 2 - v_init * a_up_bond +
//               v_init * a_low_bond - (v_end - v_init) * a_up_bond + (v_end - v_init) * a_low_bond / 2;
//    b = -distance * a_up_bond + distance * a_low_bond + v_init * a_low_bond * t + v_init * a_up_bond * t -
//               v_init * a_low_bond * t + (v_end - v_init) * a_up_bond * t - v_init * a_low_bond * t -
//               (v_end - v_init) * a_low_bond * t;
//    c = -distance * a_low_bond * t + v_init * a_low_bond * t * t + (v_end - v_init) * a_low_bond / 2 * t * t;
//
//    a_new = -(v_end - v_init) * a_up_bond;
//    b_new = 2 * (v_end - v_init) * a_up_bond * t + 2 * v_init * t * a_up_bond - 2 * distance * a_up_bond;
//    c_new = - a_up_bond * v_init * t * t - 0.5 * (v_end - v_init) * a_up_bond * t * t + a_up_bond * t * distance;
//
//    t1 = solveQuadratic(a, b, c);


//    if (t1(0) < 0 && t1(1) < 0) {
//        a_dir = -1;
//        double mid = a_low_bond;
//        a_low_bond = a_up_bond;
//        a_up_bond = mid;
//        a = v_init * a_up_bond - v_init * a_low_bond + (v_end - v_init) * a_up_bond / 2 - v_init * a_up_bond +
//            v_init * a_low_bond - (v_end - v_init) * a_up_bond + (v_end - v_init) * a_low_bond / 2;
//        b = -distance * a_up_bond + distance * a_low_bond + v_init * a_low_bond * t + v_init * a_up_bond * t -
//            v_init * a_low_bond * t + (v_end - v_init) * a_up_bond * t - v_init * a_low_bond * t -
//            (v_end - v_init) * a_low_bond * t;
//        c = -distance * a_low_bond * t + v_init * a_low_bond * t * t + (v_end - v_init) * a_low_bond / 2 * t * t;
//        t1 = solveQuadratic(a, b, c);
//    }
//    t20 = t - t1(0);
//    t21 = t - t1(1);
//    alpha_1 = -INF;
//    alpha_2 = -INF;
//    if(t1(0) > 0 && t20 > 0){
//        if(v_end == v_init && distance == 0 && t1(0) == t20){
//            alpha_1 = (2 * (- v_init) / t1(0)) / a_up_bond;
//        }
//        else{
//            alpha_1 = (v_end - v_init) / (a_up_bond * t1(0) + a_low_bond * t20);
//        }
//
//    }
//    if(t1(1) > 0 && t21 > 0){
//        if(v_end == v_init && distance == 0 && t1(1) == t21){
//            alpha_2 = (2 * (- v_init) / t1(1)) / a_up_bond;
//        }
//        else{
//            alpha_2 = (v_end - v_init) / (a_up_bond * t1(1) + a_low_bond * t21);
//        }
//
//    }
//    if((alpha_1 > 0 && alpha_1 < 1) || (alpha_1 < 0 && alpha_1 > -1)){
//        time_durition << t1(0), t20, alpha_1, 1;
//        return time_durition;
//    }
//    if((alpha_2 > 0 && alpha_2 < 1) || (alpha_2 < 0 && alpha_2 > -1)){
//        time_durition << t1(1), t21, alpha_2, 1;
//        return time_durition;
//    }
//    abort();

//    if(t1(0) > 0 && t20 > 0 && t1(1) > 0 && t21 > 0){
//        double alpha_1 = (v_end - v_init) / (a_up_bond * t1(0) + a_low_bond * t20);
//        double alpha_2 = (v_end - v_init) / (a_up_bond * t1(1) + a_low_bond * t21);
//        if(alpha_1 > 1 || alpha_1 < 0){
//            if(alpha_2 > 1 || alpha_2 < 0){
//                std::cout<<" "<<std::endl;
//                abort();
//            }
//            else{
//                time_durition << t1(1), t21, alpha_2;
//                return time_durition;
//            }
//        }
//        else{
//            time_durition << t1(0), t20, alpha_1;
//            return time_durition;
//            }
//    }
//    else{
//        if (t1(0) > 0 && t20 > 0) {
//            t1_e = t1(0);
//            t2_e = t20;
//        } else if (t1(1) > 0 && t21 > 0) {
//            t1_e = t1(1);
//            t2_e = t21;
//        }
//        double alpha;
//        if(v_end == v_init && distance == 0 && t1_e == t2_e){
//            alpha = (2 * (- v_init) / t1_e) / a_up_bond;
//        }
//        else{
//            alpha = (v_end - v_init) / (a_up_bond * t1_e + a_low_bond * t2_e);
//        }
//        if(alpha < -1 || alpha > 1){
//            std::cout<<" "<<std::endl;
//        }
//        time_durition << t1_e, t2_e, alpha;
//    }



//    double a = - a_bond;
//    double b = - (v_init - v_end + (t + (v_init - v_end) / a_bond ) * v_init + (a_bond * t - 3 * v_init + v_end - 2 * a_bond));
//    double c = distance - v_init - v_end - 1 / 2 * (v_init * v_init + v_end * v_end - 2 * v_init * v_end) / a_bond - (t - (v_init - v_end) / a_bond) * v_init;
//    double t1 = solveQuadratic(a, b, c);
//    double t3 = (v_init - v_end) / a_bond + t1;
//    double t2 = t - t1 - t3;
//    time_durition << t1, t2, t3;
//    return time_durition;
}

Eigen::Vector2d PointMassPathSearching::solveQuadratic(double a, double b, double c) {
    if ((b * b - 4 * a * c) < 0) {
        return Eigen::Vector2d{-INF, -INF};
    }
    if(a == 0){
        return (Eigen::Vector2d () << fabs(- c / b), fabs(- c / b)).finished();
    }
    double x1 = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
    double x2 = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
    if (x1 == -0){
        x1 = -0.001;
    }
    if(x2 == -0){
        x2 = -0.001;
    }
    return Eigen::Vector2d{x1, x2};
}

void PointMassPathSearching::getWaypointDistance() {
    for (auto it = waypoints_.begin(); (it + 1) != waypoints_.end(); it++) {
        //计算当前wp相对于下一个wp的位置距离
        distance_between_waypoint_.emplace_back((*(it + 1)).position - (*it).position);
        //在每个wp处进行速度采样，最后一个wp不需要采样
    }
}

std::vector<OptimalNode>
PointMassPathSearching::solve(std::deque<waypoint> waypointVec, const Eigen::Vector3d &currentPosition,
                              const Eigen::Vector3d &currentVelcity, const Eigen::Vector4d &currentAttitude, bool touch_goal, bool is_fix) {

    if(is_fix){
        current_velcity_ = currentVelcity;
        current_position_ = currentPosition;
        current_attitude_ = currentAttitude;
        waypoints_ = std::move(waypointVec);
        distance_between_waypoint_.clear();
        getWaypointDistance();
        pointmass_position_.clear();
        pointmass_velocity_.clear();
        pointmass_acc_.clear();
        Velocity_Network_.at(0).begin()->Velocity_ = currentVelcity;
        wayPoint_Optimal_VelocityVec_.clear();
        reverse();
        return wayPoint_Optimal_VelocityVec_;
    }
    else{
        reset();
        current_velcity_ = currentVelcity;
        current_position_ = currentPosition;
        current_attitude_ = currentAttitude;
        waypoints_ = std::move(waypointVec);
        for(const auto& waypoint: waypoints_){
            velocity_direction_prior_from_EGOPLANNER_.push_back(waypoint.velocity_direction);
        }
        getWaypointDistance();
    }
        preSearch(touch_goal);
//        cleanPreSample();
//        sampleVelocity(currentVelcity);
//        shortestPathSearching();
//        reverse();
//    }
    return wayPoint_Optimal_VelocityVec_;
}

double PointMassPathSearching::calculateJ(Eigen::Vector3d v_init, Eigen::Vector3d v_end, Eigen::Vector3d distance) {
    double t = calculateDuritionTinSample(std::move(v_init), std::move(v_end), std::move(distance));
    return t;
}

void PointMassPathSearching::sampleVelocity(Eigen::Vector3d curVelocity) {

    Velocity_Network_.emplace_back(std::vector<node>{node(false, 0, std::move(curVelocity), "start", 0)});
    for (int index = 0; index < preSampleVelocity.size(); index++) {
        Velocity_Network_.emplace_back(std::vector<node>{node(false, INF, preSampleVelocity.at(index), "node", index + 1)});
        auto instance = &Velocity_Network_.at(index + 1);
        Eigen::Matrix<double, 40, 3> vel_sample = coneSample(instance->begin()->Velocity_);
        for (int i = 0; i < vel_sample.rows(); i++) {
            instance->emplace_back(node(false, INF, vel_sample.row(i), "node", instance->begin()->num_wp));
        }
    }
    Velocity_Network_.emplace_back(
            std::vector<node>{node(false, INF, Eigen::Vector3d{0, 0, 0}, "end", static_cast<int>(distance_between_waypoint_.size()))});
}

void PointMassPathSearching::preSearch(bool touch_goal) {
    Velocity_Network_.emplace_back(std::vector<node>{node(false, 0, current_velcity_, "start", 0)});
    int wp_num = static_cast<int>(waypoints_.size()) - 2;
    for (int i = 0; i < wp_num; i++) {
        Velocity_Network_.emplace_back(std::vector<node>{node(false, INF, Eigen::Vector3d{0, 0, 0}, "node", i + 1)});
        auto instance = &Velocity_Network_.at(i + 1);
        Eigen::Matrix<float, 3, 21> normVel_gate_dir = normSample(velocity_direction_prior_from_EGOPLANNER_[instance->begin()->num_wp]);
        for (int j = 0; j < 20; j++) {
            instance->emplace_back(false, INF, Eigen::Vector3d {normVel_gate_dir(0, j), normVel_gate_dir(1, j), normVel_gate_dir(2, j)}, "node", instance->begin()->num_wp, j + 1);
        }
        instance->erase(instance->begin());
    }
    if (touch_goal)
    {
        Velocity_Network_.emplace_back(
            std::vector<node>{node(false, INF, Eigen::Vector3d{0, 0, 0}, "end", static_cast<int>(distance_between_waypoint_.size()))});
    }else{
        int end_wp_idx = static_cast<int>(distance_between_waypoint_.size());
        Velocity_Network_.emplace_back(std::vector<node>{node(false, INF, Eigen::Vector3d{0, 0, 0}, "end", end_wp_idx)});
        auto instance = &Velocity_Network_.back();
        Eigen::Vector3d end_direction = velocity_direction_prior_from_EGOPLANNER_.back();
        Eigen::Matrix<float, 3, 21> normVel_gate_dir = normSample(end_direction);
        for (int j = 0; j < 20; j++) {
            instance->emplace_back(false, INF, Eigen::Vector3d {normVel_gate_dir(0, j), normVel_gate_dir(1, j), normVel_gate_dir(2, j)}, "end", end_wp_idx, j + 1);
        }
        instance->erase(instance->begin());
    }
    shortestPathSearching();
    reverse();
//    for (const auto &item: wayPoint_Optimal_VelocityVec_) {
//        preSampleVelocity.emplace_back(item.Velocity_);
//    }
//    preSampleVelocity.pop_back();
//    preSampleVelocity.pop_front();

}

Eigen::Matrix<double, 40, 3> PointMassPathSearching::coneSample(Eigen::Vector3d conicalSpindle) {
    Eigen::Matrix<double, 3, 1> conical_qi = (Eigen::Matrix<double, 3, 1>() << conicalSpindle(0), conicalSpindle(1), conicalSpindle(2)).finished();
    Eigen::Matrix<double, 3, 1> cone_ = {0, 0, 1};
    auto rotMatrix = Eigen::Quaterniond::FromTwoVectors(cone_, conical_qi).toRotationMatrix();
    Eigen::Matrix<double, 40, 1> cone_x, cone_y, cone_z;
    cone_x << 0.0,0.14255886355189365,0.23066508662452245,0.23066508662452248,0.14255886355189368,2.970204768799478e-17,-0.14255886355189362,-0.23066508662452245,-0.23066508662452248,-0.1425588635518937,0.0,0.2628655560595668,0.42532540417601994,0.42532540417602,0.26286555605956685,5.476786982642026e-17,-0.26286555605956674,-0.42532540417601994,-0.42532540417602,-0.2628655560595669,0.0,0.3526711513754839,0.570633909777092,0.5706339097770922,0.35267115137548394,7.347880794884119e-17,-0.3526711513754838,-0.570633909777092,-0.5706339097770922,-0.352671151375484,0.0,0.4156269377774534,0.6724985119639573,0.6724985119639574,0.41562693777745346,8.659560562354933e-17,-0.41562693777745335,-0.6724985119639573,-0.6724985119639574,-0.4156269377774536;
    cone_y << 0.24253562503633297,0.19621544239574334,0.07494762987757687,-0.07494762987757685,-0.19621544239574332,-0.24253562503633297,-0.19621544239574337,-0.0749476298775769,0.07494762987757682,0.19621544239574332,0.4472135954999579,0.36180339887498947,0.13819660112501053,-0.13819660112501048,-0.3618033988749894,-0.4472135954999579,-0.3618033988749895,-0.13819660112501056,0.13819660112501042,0.3618033988749894,0.6,0.4854101966249685,0.18541019662496847,-0.1854101966249684,-0.48541019662496837,-0.6,-0.48541019662496854,-0.18541019662496855,0.18541019662496833,0.48541019662496837,0.7071067811865475,0.5720614028176843,0.21850801222441055,-0.21850801222441046,-0.5720614028176843,-0.7071067811865475,-0.5720614028176844,-0.21850801222441063,0.21850801222441038,0.5720614028176843;
    cone_z << 0.9701425001453319,0.9701425001453319,0.9701425001453319,0.9701425001453319,0.9701425001453319,0.9701425001453319,0.9701425001453319,0.9701425001453319,0.9701425001453319,0.9701425001453319,0.8944271909999159,0.8944271909999159,0.8944271909999159,0.8944271909999159,0.8944271909999159,0.8944271909999159,0.8944271909999159,0.8944271909999159,0.8944271909999159,0.8944271909999159,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.8,0.7071067811865475,0.7071067811865475,0.7071067811865475,0.7071067811865475,0.7071067811865475,0.7071067811865475,0.7071067811865475,0.7071067811865475,0.7071067811865475,0.7071067811865475;
    Eigen::Matrix<double, 40, 3> cone_mat;
    Eigen::Matrix<double, 40, 3> new_cone_vel;
    cone_mat.col(0) = cone_x;
    cone_mat.col(1) = cone_y;
    cone_mat.col(2) = cone_z;
    new_cone_vel = (rotMatrix * cone_mat.transpose()).transpose() * conical_qi.norm();
    return new_cone_vel;
}
Eigen::Matrix<double, 20, 3> PointMassPathSearching::coneSample_old(Eigen::Vector3d conicalSpindle) {
    Eigen::Matrix<double, 4, 1> conical_qi = (Eigen::Matrix<double, 4, 1>() << conicalSpindle(0), conicalSpindle(1), conicalSpindle(2), 1).finished();
    Eigen::Matrix<double, 20, 4> sampleMat;
    Eigen::Matrix<double, 20, 1> samplefactor;
    Eigen::Matrix<double, 20, 3> sampledVelocity;
    samplefactor << -2, -1.8, -1.6, -1.4, -1.2, -1, -0.8, -0.6, -0.4, -0.2, 0.2, 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0;
    sampleMat.col(0) = Eigen::Matrix<double, 20, 1>::Zero();
    sampleMat.col(1) = Eigen::Matrix<double, 20, 1>::Zero();
    sampleMat.col(2) = Eigen::Matrix<double, 20, 1>::Zero();
    sampleMat.col(3) = samplefactor;
    Eigen::Matrix<double, 20, 1> x_vel;
    Eigen::Matrix<double, 20, 1> y_vel;
    Eigen::Matrix<double, 20, 1> z_vel;
    sampleMat.col(0) = Eigen::Matrix<double, 20, 1>::Zero();
    sampleMat.col(0) = Eigen::Matrix<double, 20, 1>::Ones();
    x_vel = sampleMat * conical_qi;
    sampleMat.col(0) = Eigen::Matrix<double, 20, 1>::Zero();
    sampleMat.col(1) = Eigen::Matrix<double, 20, 1>::Ones();
    y_vel = sampleMat * conical_qi;
    sampleMat.col(1) = Eigen::Matrix<double, 20, 1>::Zero();
    sampleMat.col(2) = Eigen::Matrix<double, 20, 1>::Ones();
    z_vel = sampleMat * conical_qi;
    sampledVelocity.col(0) = x_vel;
    sampledVelocity.col(1) = y_vel;
    sampledVelocity.col(2) = z_vel;
    return sampledVelocity;
}

Eigen::Matrix<float, 3, 21> PointMassPathSearching::normSample(Eigen::Vector3d cone_direction) {
    Eigen::Vector3d cone_dir_norm = cone_direction.normalized();
    Eigen::Matrix<float, 3, 1> cone_dir;
    cone_dir << cone_dir_norm(0), cone_dir_norm(1), cone_dir_norm(2);
    Eigen::Matrix<float, 1, 21> vel_mat;
    vel_mat << 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21;
    return cone_dir * vel_mat / 2;
}

bool PointMassPathSearching::shortestPathSearching() {

    std::priority_queue<node *, std::vector<node *>, cmp> expanded_nodes;
    int search_index = 0;
    expanded_nodes.push(&(*((Velocity_Network_.begin())->begin())));
    while (true) {
        if (expanded_nodes.empty()) {
            return false;
        }
        auto cur_node = &(*expanded_nodes.top());
        cur_node->is_arrived = true;
        if (cur_node->flag == "end") {
//            std::cout<<search_index<<std::endl;
            return true;
        }
        expanded_nodes.pop();
        for (auto it = Velocity_Network_[cur_node->num_wp + 1].begin();
             it != Velocity_Network_[cur_node->num_wp + 1].end(); it++) {
//            if (it->num_wp == 7) {
//                std::cout << std::endl;
//            }
            if (it->g_from_begin > 1000) {
                search_index++;
                it->father_node = cur_node;
                it->g_from_begin = cur_node->g_from_begin + calculateJ(cur_node->Velocity_, it->Velocity_,
                                                                              distance_between_waypoint_[cur_node->num_wp]);
                expanded_nodes.push(&(*(it)));
            } else if (it->g_from_begin > cur_node->g_from_begin + calculateJ(cur_node->Velocity_, it->Velocity_,
                                                                              distance_between_waypoint_[cur_node->num_wp])) {
                search_index++;
                it->g_from_begin = cur_node->g_from_begin + calculateJ(cur_node->Velocity_, it->Velocity_,
                                                                       distance_between_waypoint_[cur_node->num_wp]);
                it->father_node = cur_node;
            }
        }
        close_list.push_back(cur_node);
    }
}

void PointMassPathSearching::reverse() {

    auto it = Velocity_Network_[Velocity_Network_.size() - 1][0];
//    std::cout << it.g_from_begin<<std::endl;
    while (it.flag != "start") {
        OptimalNode cur_node(it);
        calculateDuritonT(cur_node, cur_node.father_node->Velocity_, cur_node.Velocity_,
                          distance_between_waypoint_[cur_node.father_node->num_wp]);
        wayPoint_Optimal_VelocityVec_.push_back(cur_node);
        it = *(it.father_node);
    }
    OptimalNode cur_node(it);
    wayPoint_Optimal_VelocityVec_.push_back(cur_node);
    std::reverse(wayPoint_Optimal_VelocityVec_.begin(), wayPoint_Optimal_VelocityVec_.end());
}

double
PointMassPathSearching::calculateDuritionTinSample(Eigen::Vector3d v_init, Eigen::Vector3d v_end, Eigen::Vector3d distance) const {
    Eigen::Vector3d x_time = calculateTperAxis(v_init[0], v_end[0], distance[0], a_low_bond_x, a_up_bond_x);
    Eigen::Vector3d Y_time = calculateTperAxis(v_init[1], v_end[1], distance[1], a_low_bond_y, a_up_bond_y);
    Eigen::Vector3d Z_time = calculateTperAxis(v_init[2], v_end[2], distance[2], a_low_bond_z, a_up_bond_z);
    double t_x = x_time(0) + x_time(1);
    double t_y = Y_time(0) + Y_time(1);
    double t_z = Z_time(0) + Z_time(1);
    if (t_x >= t_y && t_x >= t_z) {
        return t_x;
    } else if (t_y >= t_x && t_y >= t_z) {
        return t_y;
    } else if (t_z >= t_x && t_z >= t_y) {
        return t_z;
    } else {
        return INF;
    }
}

void PointMassPathSearching::drawTrajectory(double delta_t) {

    for (auto pp = wayPoint_Optimal_VelocityVec_.size() - 1; pp > 0; pp--) {
        auto it = &wayPoint_Optimal_VelocityVec_[pp];
        std::vector<Eigen::Vector3d> segPos;
        std::vector<Eigen::Vector3d> segVel;
        std::vector<Eigen::Vector3d> segAcc;
//        segTrag.emplace_back(waypoints_[it->father_node->num_wp]);
        for (int i = 0; i < static_cast<int>((it->g_from_begin - it->father_node->g_from_begin) / delta_t); i++) {
            Eigen::Vector3d curPos;
            Eigen::Vector3d curVel;
            Eigen::Vector3d curAcc;
            double t = i * delta_t;
            if(it->x_time(0) == it->x_time(1) && it->x_time(0) == 0){
                curPos(0) = waypoints_[it->father_node->num_wp].position(0);
                curVel(0) = 0;
                curAcc(0) = 0;
            }
            else{
                if (t < it->x_time(0)) {
                    if (it->first_a(0) < 0) {
                        curPos(0) = waypoints_[it->father_node->num_wp].position(0) + it->father_node->Velocity_(0) * t +
                                    0.5 * a_low_bond_x * it->alpha_from_father_node(0) * t * t;
                        curVel(0) = it->father_node->Velocity_(0) + a_low_bond_x * it->alpha_from_father_node(0) * t;
                        curAcc(0) = a_low_bond_x * it->alpha_from_father_node(0);
                    } else {
                        curPos(0) = waypoints_[it->father_node->num_wp].position(0) + it->father_node->Velocity_(0) * t +
                                    0.5 * a_up_bond_x * it->alpha_from_father_node(0) * t * t;
                        curVel(0) = it->father_node->Velocity_(0) + a_up_bond_x * it->alpha_from_father_node(0) * t;
                        curAcc(0) = a_up_bond_x * it->alpha_from_father_node(0);
                    }
                } else {
                    if (it->first_a(0) < 0) {
                        curPos(0) = it->position_shift(0) + it->velocity_shift(0) * (t - it->x_time(0)) +
                                    0.5 * a_up_bond_x * it->alpha_from_father_node(0) * (t - it->x_time(0)) *
                                    (t - it->x_time(0));
                        curVel(0) =
                                it->velocity_shift(0) + a_up_bond_x * it->alpha_from_father_node(0) * (t - it->x_time(0));
                        curAcc(0) = a_up_bond_x * it->alpha_from_father_node(0);
                    } else {
                        curPos(0) = it->position_shift(0) + it->velocity_shift(0) * (t - it->x_time(0)) +
                                    0.5 * a_low_bond_x * it->alpha_from_father_node(0) * (t - it->x_time(0)) *
                                    (t - it->x_time(0));
                        curVel(0) =
                                it->velocity_shift(0) + a_low_bond_x * it->alpha_from_father_node(0) * (t - it->x_time(0));
                        curAcc(0) = a_low_bond_x * it->alpha_from_father_node(0);
                    }
                }
            }
            if(it->y_time(0) == it->y_time(1) && it->y_time(0) == 0){
                curPos(1) = waypoints_[it->father_node->num_wp].position(1);
                curVel(1) = 0;
                curAcc(1) = 0;
            }
            else{
                if (t < it->y_time(0)) {
                    if (it->first_a(1) < 0) {
                        curPos(1) = waypoints_[it->father_node->num_wp].position(1) + it->father_node->Velocity_(1) * t +
                                    0.5 * a_low_bond_y * it->alpha_from_father_node(1) * t * t;
                        curVel(1) = it->father_node->Velocity_(1) + a_low_bond_y * it->alpha_from_father_node(1) * t;
                        curAcc(1) = a_low_bond_y * it->alpha_from_father_node(1);
                    } else {
                        curPos(1) = waypoints_[it->father_node->num_wp].position(1) + it->father_node->Velocity_(1) * t +
                                    0.5 * a_up_bond_y * it->alpha_from_father_node(1) * t * t;
                        curVel(1) = it->father_node->Velocity_(1) + a_up_bond_y * it->alpha_from_father_node(1) * t;
                        curAcc(1) = a_up_bond_y * it->alpha_from_father_node(1);
                    }
                } else {
                    if (it->first_a(1) < 0) {
                        curPos(1) = it->position_shift(1) + it->velocity_shift(1) * (t - it->y_time(0)) +
                                    0.5 * a_up_bond_y * it->alpha_from_father_node(1) * (t - it->y_time(0)) *
                                    (t - it->y_time(0));
                        curVel(1) =
                                it->velocity_shift(1) + a_up_bond_y * it->alpha_from_father_node(1) * (t - it->y_time(0));
                        curAcc(1) = a_up_bond_y * it->alpha_from_father_node(1);
                    } else {
                        curPos(1) = it->position_shift(1) + it->velocity_shift(1) * (t - it->y_time(0)) +
                                    0.5 * a_low_bond_y * it->alpha_from_father_node(1) * (t - it->y_time(0)) *
                                    (t - it->y_time(0));
                        curVel(1) =
                                it->velocity_shift(1) + a_low_bond_y * it->alpha_from_father_node(1) * (t - it->y_time(0));
                        curAcc(1) = a_low_bond_y * it->alpha_from_father_node(1);
                    }
                }
            }
            if(it->z_time(0) == it->z_time(1) && it->z_time(0) == 0){
                curPos(2) = waypoints_[it->father_node->num_wp].position(2);
                curVel(2) = 0;
                curAcc(2) = 0;
            }
            else{
                if (t < it->z_time(0)) {
                    if (it->first_a(2) < 0) {
                        curPos(2) = waypoints_[it->father_node->num_wp].position(2) + it->father_node->Velocity_(2) * t +
                                    0.5 * a_low_bond_z * it->alpha_from_father_node(2) * t * t;
                        curVel(2) = it->father_node->Velocity_(2) + a_low_bond_z * it->alpha_from_father_node(2) * t;
                        curAcc(2) = a_low_bond_z * it->alpha_from_father_node(2);
                    } else {
                        curPos(2) = waypoints_[it->father_node->num_wp].position(2) + it->father_node->Velocity_(2) * t +
                                    0.5 * a_up_bond_z * it->alpha_from_father_node(2) * t * t;
                        curVel(2) = it->father_node->Velocity_(2) + a_up_bond_z * it->alpha_from_father_node(2) * t;
                        curAcc(2) = a_up_bond_z * it->alpha_from_father_node(2);
                    }
                } else {
                    if (it->first_a(2) < 0) {
                        curPos(2) = it->position_shift(2) + it->velocity_shift(2) * (t - it->z_time(0)) +
                                    0.5 * a_up_bond_z * it->alpha_from_father_node(2) * (t - it->z_time(0)) *
                                    (t - it->z_time(0));
                        curVel(2) =
                                it->velocity_shift(2) + a_up_bond_z * it->alpha_from_father_node(2) * (t - it->z_time(0));
                        curAcc(2) = a_up_bond_z * it->alpha_from_father_node(2);
                    } else {
                        curPos(2) = it->position_shift(2) + it->velocity_shift(2) * (t - it->z_time(0)) +
                                    0.5 * a_low_bond_z * it->alpha_from_father_node(2) * (t - it->z_time(0)) *
                                    (t - it->z_time(0));
                        curVel(2) =
                                it->velocity_shift(2) + a_low_bond_z * it->alpha_from_father_node(2) * (t - it->z_time(0));
                        curAcc(2) = a_low_bond_z * it->alpha_from_father_node(2);
                    }
                }
            }
            segPos.push_back(curPos);
            segVel.push_back(curVel);
            segAcc.push_back(curAcc);
        }
        pointmass_position_.push_back(segPos);
        pointmass_velocity_.push_back(segVel);
        pointmass_acc_.push_back(segAcc);
        std::reverse(segPos.begin(), segPos.end());
        std::reverse(segVel.begin(), segVel.end());
        std::reverse(segAcc.begin(), segAcc.end());
//        if(get_wp_num_begin == -1){
//            for(int i =0; i < segPos.size();i++){
//                Position->push_back(segPos.at(i));
//                Velocity->push_back(segVel.at(i));
//                Acce->push_back(segAcc.at(i));
//            }
//        } else{
//            if(pp > get_wp_num_begin && pp <= get_wp_num_end){
//                for(int i =0; i < segPos.size();i++){
//                    Position->push_back(segPos.at(i));
//                    Velocity->push_back(segVel.at(i));
//                    Acce->push_back(segAcc.at(i));
//                }
//            }
//        }

//        for (auto &ij: segPos) {
//            Position->push_back(ij);
//        }
//        for (auto &ijk: segVel) {
//            Velocity->push_back(ijk);
//        }
//        for (auto &ijkl: segAcc) {
//            Acce->push_back(ijkl);
//        }
    }
//    std::reverse(Position->begin(), Position->end());
//    std::reverse(Velocity->begin(), Velocity->end());
//    std::reverse(Acce->begin(), Acce->end());
    std::reverse(pointmass_position_.begin(), pointmass_position_.end());
    std::reverse(pointmass_velocity_.begin(), pointmass_velocity_.end());
    std::reverse(pointmass_acc_.begin(), pointmass_acc_.end());

//    std::ofstream file;
//    file.open("/home/zfg/Graduate_Test/RAL24/PointMassTraj/traj.py", std::ios::app);
//    file << "Pos=[";
//    for (auto i = Position->begin();i != Position->end();i++) {
//        if(i == std::prev(Position->end())){
//            file << "[" << (*i)(0) << "," << (*i)(1) << "," << (*i)(2) << "]" << std::endl;
//            break;
//        }
//        file << "[" << (*i)(0) << "," << (*i)(1) << "," << (*i)(2) << "]" << "," << std::endl;
//    }
//    file << "]" << std::endl;
//    file << "Vel=[";
//    for (auto i = Velocity->begin();i != Velocity->end();i++) {
//        if(i == std::prev(Velocity->end())){
//            file << "[" << (*i)(0) << "," << (*i)(1) << "," << (*i)(2) << "]" << std::endl;
//            break;
//        }
//        file << "[" << (*i)(0) << "," << (*i)(1) << "," << (*i)(2) << "]" << "," << std::endl;
//    }
//    file << "]" << std::endl;
//    file << "acc=[";
//    for (auto i = Acce->begin();i != Acce->end();i++) {
//        if(i == std::prev(Acce->end())){
//            file << "[" << (*i)(0) << "," << (*i)(1) << "," << (*i)(2) << "]" << std::endl;
//            break;
//        }
//        file << "[" << (*i)(0) << "," << (*i)(1) << "," << (*i)(2) << "]" << "," << std::endl;
//    }
//    file << "]" << std::endl;
//    file.close();
//    std::vector<std::thread> thread_list;
//    std::vector<std::future<void>> futures;
//    auto start = std::chrono::system_clock::now();
//    for(int i = 0; i < single_waypoint_plan_list_.size(); i++){
//        single_waypoint_plan_list_.at(i).reset();
//        if(i == 0){
//            single_waypoint_plan_list_.at(i).set_parameters(waypoints_.at(i + 1).position, wayPoint_Optimal_VelocityVec_.at(i + 1).Velocity_, current_position_, current_velcity_, current_attitude_);
//        }
//        else{
//            single_waypoint_plan_list_.at(i).set_parameters(waypoints_.at(i + 1).position, wayPoint_Optimal_VelocityVec_.at(i + 1).Velocity_, waypoints_.at(i).position, wayPoint_Optimal_VelocityVec_.at(i).Velocity_,
//                                                            calculateAttitudefromAcce(*(std::prev(pointmass_acc_.at(i - 1).end())), *(std::prev(pointmass_yaw.at(i - 1).end()))));
//        }
//        single_waypoint_plan_list_.at(i).futurepathCallback(pointmass_position_.at(i), pointmass_velocity_.at(i), pointmass_acc_.at(i), pointmass_yaw.at(i));
//        single_waypoint_plan_list_.at(i).create_full_formulation();
//        futures.emplace_back(std::async(std::launch::async, &SingleWaypointPlan::process, &single_waypoint_plan_list_.at(i), &optimal_position_.at(i), &optimal_velocity_.at(i), &optimal_attitude_.at(i), &optimal_omega_.at(i), &optimal_thrust_.at(i), &optimal_time.at(i)));
//    }
//    for (auto& future : futures) {
//        future.get();
//    }
//    auto end = std::chrono::system_clock::now();
//    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
//    std::cout <<  "全部求解花费"
//              << double(duration.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den
//              << "秒" << std::endl;
//    std::cout<<"轨迹时间："<< std::endl;
//    double full_time = 0;
//    for (auto item : optimal_time){
//        std::cout<<item<<", ";
//        full_time += item;
//    }
//    std::cout<<std::endl<<full_time<<std::endl;
//    for(int i = 0; i < num_of_full_waypoints_seg_; i++){
//        while (optimal_position_.at(i).empty()){
//            std::cout<<"the "<<i<<" waypoints finish"<<std::endl;
//        }
//    }
//    std::ofstream file_o;
//    file_o.open("/home/zfg/Graduate_Test/RAL24/PointMassTraj/full_traj_opti.py", std::ios::out);
////    file_o << "t=" << mr.T_optimal_ << std::endl;
//    file_o << "Pos=[";
//    for(int j = 0; j < optimal_position_.size();j++){
//        for (auto i = optimal_position_.at(j).begin(); i != optimal_position_.at(j).end(); i++) {
//            if (i == std::prev(optimal_position_.at(j).end()) && j == optimal_position_.size() - 1) {
//                file_o << "[" << (*i)(0) << "," << (*i)(1) << "," << (*i)(2) << "]" << std::endl;
//                break;
//            }
//            file_o << "[" << (*i)(0) << "," << (*i)(1) << "," << (*i)(2) << "]" << "," << std::endl;
//        }
//    }
//    file_o << "]" << std::endl;
//    file_o << "Vel=[";
//    for(int j = 0; j < optimal_velocity_.size();j++){
//        for (auto i = optimal_velocity_.at(j).begin(); i != optimal_velocity_.at(j).end(); i++) {
//            if (i == std::prev(optimal_velocity_.at(j).end())&& j == optimal_velocity_.size() - 1) {
//                file_o << "[" << (*i)(0) << "," << (*i)(1) << "," << (*i)(2) << "]" << std::endl;
//                break;
//            }
//            file_o << "[" << (*i)(0) << "," << (*i)(1) << "," << (*i)(2) << "]" << "," << std::endl;
//        }
//    }
//    file_o << "]" << std::endl;
//    file_o << "att=[";
//    for(int j = 0; j < optimal_attitude_.size();j++){
//        for (auto i = optimal_attitude_.at(j).begin(); i != optimal_attitude_.at(j).end(); i++) {
//            if (i == std::prev(optimal_attitude_.at(j).end())&& j == optimal_attitude_.size() - 1) {
//                file_o << "[" << (*i)(0) << "," << (*i)(1) << "," << (*i)(2) << "," << (*i)(3) << "]" << std::endl;
//                break;
//            }
//            file_o << "[" << (*i)(0) << "," << (*i)(1) << "," << (*i)(2) << "," << (*i)(3) << "]," << std::endl;
//        }
//    }
//    file_o << "]" << std::endl;
//    file_o << "omega=[";
//    for(int j = 0; j < optimal_omega_.size();j++){
//        for (auto i = optimal_omega_.at(j).begin(); i != optimal_omega_.at(j).end(); i++) {
//            if (i == std::prev(optimal_omega_.at(j).end())&& j == optimal_omega_.size() - 1) {
//                file_o << "[" << (*i)(0) << "," << (*i)(1) << "," << (*i)(2) << "]" << std::endl;
//                break;
//            }
//            file_o << "[" << (*i)(0) << "," << (*i)(1) << "," << (*i)(2) << "]" << "," << std::endl;
//        }
//    }
//    file_o << "]" << std::endl;
//    file_o << "thrust=[";
//    for(int j = 0; j < optimal_thrust_.size();j++){
//        for (auto i = optimal_thrust_.at(j).begin(); i != optimal_thrust_.at(j).end(); i++) {
//            if (i == std::prev(optimal_thrust_.at(j).end())&& j == optimal_thrust_.size() - 1) {
//                file_o << "[" << (*i) << "]" << std::endl;
//                break;
//            }
//            file_o << "[" << (*i) << "]" << "," << std::endl;
//        }
//    }
//    file_o << "]" << std::endl;
//    file_o.close();
//    std::cout<<" ";
}

void PointMassPathSearching::reset() {
    for(int i = 0; i < optimal_position_.size(); i++){
        optimal_position_.at(i).clear();
        optimal_velocity_.at(i).clear();
        optimal_attitude_.at(i).clear();
        optimal_omega_.at(i).clear();
        optimal_thrust_.at(i).clear();
    }
    pointmass_position_.clear();
    pointmass_velocity_.clear();
    pointmass_acc_.clear();
    std::deque<Eigen::Vector3d>().swap(preSampleVelocity);
    std::vector<std::vector<node>>().swap(Velocity_Network_);
    // 存放搜索到的最快路径每一段的三个时间
    std::vector<Eigen::Vector3d>().swap(wayPoint_Optimal_TimeVec_);
    // 存放搜索到的最快路径每一个waypoint处的速度
    std::vector<OptimalNode>().swap(wayPoint_Optimal_VelocityVec_);
    // 存放每一个wp之间的距离
    std::vector<Eigen::Vector3d>().swap(distance_between_waypoint_);
    std::vector<Eigen::Vector3d>().swap(velocity_direction_prior_from_EGOPLANNER_);
    std::deque<waypoint>().swap(waypoints_);
    std::vector<node *>().swap(close_list);
}

void PointMassPathSearching::cleanPreSample() {
    std::vector<std::vector<node>>().swap(Velocity_Network_);
    // 存放搜索到的最快路径每一段的三个时间
    std::vector<Eigen::Vector3d>().swap(wayPoint_Optimal_TimeVec_);
    // 存放搜索到的最快路径每一个waypoint处的速度
    std::vector<OptimalNode>().swap(wayPoint_Optimal_VelocityVec_);
    // 存放每一个wp之间的距离
    std::vector<node *>().swap(close_list);
}

std::vector<Eigen::Vector3d> PointMassPathSearching::getOptimalVelocity() const {
    std::vector<Eigen::Vector3d> optimal_vel_list;
    for(int i = 1; i < wayPoint_Optimal_VelocityVec_.size(); i++){
        auto item = wayPoint_Optimal_VelocityVec_.at(i);
//        std::cout<<i<< " : " << item.Velocity_(0) << " " << item.Velocity_(1) << " " << item.Velocity_(2) << std::endl;
        optimal_vel_list.push_back(item.Velocity_);
    }
//    for(auto item : optimal_vel_list){
//        std::cout<<"vel: "<<item(0)<< " " << item(1)<< " " << item(2)<<std::endl;
//    }
    return optimal_vel_list;
}

void PointMassPathSearching::saveTrajectory2file() {
    std::ofstream file;
    file.open("/home/zfg/Graduate_Test/RAL24/VelocityDirection/1.txt", std::ios::out);
    std::vector<Eigen::Vector3d> pos, vel, acc;
    drawTrajectory(0.03);
    for(int i = 0;i < pos.size();i++){
        file<<pos.at(i)(0)<<","<<pos.at(i)(1)<<","<<pos.at(i)(2)<<","<<vel.at(i)(0)<<","<<vel.at(i)(1)<<","<<vel.at(i)(2)<<","<<acc.at(i)(0)<<","<<acc.at(i)(1)<<","<<acc.at(i)(2)<<std::endl;
    }
    auto optimal_vel = getOptimalVelocity();
    for(int i = 0; i < optimal_vel.size(); i++){
        file << optimal_vel.at(i)(0)<< "," << optimal_vel.at(i)(1)<< ", " << optimal_vel.at(i)(2)<<std::endl;
    }
    file.close();
}

std::vector<float> PointMassPathSearching::getOptimalTime() const {
    std::vector<float> optimal_time_list;
    for(int i = 1; i < wayPoint_Optimal_VelocityVec_.size(); i++){
        auto item = wayPoint_Optimal_VelocityVec_.at(i);
//        std::cout<<i<< " : " << item.Velocity_(0) << " " << item.Velocity_(1) << " " << item.Velocity_(2) << std::endl;
        if(i == 1){
            optimal_time_list.push_back(static_cast<float>(item.g_from_begin));

        }
        else{
            optimal_time_list.push_back(static_cast<float>(item.g_from_begin - wayPoint_Optimal_VelocityVec_.at(i-1).g_from_begin));
        }
    }
//    for(auto item : optimal_vel_list){
//        std::cout<<"vel: "<<item(0)<< " " << item(1)<< " " << item(2)<<std::endl;
//    }
    return optimal_time_list;
}

Eigen::Vector4d PointMassPathSearching::calculateAttitudefromAcce(const Eigen::Vector3d acc, const double yaw) {
    Eigen::Vector3d zb_des = acc / acc.norm();
    Eigen::Vector3d yc(-sin(yaw), cos(yaw), 0);
    Eigen::Vector3d xb_des = yc.cross(zb_des) / (yc.cross(zb_des)).norm();
    Eigen::Vector3d yb_des = zb_des.cross(xb_des);

    double psi_des = atan2(xb_des[1], xb_des[0]);
    double theta_des = asin(-xb_des[2]);
    double phi_des = atan(yb_des[2] / zb_des[2]);
    Eigen::Vector3d eulerAngle(phi_des, theta_des, yaw);
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond quaternion;
    quaternion=yawAngle*pitchAngle*rollAngle;
    return (Eigen::Vector4d ()<<quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()).finished();
}

void PointMassPathSearching::getOptimalTrajectory(std::vector<std::vector<Eigen::Vector3d>> *Position, std::vector<std::vector<Eigen::Vector3d>> *Velocity,
                                                  std::vector<std::vector<Eigen::Vector4d>> *Attitude, std::vector<std::vector<Eigen::Vector3d>> *Omega,
                                                  std::vector<std::vector<double>> *Thrust) const {
    for(int i = 0; i < optimal_position_.size(); i++){
        Position->push_back(optimal_position_.at(i));
        Velocity->push_back(optimal_velocity_.at(i));
        Attitude->push_back(optimal_attitude_.at(i));
        Omega->push_back(optimal_omega_.at(i));
        Thrust->push_back(optimal_thrust_.at(i));
    }
}

void PointMassPathSearching::getOptimalTrajectory(Trajectories::Trajectories* traj) const {

    traj->set_full_state_trajectories(optimal_position_, optimal_velocity_, optimal_attitude_, optimal_omega_, optimal_thrust_);

}

void PointMassPathSearching::getPointMassTrajectory(Trajectories::Trajectories *traj) const {
    traj->set_pointmass_trajectories(pointmass_position_, pointmass_velocity_, pointmass_acc_);

}
//int main() {
//    double max_t = 30;
//    PointMassPathSearching cpc(0.5 * max_t, 0.5 * max_t, max_t, -0.5 * max_t, -0.5 * max_t, -max_t);
//    std::vector<waypoint> waypoints;
//    std::vector<Eigen::Vector3d> Trajectory;
//    std::vector<Eigen::Vector3d> Velocity;
//    std::vector<Eigen::Vector3d> acc;
////    waypoints.emplace_back(0, 0, 1);
////    waypoints.emplace_back(4, 4, 1);
////    waypoints.emplace_back(0, 8, 1);
////    waypoints.emplace_back(4, 12, 1);
//    waypoints.emplace_back(Eigen::Vector3d (15, 12, 2), 0);
//    waypoints.emplace_back(Eigen::Vector3d (15, 25, 2), 1);
//    waypoints.emplace_back(Eigen::Vector3d (35, 15, 3), 2);
//    waypoints.emplace_back(Eigen::Vector3d (30, 35, 5), 3);
//    waypoints.emplace_back(Eigen::Vector3d (24, 24, 2), 4);
//    waypoints.emplace_back(Eigen::Vector3d (28, 28, 6), 5);
//    waypoints.emplace_back(Eigen::Vector3d (15, 24, 4), 6);
//    waypoints.emplace_back(Eigen::Vector3d (15, 12, 2), 7);
//    auto start = std::chrono::system_clock::now();
//    cpc.solve(waypoints, Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 0));
//    auto end_compute = std::chrono::system_clock::now();
//    cpc.drawTrajectory(0.03, &Trajectory, &Velocity, &acc);
//    auto end_total = std::chrono::system_clock::now();
//    auto duration_com = std::chrono::duration_cast<std::chrono::microseconds>(end_compute - start);
//    auto duration_total = std::chrono::duration_cast<std::chrono::microseconds>(end_total - start);
//    std::cout <<  "计算花费了"
//         << double(duration_com.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den
//         << "秒， 总共花费了" << double(duration_total.count()) * std::chrono::microseconds::period::num / std::chrono::microseconds::period::den
//                 << "秒" << std::endl;
//
//    cpc.saveTrajectory2file();
//    for(auto item : cpc.getOptimalVelocity()){
//        std::cout<<item(0)<<" "<<item(1)<<" "<<item(2)<<std::endl;
//    }
//    std::cout << "Pos=[";
//    for (auto i: Trajectory) {
//        std::cout << "[" << i(0) << "," << i(1) << "," << i(2) << "]" << "," << std::endl;
//    }
//    std::cout << "]" << std::endl;
//    std::cout << "Vel=[";
//    for (auto i: Velocity) {
//        std::cout << "[" << i(0) << "," << i(1) << "," << i(2) << "]" << "," << std::endl;
//    }
//    std::cout << "]" << std::endl;
//    std::cout << "acc=[";
//    for (auto i: acc) {
//        std::cout << "[" << i(0) << "," << i(1) << "," << i(2) << "]" << "," << std::endl;
//    }
//    std::cout << "]" << std::endl;
//}
