/*********************************************************************
 * purpose:
 * stereo extrinsics calib result convert to 
 * T_vehicle_left and T_vehicle_right
 * 
 * pipeline:
 * 
 * usage:
 * mkdir build
 * cd build
 * cmake ..
 * make -j
 * ./poseTrans sensor_name
 * 
 ********************************************************************/
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
using namespace std;

int main(int argc, char **argv) {
     // cam-ins
     //step1: R_i_c * R_c_l = Ｒ_i_l
     //Rl = R_c_l
     //handeye_calib => R = R_i_c
     Eigen::Matrix3d Rl = Eigen::Matrix3d::Identity();
     Rl << 0.99959609227669743, -0.026906788046576265, -0.0091475167216470664, 
           0.026968630928425615, 0.99961378775449794, 0.0067058390192404191,
           0.0089635512495080637, -0.0069498264814470282, 0.99993567526160143;
     Eigen::Quaterniond Rl_q = Eigen::Quaterniond(Rl);
     //w,x,y,z
     Eigen::Quaterniond R(0.511303, -0.490753, 0.491309,-0.506306);
     Eigen::Quaterniond q_i_l = R*Rl_q;
     cout << "q_i_l = " << endl 
          << "w:" << " " << q_i_l.w() << endl
          << "x:" << " " << q_i_l.x() << endl
          << "y:" << " " << q_i_l.y() << endl
          << "z:" << " " << q_i_l.z() << endl;
     //handeye_calib => t_i_c = t_i_l 
     Eigen::Vector3d t_i_l(2.32434, 0.19497, 1.29);

     Eigen::Isometry3d T_i_l = Eigen::Isometry3d::Identity();
     T_i_l.rotate(q_i_l);
     T_i_l.pretranslate(t_i_l);
     cout << "T_i_l =" << endl << T_i_l.matrix() << endl;

     //T_i_l * T_r_l.inverse() = T_i_r
     //stereo calib => R = R_r_l
     Eigen::Matrix3d R_r_l = Eigen::Matrix3d::Identity();
     R_r_l << 0.99980953496007352, -0.017039459402033344, -0.0095158092776154168, 
              0.017167563709708400, 0.99976084987487190, 0.013546874685579358,
              0.0092827021494065742, -0.013707657741574418, 0.99986295638954703;
     //stereo calib => t = t_r_l (!!mm->m)
     Eigen::Vector3d t_r_l(-0.086095217518766574, 0.00084925399175200467, -0.000043180469561086961);
     
     Eigen::Isometry3d T_r_l = Eigen::Isometry3d::Identity();
     T_r_l.rotate(R_r_l);
     T_r_l.pretranslate(t_r_l);
     cout <<  "T_r_l =" << endl << T_r_l.matrix() << endl;

     Eigen::Isometry3d T_i_r = T_i_l * T_r_l.inverse();
     cout <<  "T_i_r =" << endl << T_i_r.matrix() << endl;

     Eigen::Matrix3d R_i_r = T_i_r.rotation();
     Eigen::Quaterniond q_i_r(R_i_r);
     cout << "q_i_r = " << endl 
          << "w:" << " " << q_i_r.w() << endl
          << "x:" << " " << q_i_r.x() << endl
          << "y:" << " " << q_i_r.y() << endl
          << "z:" << " " << q_i_r.z() << endl;

     Eigen::Quaterniond q_v_i(0.99989663869656331,
                              -0.010300352342795438,
                              -0.0014058076374541717,
                              0.0099316851447846);
     Eigen::Vector3d t_v_i(-0.55, -0.07, 0.38);
     Eigen::Isometry3d T_v_i = Eigen::Isometry3d::Identity();
     T_v_i.rotate(q_v_i);
     T_v_i.pretranslate(t_v_i);
     std::cout << "T_v_i = " << std::endl << T_v_i.matrix() << std::endl;
     // T_v_i * T_i_l = T_v_l
     Eigen::Isometry3d T_v_l = T_v_i * T_i_l;
     std::cout << "T_v_l = " << std::endl << T_v_l.matrix() << std::endl;
     Eigen::Matrix3d R_v_l = T_v_l.rotation();
     Eigen::Quaterniond q_v_l(R_v_l);
     cout << "q_v_l = " << endl 
          << "w:" << " " << q_v_l.w() << endl
          << "x:" << " " << q_v_l.x() << endl
          << "y:" << " " << q_v_l.y() << endl
          << "z:" << " " << q_v_l.z() << endl;

     // T_v_i * T_i_r = T_v_r
     Eigen::Isometry3d T_v_r = T_v_i * T_i_r;
     std::cout << "T_v_r = " << std::endl << T_v_r.matrix() << std::endl;
     Eigen::Matrix3d R_v_r = T_v_r.rotation();
     Eigen::Quaterniond q_v_r(R_v_r);
     cout << "q_v_r = " << endl 
          << "w:" << " " << q_v_r.w() << endl
          << "x:" << " " << q_v_r.x() << endl
          << "y:" << " " << q_v_r.y() << endl
          << "z:" << " " << q_v_r.z() << endl;


     // // cam-lidar
     // //step1: R_lidar_c * R_c_l = Ｒ_lidar_l
     // //Rl = R_c_l
     // Eigen::Matrix3d Rl = Eigen::Matrix3d::Identity();
     
     // Rl << 0.99959609227669743, -0.026906788046576265, -0.0091475167216470664, 
     //       0.026968630928425615, 0.99961378775449794, 0.0067058390192404191,
     //       0.0089635512495080637, -0.0069498264814470282, 0.99993567526160143;
     // Eigen::Quaterniond Rl_q = Eigen::Quaterniond(Rl);

     // //w,x,y,z
     // Eigen::Quaterniond R(-0.443896, 0.436516, -0.538601,0.567731);
     // Eigen::Quaterniond q_L_l = R*Rl_q;
     // cout << "q_L_l = " << endl 
     //      << "w:" << " " << q_L_l.w() << endl
     //      << "x:" << " " << q_L_l.x() << endl
     //      << "y:" << " " << q_L_l.y() << endl
     //      << "z:" << " " << q_L_l.z() << endl;
     // //handeye_calib => t_L_c = t_L_l
     // Eigen::Vector3d t_L_l(0.557465, -0.00201945, 1.29);

     // Eigen::Isometry3d T_L_l = Eigen::Isometry3d::Identity();
     // T_L_l.rotate(q_L_l);
     // T_L_l.pretranslate(t_L_l);
     // cout << "T_L_l =" << endl << T_L_l.matrix() << endl;

     // //T_L_l * T_r_l.inverse() = T_L_r
     // //stereo calib => R = R_r_l
     // Eigen::Matrix3d R_r_l = Eigen::Matrix3d::Identity();
       
     // R_r_l << 0.99980953496007352, -0.017039459402033344, -0.0095158092776154168,
     //          0.017167563709708400, 0.99976084987487190, 0.013546874685579358,
     //          0.0092827021494065742, -0.013707657741574418, 0.99986295638954703;
     // //stereo calib => t = t_r_l (!!mm->m)
     // Eigen::Vector3d t_r_l(-0.086095217518766574, 0.00084925399175200467, -0.000043180469561086961);
     
     // Eigen::Isometry3d T_r_l = Eigen::Isometry3d::Identity();
     // T_r_l.rotate(R_r_l);
     // T_r_l.pretranslate(t_r_l);
     // cout <<  "T_r_l =" << endl << T_r_l.matrix() << endl;

     // Eigen::Isometry3d T_L_r = T_L_l * T_r_l.inverse();
     // cout <<  "T_L_r =" << endl << T_L_r.matrix() << endl;

     // // Eigen::Matrix3d R_L_r = T_L_r.rotation();
     // // Eigen::Quaterniond q_L_r(R_L_r);
     // // cout << "q_L_r = " << endl 
     // //      << "w:" << " " << q_L_r.w() << endl
     // //      << "x:" << " " << q_L_r.x() << endl
     // //      << "y:" << " " << q_L_r.y() << endl
     // //      << "z:" << " " << q_L_r.z() << endl;

     // Eigen::Quaterniond q_v_L(0.99294674549910367,
     //                          -0.0015684960312884379,
     //                          -0.00079914653426793386,
     //                          0.1185207993933217);
     // Eigen::Vector3d t_v_L(1.2855096661820256, -0.026190575660746417, 1.4798723041687132);
     // Eigen::Isometry3d T_v_L = Eigen::Isometry3d::Identity();
     // T_v_L.rotate(q_v_L);
     // T_v_L.pretranslate(t_v_L);
     // std::cout << "T_v_L = " << std::endl << T_v_L.matrix() << std::endl;
     // //T_v_L * T_L_l = T_v_l
     // Eigen::Isometry3d T_v_l = T_v_L * T_L_l;
     // std::cout << "T_v_l = " << std::endl << T_v_l.matrix() << std::endl;
     // Eigen::Matrix3d R_v_l = T_v_l.rotation();
     // Eigen::Quaterniond q_v_l(R_v_l);
     // cout << "q_v_l = " << endl 
     //      << "w:" << " " << q_v_l.w() << endl
     //      << "x:" << " " << q_v_l.x() << endl
     //      << "y:" << " " << q_v_l.y() << endl
     //      << "z:" << " " << q_v_l.z() << endl;
     // //T_v_L * T_L_r = T_v_r
     // Eigen::Isometry3d T_v_r = T_v_L * T_L_r;
     // std::cout << "T_v_r = " << std::endl << T_v_r.matrix() << std::endl;
     // Eigen::Matrix3d R_v_r = T_v_r.rotation();
     // Eigen::Quaterniond q_v_r(R_v_r);
     // cout << "q_v_r = " << endl 
     //      << "w:" << " " << q_v_r.w() << endl
     //      << "x:" << " " << q_v_r.x() << endl
     //      << "y:" << " " << q_v_r.y() << endl
     //      << "z:" << " " << q_v_r.z() << endl;

     return 0;
}