#ifndef LOCALPLANNER_HPP
#define LOCALPLANNER_HPP

#include <QObject>
#include <QAtomicInt>
#include <QMutex>
#include <QThread>

#include <eigen3/Eigen/Dense>
#include <vector>

#include"qnode.hpp"
#include "ctrl_context.h"

#include "object.hpp"
#include "target.hpp"


//#include "ui_main_window.h"

namespace motion_manager {
class QNode; //foward declaration - improve compilation time, reduce coupling, and break circular dependencies
typedef boost::shared_ptr<Object> objectPtr;
typedef boost::shared_ptr<Target> targetPtr;
}

namespace Ui {
class MainWindowDesign;
}


class LocalPlanner : public QObject
{
    Q_OBJECT
public:
    explicit LocalPlanner(motion_manager::QNode& node, Ui::MainWindowDesign& ui_ref, HUMotion::planning_result_ptr& h_results_ref,
                          motion_manager::CtrlContext& ctrl_context, motion_manager::LocalPlannerFilters& filters,
                          HUMotion::planning_dual_result_ptr& h_dual_results_ref, vector<motion_manager::TaskPlannedMovData>& planned_task_movs_ref,
                          QObject *parent = 0);
    ~LocalPlanner();

    //void init(QNode* qnode, Ui::MainWindowDesign* ui, vector<double>* sim_time ,scenarioPtr scene, LocalPlannerFilters* lpf);

    void exec_pos_ctrl();
    void exec_vel_ctrl();
private:
    /******************************************
    *           Function Definition           *
    *******************************************/
    void task_pre_processing();
    void stage_ctrl_logic();
    void use_plan_hand_pos();
    void obs_avoidance();
    void human_likeness_pre_process();
    void human_like_exec();
    void exec_trapezoidal_vel_profile();
    void read_usr_interface();
    void mes_pose();

    //gets
    void get_des_hand_vel();
    void get_swivel_angle();
    void get_joint_velocities();
    void getAllVel();
    void get_joint_accel();
    void get_hand_accel();
    void get_wrist_accel();
    void get_elbow_accel();
    void get_shoulder_accel();
    void getAllAcc();

    void calc_error();
    void Recording();

    //aux
    double getNoiseRobustDerivate(int N, double h, std::deque<double>& buff);
    int binomialCoeff(int n, int k);

    /******************************************
    *           Structs Definition            *
    *******************************************/

    struct DesPoseStrc {
      // ==========================
      // === Single Arm Variables ===
      // ==========================
      double des_hand_pos_x = 0.0, des_hand_pos_y = 0.0, des_hand_pos_z = 0.0;
      double des_hand_or_x  = 0.0, des_hand_or_y = 0.0, des_hand_or_z = 0.0;
      double des_hand_or_q_x = 0.0, des_hand_or_q_y = 0.0, des_hand_or_q_z = 0.0, des_hand_or_q_w = 0.0;

      std::vector<double> hand_pos_vec_xd = std::vector<double>(7, 0.0);

      double des_alpha_pos = 0.0, des_alpha_vel = 0.0, des_alpha_acc = 0.0; // desired swivel angle refs

      Eigen::VectorXd trap_hand_pose = Eigen::VectorXd::Zero(7);
      Eigen::VectorXd trap_hand_vel  = Eigen::VectorXd::Zero(7);
      Eigen::VectorXd trap_hand_acc  = Eigen::VectorXd::Zero(7);

      Eigen::VectorXd h_hand_pose = Eigen::VectorXd::Zero(7);
      Eigen::VectorXd h_hand_vel  = Eigen::VectorXd::Zero(6);
      Eigen::VectorXd h_hand_acc  = Eigen::VectorXd::Zero(6);

      Eigen::VectorXd hand_acc_xd_vec = Eigen::VectorXd::Zero(6);

      Eigen::VectorXd h_fing_pos = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND);
      Eigen::VectorXd h_fing_vel = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND);
      Eigen::VectorXd h_fing_acc = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND);

      double hand_or_q_w_init_vec = 0.0;
      Eigen::Vector3d hand_pos_init_vec = Eigen::Vector3d::Zero();
      Eigen::Vector3d hand_or_init_vec = Eigen::Vector3d::Zero();
      Eigen::Vector3d hand_or_q_e_init_vec = Eigen::Vector3d::Zero();

      double hand_or_q_w_final_vec = 0.0;
      Eigen::Vector3d hand_pos_final_vec = Eigen::Vector3d::Zero();
      Eigen::Vector3d hand_or_final_vec = Eigen::Vector3d::Zero();
      Eigen::Vector3d hand_or_q_e_final_vec = Eigen::Vector3d::Zero();

      std::vector<double> hand_acc_vec;
      Eigen::VectorXd hand_vel_vec_x = Eigen::VectorXd::Zero(7);
      Eigen::VectorXd hand_pos_vec_x = Eigen::VectorXd::Zero(7);
      Eigen::Vector3d des_hand_pos = Eigen::Vector3d::Zero();

      Eigen::VectorXd error_f_tot;
      Eigen::VectorXd error_f_fing_tot;


      // ==========================
      // === Dual Arm Variables ===
      // ==========================
      double des_hand_pos_x_left = 0.0, des_hand_pos_y_left = 0.0, des_hand_pos_z_left = 0.0;
      double des_hand_or_x_left  = 0.0, des_hand_or_y_left = 0.0, des_hand_or_z_left = 0.0;
      double des_hand_or_q_x_left = 0.0, des_hand_or_q_y_left = 0.0, des_hand_or_q_z_left = 0.0, des_hand_or_q_w_left = 0.0;

      std::vector<double> hand_pos_vec_xd_left = std::vector<double>(7, 0.0);

      double des_alpha_pos_left = 0.0, des_alpha_vel_left = 0.0, des_alpha_acc_left = 0.0; // desired swivel angle refs for left arm

      Eigen::VectorXd trap_hand_pose_left = Eigen::VectorXd::Zero(7);
      Eigen::VectorXd trap_hand_vel_left  = Eigen::VectorXd::Zero(7);
      Eigen::VectorXd trap_hand_acc_left  = Eigen::VectorXd::Zero(7);

      Eigen::VectorXd h_hand_pose_left = Eigen::VectorXd::Zero(7);
      Eigen::VectorXd h_hand_vel_left  = Eigen::VectorXd::Zero(6);
      Eigen::VectorXd h_hand_acc_left  = Eigen::VectorXd::Zero(6);

      Eigen::VectorXd hand_acc_xd_vec_left = Eigen::VectorXd::Zero(6);

      Eigen::VectorXd h_fing_pos_left = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND);
      Eigen::VectorXd h_fing_vel_left = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND);
      Eigen::VectorXd h_fing_acc_left = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND);

      double hand_or_q_w_init_vec_left = 0.0;
      Eigen::Vector3d hand_pos_init_vec_left = Eigen::Vector3d::Zero();
      Eigen::Vector3d hand_or_init_vec_left = Eigen::Vector3d::Zero();
      Eigen::Vector3d hand_or_q_e_init_vec_left = Eigen::Vector3d::Zero();

      double hand_or_q_w_final_vec_left = 0.0;
      Eigen::Vector3d hand_pos_final_vec_left = Eigen::Vector3d::Zero();
      Eigen::Vector3d hand_or_final_vec_left = Eigen::Vector3d::Zero();
      Eigen::Vector3d hand_or_q_e_final_vec_left = Eigen::Vector3d::Zero();

      std::vector<double> hand_acc_vec_left;
      Eigen::VectorXd hand_vel_vec_x_left = Eigen::VectorXd::Zero(7);
      Eigen::VectorXd hand_pos_vec_x_left = Eigen::VectorXd::Zero(7);
      Eigen::Vector3d des_hand_pos_left = Eigen::Vector3d::Zero();

      Eigen::VectorXd error_f_tot_left;
      Eigen::VectorXd error_f_fing_tot_left;

    };

    //medições efetuadas no robot
   struct MesPoseStrc {
      // posture
      std::vector<double> r_arm_posture_mes = std::vector<double>(motion_manager::JOINTS_ARM, 0.0),         l_arm_posture_mes = std::vector<double>(motion_manager::JOINTS_ARM, 0.0);
      std::vector<double> r_hand_posture_mes = std::vector<double>(motion_manager::JOINTS_HAND, 0.0),       l_hand_posture_mes = std::vector<double>(motion_manager::JOINTS_HAND, 0.0);
      std::vector<double> r_arm_posture = std::vector<double>(motion_manager::JOINTS_ARM, 0.0),             l_arm_posture = std::vector<double>(motion_manager::JOINTS_ARM, 0.0);
      std::vector<double> r_hand_posture = std::vector<double>(motion_manager::JOINTS_HAND, 0.0),           l_hand_posture = std::vector<double>(motion_manager::JOINTS_HAND, 0.0);
      // velocities
      Eigen::VectorXd r_arm_null_velocities = Eigen::VectorXd::Zero(motion_manager::JOINTS_ARM);
      std::vector<double> r_arm_velocities = std::vector<double>(motion_manager::JOINTS_ARM, 0.0),          l_arm_velocities = std::vector<double>(motion_manager::JOINTS_ARM, 0.0);
      std::vector<double> r_hand_velocities = std::vector<double>(motion_manager::JOINTS_HAND, 0.0),        l_hand_velocities = std::vector<double>(motion_manager::JOINTS_HAND, 0.0);
      std::vector<double> r_arm_velocities_read = std::vector<double>(motion_manager::JOINTS_ARM, 0.0),     l_arm_velocities_read = std::vector<double>(motion_manager::JOINTS_ARM, 0.0);
      std::vector<double> r_hand_velocities_read = std::vector<double>(motion_manager::JOINTS_HAND, 0.0),   l_hand_velocities_read = std::vector<double>(motion_manager::JOINTS_HAND, 0.0);
      // accelerations
      std::vector<double> r_arm_accelerations_read = std::vector<double>(motion_manager::JOINTS_ARM, 0.0),  l_arm_accelerations_read = std::vector<double>(motion_manager::JOINTS_ARM, 0.0);
      std::vector<double> r_hand_accelerations_read = std::vector<double>(motion_manager::JOINTS_HAND, 0.0),l_hand_accelerations_read = std::vector<double>(motion_manager::JOINTS_HAND, 0.0);
      std::vector<double> r_hand_acc_read = std::vector<double>(6, 0.0),      l_hand_acc_read = std::vector<double>(6, 0.0);
      std::vector<double> r_wrist_acc_read = std::vector<double>(6, 0.0),     l_wrist_acc_read = std::vector<double>(6, 0.0);
      std::vector<double> r_elbow_acc_read = std::vector<double>(6, 0.0),     l_elbow_acc_read = std::vector<double>(6, 0.0);
      std::vector<double> r_shoulder_acc_read = std::vector<double>(6, 0.0),  l_shoulder_acc_read = std::vector<double>(6, 0.0);
      // swivel angle <- Porquê vetor? - pode fazer sentido juntar left com right
      std::vector<double> alpha_pos_read = std::vector<double>(1, 0.0),       alpha_pos_read_left = std::vector<double>(1, 0.0);
      std::vector<double> alpha_vel_read = std::vector<double>(1, 0.0),       alpha_vel_read_left = std::vector<double>(1, 0.0);
      std::vector<double> alpha_acc_read = std::vector<double>(1, 0.0),       alpha_acc_read_left = std::vector<double>(1, 0.0);


      Eigen::VectorXd l_arm_null_velocities = Eigen::VectorXd::Zero(motion_manager::JOINTS_ARM);


      //este codigo poderá mudar de sitio !!!!
      Eigen::Vector3d r_hand_pos_vec, l_hand_pos_vec;

      //Eigen::Vector3d des_hand_pos;
      Eigen::Vector3d error_pos, error_pos_left;
      Eigen::Vector3d error_or, error_or_left;
      double e_n_pos;
      double e_n_or;

      double r_hand_q_w,                l_hand_q_w;
      Eigen::Vector3d r_hand_or_q_e,    l_hand_or_q_e;

      double          des_hand_or_q_w,  des_hand_or_q_w_left;
      Eigen::Vector3d des_hand_or_q_e,  des_hand_or_q_e_left;
      Eigen::Vector3d des_hand_or,      des_hand_or_left;
      Eigen::VectorXd des_hand_or_q      = Eigen::VectorXd::Zero(4);
      Eigen::VectorXd des_hand_or_q_left = Eigen::VectorXd::Zero(4);

      vector<double>  r_hand_pos,          l_hand_pos;
      vector<double>  r_hand_lin_pos,      l_hand_lin_pos;
      vector<double>  r_hand_ang_pos,      l_hand_ang_pos;
      vector<double>  r_hand_q,            l_hand_q;
      vector<double>  r_hand_pos_q,        l_hand_pos_q;
      vector<double>  r_hand_lin_vel,      l_hand_lin_vel;
      Eigen::Vector3d r_hand_lin_vel_vec,  l_hand_lin_vel_vec;
      vector<double>  r_hand_ang_vel,      l_hand_ang_vel;
      Eigen::Vector3d r_hand_ang_vel_vec,  l_hand_ang_vel_vec;
      vector<double>  r_hand_lin_acc,      l_hand_lin_acc;
      vector<double>  r_hand_ang_acc,      l_hand_ang_acc;

      vector<double> r_wrist_pos,         l_wrist_pos;
      vector<double> r_wrist_lin_pos,     l_wrist_lin_pos;
      vector<double> r_wrist_ang_pos,     l_wrist_ang_pos;
      vector<double> r_wrist_pos_q,       l_wrist_pos_q;
      vector<double> r_wrist_lin_vel,     l_wrist_lin_vel;
      vector<double> r_wrist_ang_vel,     l_wrist_ang_vel;
      vector<double> r_wrist_lin_acc,     l_wrist_lin_acc;
      vector<double> r_wrist_ang_acc,     l_wrist_ang_acc;

      vector<double> r_elbow_pos,         l_elbow_pos;
      vector<double> r_elbow_lin_pos,     l_elbow_lin_pos;
      vector<double> r_elbow_ang_pos,     l_elbow_ang_pos;
      vector<double> r_elbow_pos_q,       l_elbow_pos_q;
      vector<double> r_elbow_lin_vel,     l_elbow_lin_vel;
      vector<double> r_elbow_ang_vel,     l_elbow_ang_vel;
      vector<double> r_elbow_lin_acc,     l_elbow_lin_acc;
      vector<double> r_elbow_ang_acc,     l_elbow_ang_acc;

      vector<double> r_shoulder_pos,       l_shoulder_pos;
      vector<double> r_shoulder_lin_pos,   l_shoulder_lin_pos;
      vector<double> r_shoulder_pos_q,     l_shoulder_pos_q;
      vector<double> r_shoulder_ang_pos,   l_shoulder_ang_pos;
      vector<double> r_shoulder_lin_vel,   l_shoulder_lin_vel;
      vector<double> r_shoulder_ang_vel,   l_shoulder_ang_vel;
      vector<double> r_shoulder_lin_acc,   l_shoulder_lin_acc;
      vector<double> r_shoulder_ang_acc,   l_shoulder_ang_acc;

      Eigen::VectorXd r_hand_acc_read_vec, l_hand_acc_read_vec;
      Eigen::VectorXd r_hand_accelerations_read_vec, l_hand_accelerations_read_vec;

      Eigen::Vector3d r_hand_ang_vel_q_e, l_hand_ang_vel_q_e;
      double r_hand_ang_vel_q_w, l_hand_ang_vel_q_w;

      motion_manager::targetPtr tar_rec;
      Eigen::Vector3d tar_rec_pos;
      Eigen::Vector3d tar_rec_pos_left;
      vector<double> r_hand_vel,     l_hand_vel;
      vector<double> r_wrist_vel,    l_wrist_vel;
      vector<double> r_elbow_vel,    l_elbow_vel;
      vector<double> r_shoulder_vel, l_shoulder_vel;

    };

   //provavelmente pra outra struct !!!
   struct DesVelStrc {
     /*
     vector<double> hand_acc_vec;
     Eigen::VectorXd hand_vel_vec_x = Eigen::VectorXd::Zero(7);*/
   };

   //bounce posture para obstacle avoidace - verificado
   struct BouncePoseStrc {
     double bounce_hand_pos_x = 0.0; double bounce_hand_pos_y = 0.0; double bounce_hand_pos_z = 0.0;
     double bounce_hand_or_x = 0.0; double bounce_hand_or_y = 0.0; double bounce_hand_or_z = 0.0;
     double bounce_hand_q_x = 0.0; double bounce_hand_q_y = 0.0; double bounce_hand_q_z = 0.0; double bounce_hand_q_w = 0.0;

     double bounce_hand_pos_x_left = 0.0; double bounce_hand_pos_y_left = 0.0; double bounce_hand_pos_z_left = 0.0;
     double bounce_hand_or_x_left = 0.0; double bounce_hand_or_y_left = 0.0; double bounce_hand_or_z_left = 0.0;
     double bounce_hand_q_x_left = 0.0; double bounce_hand_q_y_left = 0.0; double bounce_hand_q_z_left = 0.0; double bounce_hand_q_w_left = 0.0;
   };


   //inserção de ruido - verificado
   struct NoiseSimObjStrc {
     double obj_x_var = 100; // mm
     double obj_y_var = 100; // mm
     double obj_z_var = 100; // mm
     double obj_q_x_var = 0.1;
     double obj_q_y_var = 0.1;
     double obj_q_z_var = 0.1;
     double obj_q_w_var = 0.1;
   };

   //coeficientes necessarios para human-likeness - verificado
   struct HumanLikenessCoefStrc {
     // Plan
     double hl_p_x_pos_coeff_plan = 1; double hl_p_y_pos_coeff_plan = 1; double hl_p_z_pos_coeff_plan = 1;
     double hl_p_x_or_coeff_plan = 1; double hl_p_y_or_coeff_plan = 1; double hl_p_z_or_coeff_plan = 1;
     double hl_d_x_pos_coeff_plan = 1; double hl_d_y_pos_coeff_plan = 1; double hl_d_z_pos_coeff_plan = 1;
     double hl_d_x_or_coeff_plan = 1; double hl_d_y_or_coeff_plan = 1; double hl_d_z_or_coeff_plan = 1;
     // Approach
     double hl_p_x_pos_coeff_app = 1; double hl_p_y_pos_coeff_app = 1; double hl_p_z_pos_coeff_app = 1;
     double hl_p_x_or_coeff_app = 1; double hl_p_y_or_coeff_app = 1; double hl_p_z_or_coeff_app = 1;
     double hl_d_x_pos_coeff_app = 1; double hl_d_y_pos_coeff_app = 1; double hl_d_z_pos_coeff_app = 1;
     double hl_d_x_or_coeff_app = 1; double hl_d_y_or_coeff_app = 1; double hl_d_z_or_coeff_app = 1;
     // Retreat
     double hl_p_x_pos_coeff_ret = 1; double hl_p_y_pos_coeff_ret = 1; double hl_p_z_pos_coeff_ret = 1;
     double hl_p_x_or_coeff_ret = 1; double hl_p_y_or_coeff_ret = 1; double hl_p_z_or_coeff_ret = 1;
     double hl_d_x_pos_coeff_ret = 1; double hl_d_y_pos_coeff_ret = 1; double hl_d_z_pos_coeff_ret = 1;
     double hl_d_x_or_coeff_ret = 1; double hl_d_y_or_coeff_ret = 1; double hl_d_z_or_coeff_ret = 1;
     //Swivel angle
     double hl_alpha_pos_coeff = 1.0; double hl_alpha_vel_coeff = 0.1;
     // Finger
     double fing_p_coeff = 0.1; double fing_d_coeff = 0.1;

     // Stage related coefficients
     double hl_p_x_pos_coeff = 1; double hl_p_y_pos_coeff = 1; double hl_p_z_pos_coeff = 1;
     double hl_p_x_or_coeff = 1; double hl_p_y_or_coeff = 1; double hl_p_z_or_coeff = 1;
     double hl_d_x_pos_coeff = 1; double hl_d_y_pos_coeff = 1; double hl_d_z_pos_coeff = 1;
     double hl_d_x_or_coeff = 1; double hl_d_y_or_coeff = 1; double hl_d_z_or_coeff = 1;


     double error_pos_th;
     double error_or_th;
     double coeff_p_pos;
     double coeff_p_or;
     double coeff_d_pos;
     double coeff_d_or;

     Eigen::MatrixXd Koeff_d = Eigen::MatrixXd::Identity(6,6);
     Eigen::MatrixXd Koeff_p = Eigen::MatrixXd::Identity(6,6);

   };

   //coeficientes necessários para obstacle avoidance -verificado
   struct AvoidanceVarStrc {
     double jlim_th = 0; double jlim_rate = 1;
     double jlim_coeff = 1; double jlim_damping = 0.001;
     double obst_coeff = 1; double obst_damping = 0.001;
     double obst_coeff_torso = 1; double obst_damping_torso = 0.001;
     double sing_coeff = 1; double sing_damping = 0.001;

     std::vector<motion_manager::objectPtr> obsts_n;

   };



   struct JointStateStrc {
     Eigen::VectorXd jointsInitPosition_hand = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND);
     std::vector<double> jointsInitPosition_hand_vec = std::vector<double>(motion_manager::JOINTS_HAND, 0.0);
     Eigen::VectorXd jointsFinalPosition_hand = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND);
     std::vector<double> jointsFinalPosition_hand_vec = std::vector<double>(motion_manager::JOINTS_HAND, 0.0);
     Eigen::VectorXd jointsInitVelocity_hand = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND);
     std::vector<double> jointsInitVelocity_hand_vec = std::vector<double>(motion_manager::JOINTS_HAND, 0.0);
     Eigen::VectorXd jointsFinalVelocity_hand = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND);
     std::vector<double> jointsFinalVelocity_hand_vec = std::vector<double>(motion_manager::JOINTS_HAND, 0.0);
     Eigen::VectorXd jointsInitAcceleration_hand = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND);
     std::vector<double> jointsInitAcceleration_hand_vec = std::vector<double>(motion_manager::JOINTS_HAND, 0.0);
     Eigen::VectorXd jointsFinalAcceleration_hand = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND);
     std::vector<double> jointsFinalAcceleration_hand_vec = std::vector<double>(motion_manager::JOINTS_HAND, 0.0);
     Eigen::VectorXd jointsBouncePosition_hand = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND);
     std::vector<double> jointsBouncePosition_hand_vec = std::vector<double>(motion_manager::JOINTS_HAND, 0.0);

     Matrix3d I_3 = Matrix3d::Identity();
     int n_steps; double period_T;

     std::vector<std::vector<double>> hand_h_positions, hand_h_positions_left;
     std::vector<std::vector<double>> hand_h_orientations, hand_h_orientations_left;
     std::vector<std::vector<double>> hand_h_orientations_q, hand_h_orientations_q_left;
     std::vector<std::vector<double>> hand_h_lin_velocities, hand_h_lin_velocities_left;
     std::vector<std::vector<double>> hand_h_ang_velocities, hand_h_ang_velocities_left;
     std::vector<std::vector<double>> hand_h_lin_accelerations, hand_h_lin_accelerations_left;
     std::vector<std::vector<double>> hand_h_ang_accelerations, hand_h_ang_accelerations_left;

     std::vector<double> alpha_positions, alpha_positions_left;
     std::vector<double> alpha_velocities, alpha_velocities_left;
     std::vector<double> alpha_accelerations, alpha_accelerations_left;


     Eigen::Vector3d h_hand_ang_vel_q_e_init; double h_hand_ang_vel_q_w_init; Eigen::Vector3d h_hand_ang_vel_q_e_end; double h_hand_ang_vel_q_w_end;
     Eigen::Vector3d h_hand_ang_acc_q_e_init; double h_hand_ang_acc_q_w_init; Eigen::Vector3d h_hand_ang_acc_q_e_end; double h_hand_ang_acc_q_w_end;

     //mudei para cpp, ver se ajuda
     //Eigen::Vector3d h_hand_ang_vel_init_vec; Eigen::Vector3d h_hand_ang_vel_end_vec;
     //Eigen::Vector3d h_hand_ang_acc_init_vec; Eigen::Vector3d h_hand_ang_acc_end_vec;
     double h_hand_or_q_w_init; Eigen::Vector3d h_hand_or_init_vec; Eigen::Vector3d h_hand_or_q_e_init;
     double alpha_acc_xd = 0.0, alpha_acc_xd_left = 0.0;

     double h_hand_or_q_w_end; Eigen::Vector3d h_hand_or_q_e_end;
     double h_hand_or_q_w_end_left; Eigen::Vector3d h_hand_or_q_e_end_left;

     //em quarentena
     std::vector<double> h_hand_pos_init; /**< initial hand position during control*/
     std::vector<double> h_hand_or_init; /**< initial hand orientation (rpy) during control*/
     std::vector<double> h_hand_or_q_init; /**< initial hand orientation (quaternion) during control*/
     vector<double> h_hand_pos_end; /**< end hand position during control */
     vector<double> h_hand_or_end; /**< end hand orientation (rpy) during control */
     vector<double> h_hand_or_q_end; /**< end hand orientation (quaternion) during control */
     vector<double> h_hand_lin_vel_init; /**< initial hand linear velocity during control */
     vector<double> h_hand_ang_vel_init; /**< initial hand angular velocity during control */
     vector<double> h_hand_lin_vel_end; /**< end hand linear velocity during control */
     vector<double> h_hand_ang_vel_end; /**< end hand angular velocity during control */
     vector<double> h_hand_lin_acc_end; /**< end hand linear acceleration during control */
     vector<double> h_hand_ang_acc_end; /**< end hand angular acceleration during control */
     vector<double> h_hand_lin_acc_init; /**< initial hand linear acceleration during control */
     vector<double> h_hand_ang_acc_init; /**< initial hand angular acceleration during control */

     //dual
     std::vector<double> h_hand_pos_init_left; /**< initial hand position during control*/
     std::vector<double> h_hand_or_init_left; /**< initial hand orientation (rpy) during control*/
     std::vector<double> h_hand_or_q_init_left; /**< initial hand orientation (quaternion) during control*/
     vector<double> h_hand_pos_end_left; /**< end hand position during control */
     vector<double> h_hand_or_end_left; /**< end hand orientation (rpy) during control */
     vector<double> h_hand_or_q_end_left; /**< end hand orientation (quaternion) during control */
     vector<double> h_hand_lin_vel_init_left; /**< initial hand linear velocity during control */
     vector<double> h_hand_ang_vel_init_left; /**< initial hand angular velocity during control */
     vector<double> h_hand_lin_vel_end_left; /**< end hand linear velocity during control */
     vector<double> h_hand_ang_vel_end_left; /**< end hand angular velocity during control */
     vector<double> h_hand_lin_acc_end_left; /**< end hand linear acceleration during control */
     vector<double> h_hand_ang_acc_end_left; /**< end hand angular acceleration during control */
     vector<double> h_hand_lin_acc_init_left; /**< initial hand linear acceleration during control */
     vector<double> h_hand_ang_acc_init_left; /**< initial hand angular acceleration during control */

     double h_hand_or_q_w_init_left; Eigen::Vector3d h_hand_or_init_vec_left; Eigen::Vector3d h_hand_or_q_e_init_left;
     Eigen::Vector3d h_hand_ang_vel_q_e_init_left; double h_hand_ang_vel_q_w_init_left; Eigen::Vector3d h_hand_ang_vel_q_e_end_left; double h_hand_ang_vel_q_w_end_left;
     Eigen::Vector3d h_hand_ang_acc_q_e_init_left; double h_hand_ang_acc_q_w_init_left; Eigen::Vector3d h_hand_ang_acc_q_e_end_left; double h_hand_ang_acc_q_w_end_left;

     Eigen::VectorXd jointsInitPosition_hand_left = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND);
     std::vector<double> jointsInitPosition_hand_vec_left = std::vector<double>(motion_manager::JOINTS_HAND, 0.0);
     Eigen::VectorXd jointsFinalPosition_hand_left = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND);
     std::vector<double> jointsFinalPosition_hand_vec_left = std::vector<double>(motion_manager::JOINTS_HAND, 0.0);
     Eigen::VectorXd jointsInitVelocity_hand_left = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND);
     std::vector<double> jointsInitVelocity_hand_vec_left = std::vector<double>(motion_manager::JOINTS_HAND, 0.0);
     Eigen::VectorXd jointsFinalVelocity_hand_left = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND);
     std::vector<double> jointsFinalVelocity_hand_vec_left = std::vector<double>(motion_manager::JOINTS_HAND, 0.0);
     Eigen::VectorXd jointsInitAcceleration_hand_left = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND);
     std::vector<double> jointsInitAcceleration_hand_vec_left = std::vector<double>(motion_manager::JOINTS_HAND, 0.0);
     Eigen::VectorXd jointsFinalAcceleration_hand_left = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND);
     std::vector<double> jointsFinalAcceleration_hand_vec_left = std::vector<double>(motion_manager::JOINTS_HAND, 0.0);
     Eigen::VectorXd jointsBouncePosition_hand_left = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND);
     std::vector<double> jointsBouncePosition_hand_vec_left = std::vector<double>(motion_manager::JOINTS_HAND, 0.0);

   };


   //le os inputs do utilizador -verificado
   struct UsrInterfaceInputStrc {
     bool sim_robot = false;
     bool plan_hand_pos = false;
     double vel_max = false;
     bool joints_arm_vel_ctrl = false;
     double swivel_angle_th = false;//nao usado
     double g_map_th_max_replan = false;//nao usado
     double g_map_th_min_replan = false;//nao usado
     double t_f_trap = false;
     bool jlim_en = false;
     bool sing_en = false;
     bool obsts_en = false;
     bool hl_en = false;
     bool hl_alpha_en = false;
     bool follow_tar = false;
    // double e_n_pos; double e_n_or;
   };

   //variaveis relativas ao tempo de execução -verificado
   struct TimeVarStrc {
     double g_map_th_pa = 0.99; double g_map_th_rp = 0.99;
     double phi = 0.0; double tb = 0.0;
     double time_step; // time step of the controlling
     double g_map = 0.0; int index = 0;// normalized mapped time
   };


   //variaiveis relativas ao erro - verificado
   struct ErrorVarStrc {
     Eigen::VectorXd error_trap_tot = Eigen::VectorXd::Zero(6); // error with the trapezoidal pose
     Eigen::VectorXd der_error_trap_tot = Eigen::VectorXd::Zero(6); // time derivative of the error with the trapezoidal pose
     Eigen::VectorXd error_h_tot = Eigen::VectorXd::Zero(6); // error with the human-like pose
     Eigen::VectorXd der_error_h_tot = Eigen::VectorXd::Zero(6); // time derivative of the error with the human-like pose

     Eigen::VectorXd error_h_fing_tot = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND); // error with the human-like pose
     Eigen::VectorXd der_error_h_fing_tot = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND); // time derivative of the error with the human-like pose
     Eigen::VectorXd h_fing_ref_acc = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND); // human-like reference acceleration

     Eigen::VectorXd trap_hand_ref_vel = Eigen::VectorXd::Zero(6); // trapezoidal reference velocity
     Eigen::VectorXd trap_hand_ref_acc = Eigen::VectorXd::Zero(6); // trapezoidal reference acceleration
     Eigen::VectorXd h_hand_ref_acc = Eigen::VectorXd::Zero(6); // human-like reference acceleration

     double error_alpha_pos = 0.0; double error_alpha_vel = 0.0; double error_alpha_acc = 0.0; // error with the human-like swivel angle
     double des_alpha_pos, des_alpha_vel, des_alpha_acc; // desired swivel angle references
     double des_alpha_pos_left, des_alpha_vel_left, des_alpha_acc_left; // desired swivel angle references


     Eigen::VectorXd error_tot = Eigen::VectorXd::Zero(6);
     Eigen::VectorXd error_f_fing_tot;
     Eigen::Vector3d error_f_pos;
     Eigen::VectorXd error_f_orr = Eigen::VectorXd::Zero(4);
     Eigen::VectorXd error_f_tot = Eigen::VectorXd::Zero(7);


     double error_alpha_pos_left = 0.0; double error_alpha_vel_left = 0.0; double error_alpha_acc_left = 0.0; // error with the human-like swivel angle
     Eigen::VectorXd error_h_tot_left = Eigen::VectorXd::Zero(6); // error with the human-like pose
     Eigen::VectorXd der_error_h_tot_left = Eigen::VectorXd::Zero(6); // time derivative of the error with the human-like pose
     Eigen::VectorXd error_h_fing_tot_left = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND); // error with the human-like pose
     Eigen::VectorXd der_error_h_fing_tot_left = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND); // time derivative of the error with the human-like pose
     Eigen::VectorXd h_hand_ref_acc_left = Eigen::VectorXd::Zero(6); // human-like reference acceleration
     Eigen::VectorXd h_fing_ref_acc_left = Eigen::VectorXd::Zero(motion_manager::JOINTS_HAND); // human-like reference acceleration

     Eigen::VectorXd error_trap_tot_left = Eigen::VectorXd::Zero(6); // error with the trapezoidal pose
     Eigen::VectorXd der_error_trap_tot_left = Eigen::VectorXd::Zero(6); // time derivative of the error with the trapezoidal pose

     Eigen::VectorXd trap_hand_ref_vel_left = Eigen::VectorXd::Zero(6); // trapezoidal reference velocity
     Eigen::VectorXd trap_hand_ref_acc_left = Eigen::VectorXd::Zero(6); // trapezoidal reference acceleration
     Eigen::VectorXd error_tot_left = Eigen::VectorXd::Zero(6);
     Eigen::VectorXd error_f_fing_tot_left;
     Eigen::Vector3d error_f_pos_left;
     Eigen::VectorXd error_f_orr_left = Eigen::VectorXd::Zero(4);
     Eigen::VectorXd error_f_tot_left = Eigen::VectorXd::Zero(7);
   };


   /******************************************
   *            VArType Definition           *
   *******************************************/

    //Member classes
    motion_manager::QNode& qnode;
    Ui::MainWindowDesign& ui;
    HUMotion::planning_result_ptr& h_results;
    motion_manager::CtrlContext& ctx;
    motion_manager::LocalPlannerFilters& lpf;
    HUMotion::planning_dual_result_ptr& h_dual_results;
    vector<motion_manager::TaskPlannedMovData>& all_planned_task_movements;

    //Member structs
    DesPoseStrc des_pose;
    MesPoseStrc mes_pose_str; //mudei o formato par anão entrar em conflito com a função anterior
    DesVelStrc des_vel;
    BouncePoseStrc bounce_pose;
    NoiseSimObjStrc noise_sim_obj;
    HumanLikenessCoefStrc human_likeness_coef;
    AvoidanceVarStrc avoidance_var;
    JointStateStrc JointState;
    UsrInterfaceInputStrc usr_interface_input;
    TimeVarStrc time_var;
    ErrorVarStrc error_var;


    //================= PARA ARRUMAR DEPOIS ===============
    //h_results esta declarado com this-> no entanto apesar de nao ser necessario não afeta o
    //funcionamento, uma vez que ja estaria implicito o apontador para este escopo.



    int mov_type = 0;
    std::string stage_descr = "plan";
    int stages = 1;
    int stages_left = 1;
    int mov_in_task_index = -1;
    Vector3d tar_pos; Vector3d tar_pos_left; Quaterniond tar_q; Quaterniond tar_q_left;
    int temp_time = 0;

    vector<string> trajectory_descriptions;
    int status;
    int h_r_flag = 0; //flag to reveal if it as used h_results ou h_dual_results (0-mov / 1-task single / 2-task dual)

    //=====================================================

};

#endif
