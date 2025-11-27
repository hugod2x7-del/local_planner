#include "../include/motion_manager/local_planner.hpp"
#include "../include/motion_manager/main_window.hpp"
#include <QThread>

#include <ros/ros.h>
#include <ros/network.h>


//#include <boost/variant.hpp>


/*
TO DO
ControlData
resolver questão de jointsBouncePosition_hand

por enquanto a mao direita é prefeitamente sincrona á mao esquerda, convem ser alterado no futuro


tar_rec e tar_rec_pos ver como funciona e se necessita de ser replicado no left
*/


using namespace motion_manager;

/**************************************************************
*                Initialization (Constructure)        				*
* This code should only run once in the beggining of the code *
* to avoid delays in the thread                               *
* It needs to be called before the other funcionts as it      *
* atriibuts values to nullptrs                                *
**************************************************************/
LocalPlanner::LocalPlanner(QNode& node, Ui::MainWindowDesign& ui_ref, HUMotion::planning_result_ptr& h_results_ref, CtrlContext& ctrl_context,
                           motion_manager::LocalPlannerFilters& filters,
                           HUMotion::planning_dual_result_ptr& h_dual_results_ref, vector<motion_manager::TaskPlannedMovData>& planned_task_movs_ref, QObject *parent)
    : QObject(parent), qnode(node), ui(ui_ref), h_results(h_results_ref), ctx(ctrl_context), lpf(filters), h_dual_results(h_dual_results_ref), all_planned_task_movements(planned_task_movs_ref)
{
}

LocalPlanner::~LocalPlanner() {}


//o obejtivo desta funçao é selecionar os dados a serem utilizados no controlo tendo em conta se é planeado ou task e se é uma ou duas maos
//temos que receber flags trazidas pelo main window
//idealmente atribuiria aqui os valores das variaveis que sao colocados em use_plan_hand e pre_proces_humann_likeness


/**********************************************
*               Exec Pos Ctrl             		*
* Executs the main floow of the position      *
* controller                                  *
*                                             *
**********************************************/
void LocalPlanner::exec_pos_ctrl() {

  while(ctx.exec_control)
  {
    if(ctx.pos_control){

    boost::unique_lock<boost::mutex> lck(ctx.hh_control_mtx);

    if(ui.radioButton_ctrl_task->isChecked() && mov_in_task_index == -1){
      // 1 - passar os valores das primeiras variaveis de task_strc para ctrl_ctx
      // 2 - perceber se já acabou o movimento
      // 3 - quando acabar o movimento repetir o passo 1 para o seguinte movimento
      // 4 - Acabar a tarefa e se possivel notificar
      mov_in_task_index++;
      task_pre_processing();

      //agora nao necessito de utilizar o h_results nem h_dual_results pro referencia

    }

    //--------------------Pre-Process-------------------------
    read_usr_interface();

    //esta condição terá que ser diferente para suportar a task!!!
    //provavelmente nao precisa de considção de sucesso pois o planeamento já esta feito
    //posso aproveitar este facto para destinguir o tratamento do codigo de uma task relativamente a um mov
    if (usr_interface_input.plan_hand_pos &&
        (
            // movimento único
            ((this->h_results != nullptr && this->h_results->status == 0) ||(this->h_dual_results != nullptr && this->h_dual_results->status == 0))
            ||
            // modo task
            (ui.radioButton_ctrl_task->isChecked() && ((h_r_flag == 1 && status == 0) || (h_r_flag == 2 && status == 0)))
        )
    ){
      use_plan_hand_pos();
    }
    else {
      des_pose.des_hand_pos_x = ui.lineEdit_des_right_hand_pos_x->text().toDouble();
      des_pose.des_hand_pos_y = ui.lineEdit_des_right_hand_pos_y->text().toDouble();
      des_pose.des_hand_pos_z = ui.lineEdit_des_right_hand_pos_z->text().toDouble();
      des_pose.des_hand_or_q_x = ui.lineEdit_des_right_hand_q_x->text().toDouble();
      des_pose.des_hand_or_q_y = ui.lineEdit_des_right_hand_q_y->text().toDouble();
      des_pose.des_hand_or_q_z = ui.lineEdit_des_right_hand_q_z->text().toDouble();
      des_pose.des_hand_or_q_w = ui.lineEdit_des_right_hand_q_w->text().toDouble();
      //dual-arm to do
    }

    if(usr_interface_input.obsts_en) obs_avoidance();
    if(usr_interface_input.hl_en) human_likeness_pre_process(); //dual-arm to do

    // -------------- simulation or real robot --------------------------------- //
    qnode.setSimRobot(usr_interface_input.sim_robot);
    bool condition; // condition to process the control

    if(usr_interface_input.sim_robot){
      // ---------------- start the simulation --------------------------- //
      if(!qnode.isSimulationRunning() || qnode.isSimulationPaused())
      {
        // enable set joints subscriber
        qnode.enableSetJoints();

        // start the simulation
        qnode.startSim();
      }
      time_var.time_step = qnode.getSimTimeStep(); // sec
      condition = qnode.isSimulationRunning() && ((qnode.getSimTime()-ctx.t_j_past)>time_var.time_step);

    }else{
      time_var.time_step = 0.005; // time step of receiving the state of the joints from ARoS [sec]
      condition = (Clock::now() - ctx.t_j_past_ctrl) > boost::chrono::duration<double,boost::ratio<1>>(time_var.time_step);

    }
    ros::spinOnce(); // handle ROS messages



    //--------------Processing--------------------------------
    if (condition){
        //robot update functions
          mes_pose();
          calc_error();

        //time update
          if(usr_interface_input.sim_robot){
              ctx.curr_time = qnode.getSimTime() - ctx.t_past - ctx.t_der_past;
          }else{
              ctx.curr_time_ctrl = Clock::now() - ctx.t_past_ctrl - ctx.t_der_past_ctrl;
          }

          //velocity profile
          if(usr_interface_input.plan_hand_pos)
          {

            if(usr_interface_input.hl_en){
            //human-like velocity profile

              human_like_exec();
            }
            else{
            //trapezoidal velocity profile

              exec_trapezoidal_vel_profile();
            }
          }

          // closed-loop control
          //VectorXd hand_vel_xd_vec(6);
          //VectorXd hand_acc_xd_vec = VectorXd::Zero(6);
          des_pose.hand_acc_xd_vec = VectorXd::Zero(6);

          if(usr_interface_input.hl_en){
          //pagina 205 dissertação Gianpaollo Gulleta
            //lidar com hand_j_acc
              des_pose.hand_acc_xd_vec = error_var.h_hand_ref_acc + human_likeness_coef.Koeff_d*error_var.der_error_h_tot + human_likeness_coef.Koeff_p*error_var.error_h_tot - ctx.hand_j_acc;
              JointState.alpha_acc_xd = (error_var.des_alpha_acc + human_likeness_coef.hl_alpha_vel_coeff*error_var.error_alpha_vel + human_likeness_coef.hl_alpha_pos_coeff*error_var.error_alpha_pos) - ctx.alpha_j_acc;
              if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
                des_pose.hand_acc_xd_vec_left = error_var.h_hand_ref_acc_left + human_likeness_coef.Koeff_d * error_var.der_error_h_tot_left + human_likeness_coef.Koeff_p * error_var.error_h_tot_left - ctx.hand_j_acc;
                JointState.alpha_acc_xd_left = (error_var.des_alpha_acc_left +  human_likeness_coef.hl_alpha_vel_coeff * error_var.error_alpha_vel_left + human_likeness_coef.hl_alpha_pos_coeff * error_var.error_alpha_pos_left) - ctx.alpha_j_acc;
              }
          }else{
              des_pose.hand_acc_xd_vec = error_var.trap_hand_ref_acc + human_likeness_coef.Koeff_d*error_var.der_error_trap_tot + human_likeness_coef.Koeff_p*error_var.error_trap_tot - ctx.hand_j_acc;
              if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
                  des_pose.hand_acc_xd_vec_left = error_var.trap_hand_ref_acc_left + human_likeness_coef.Koeff_d*error_var.der_error_trap_tot_left + human_likeness_coef.Koeff_p*error_var.error_trap_tot_left - ctx.hand_j_acc;//falta alterar
              }
          }

          des_pose.hand_acc_vec.resize(des_pose.hand_acc_xd_vec.size());
          VectorXd::Map(&des_pose.hand_acc_vec[0], des_pose.hand_acc_xd_vec.size()) =  des_pose.hand_acc_xd_vec;

          if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
            des_pose.hand_acc_vec_left.resize(des_pose.hand_acc_xd_vec_left.size());
            VectorXd::Map(&des_pose.hand_acc_vec_left[0], des_pose.hand_acc_xd_vec_left.size()) =  des_pose.hand_acc_xd_vec_left;
          }

          //dual arm to do

          stage_ctrl_logic();


            // inverse algorithm
            ctx.curr_scene->getHumanoid()->inverseDiffKinematicsSingleArm2(1,mes_pose_str.r_arm_posture_mes,des_pose.hand_acc_vec,JointState.alpha_acc_xd,mes_pose_str.r_arm_velocities,
                                                                          mes_pose_str.r_arm_null_velocities,time_var.time_step,usr_interface_input.hl_alpha_en,usr_interface_input.jlim_en,
                                                                          usr_interface_input.sing_en,usr_interface_input.obsts_en,usr_interface_input.vel_max,avoidance_var.sing_coeff,
                                                                          avoidance_var.sing_damping,avoidance_var.obst_coeff,avoidance_var.obst_damping,avoidance_var.obst_coeff_torso,
                                                                          avoidance_var.obst_damping_torso,avoidance_var.jlim_th,avoidance_var.jlim_rate,avoidance_var.jlim_coeff,
                                                                          avoidance_var.jlim_damping,avoidance_var.obsts_n);
            //dual_Arm
            if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
              //mudar JointState.alpha_acc_xd, depois
              ctx.curr_scene->getHumanoid()->inverseDiffKinematicsSingleArm2(2,mes_pose_str.l_arm_posture_mes, des_pose.hand_acc_vec_left,JointState.alpha_acc_xd_left,mes_pose_str.l_arm_velocities,
                                                                            mes_pose_str.l_arm_null_velocities,time_var.time_step,usr_interface_input.hl_alpha_en,usr_interface_input.jlim_en,
                                                                            usr_interface_input.sing_en,usr_interface_input.obsts_en,usr_interface_input.vel_max,avoidance_var.sing_coeff,
                                                                            avoidance_var.sing_damping,avoidance_var.obst_coeff,avoidance_var.obst_damping,avoidance_var.obst_coeff_torso,
                                                                            avoidance_var.obst_damping_torso,avoidance_var.jlim_th,avoidance_var.jlim_rate,avoidance_var.jlim_coeff,
                                                                            avoidance_var.jlim_damping,avoidance_var.obsts_n);
            }

            // execute the control
            if(ctx.exec_command_ctrl){
                if(usr_interface_input.sim_robot){
                   if((h_results != nullptr) || (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 1))){
                      qnode.execKinControl(1,mes_pose_str.r_arm_posture_mes,mes_pose_str.r_arm_velocities,mes_pose_str.r_hand_posture_mes,
                                         mes_pose_str.r_hand_velocities,usr_interface_input.joints_arm_vel_ctrl,true);
                    }
                    if((h_dual_results != nullptr) || (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
                      qnode.execKinDualControl(0,mes_pose_str.r_arm_posture_mes,mes_pose_str.r_arm_velocities,mes_pose_str.r_hand_posture_mes,mes_pose_str.r_hand_velocities,
                                             mes_pose_str.l_arm_posture_mes,mes_pose_str.l_arm_velocities,mes_pose_str.l_hand_posture_mes,mes_pose_str.l_hand_velocities,
                                             usr_interface_input.joints_arm_vel_ctrl,true);
                    }
                }else{
                   qnode.execKinRealControl(1,mes_pose_str.r_arm_velocities,mes_pose_str.r_hand_velocities,true);
                }
            }else{
                // stop the motion
                std::vector<double> r_arm_velocities_0(JOINTS_ARM,0.0);
                std::vector<double> r_hand_velocities_0(JOINTS_HAND,0.0);
                if(usr_interface_input.sim_robot){
                    qnode.execKinControl(1,mes_pose_str.r_arm_posture_mes,r_arm_velocities_0,mes_pose_str.r_hand_posture_mes,r_hand_velocities_0,
                                         usr_interface_input.joints_arm_vel_ctrl,true);
                }else{
                   qnode.execKinRealControl(1,r_arm_velocities_0,r_hand_velocities_0,true);
                }
            }

          Recording();

      } //if condition
    }//if
  }//while
}//ex_pos_ctrl end




void LocalPlanner::task_pre_processing(){
  const TaskPlannedMovData& mov = all_planned_task_movements[mov_in_task_index];

  ctx.curr_mov                                  = mov.curr_mov;
  ctx.des_handPosition                          = mov.des_handPosition;
  ctx.des_handPosition_left                     = mov.des_handPosition_left;

  ctx.handPosition_mov_stages                   = mov.handPosition_mov_stages;
  ctx.handOrientation_mov_stages                = mov.handOrientation_mov_stages;
  ctx.handOrientation_q_mov_stages              = mov.handOrientation_q_mov_stages;
  ctx.handLinearVelocity_mov_stages             = mov.handLinearVelocity_mov_stages;
  ctx.handAngularVelocity_mov_stages            = mov.handAngularVelocity_mov_stages;
  ctx.handLinearAcceleration_mov_stages         = mov.handLinearAcceleration_mov_stages;
  ctx.handAngularAcceleration_mov_stages        = mov.handAngularAcceleration_mov_stages;
  ctx.swivel_angle_mov_stages                   = mov.swivel_angle_mov_stages;
  ctx.der_swivel_angle_mov_stages               = mov.der_swivel_angle_mov_stages;
  ctx.der_der_swivel_angle_mov_stages           = mov.der_der_swivel_angle_mov_stages;

  ctx.handPosition_mov_stages_left              = mov.handPosition_mov_stages_left;
  ctx.handOrientation_mov_stages_left           = mov.handOrientation_mov_stages_left;
  ctx.handOrientation_q_mov_stages_left         = mov.handOrientation_q_mov_stages_left;
  ctx.handLinearVelocity_mov_stages_left        = mov.handLinearVelocity_mov_stages_left;
  ctx.handAngularVelocity_mov_stages_left       = mov.handAngularVelocity_mov_stages_left;
  ctx.handLinearAcceleration_mov_stages_left    = mov.handLinearAcceleration_mov_stages_left;
  ctx.handAngularAcceleration_mov_stages_left   = mov.handAngularAcceleration_mov_stages_left;
  ctx.swivel_angle_mov_stages_left              = mov.swivel_angle_mov_stages_left;
  ctx.der_swivel_angle_mov_stages_left          = mov.der_swivel_angle_mov_stages_left;
  ctx.der_der_swivel_angle_mov_stages_left      = mov.der_der_swivel_angle_mov_stages_left;

  ctx.jointsPosition_mov_ctrl                   = mov.jointsPosition_mov_ctrl;
  ctx.jointsVelocity_mov_ctrl                   = mov.jointsVelocity_mov_ctrl;
  ctx.jointsAcceleration_mov_ctrl               = mov.jointsAcceleration_mov_ctrl;

  ctx.jointsPosition_mov_ctrl_left              = mov.jointsPosition_mov_ctrl_left;
  ctx.jointsVelocity_mov_ctrl_left              = mov.jointsVelocity_mov_ctrl_left;
  ctx.jointsAcceleration_mov_ctrl_left          = mov.jointsAcceleration_mov_ctrl_left;

  ctx.bounce_handPosition                       = mov.bounce_handPosition;
  ctx.bounce_handOrientation                    = mov.bounce_handOrientation;
  ctx.bounce_handOrientation_q                  = mov.bounce_handOrientation_q;
  ctx.bounce_posture                            = mov.bounce_posture;

  ctx.bounce_handPosition_left                  = mov.bounce_handPosition_left;
  ctx.bounce_handOrientation_left               = mov.bounce_handOrientation_left;
  ctx.bounce_handOrientation_q_left             = mov.bounce_handOrientation_q_left;
  ctx.bounce_posture_left                       = mov.bounce_posture_left;

  ctx.dHO_ctrl                                  = mov.dHO_ctrl;
  ctx.dHO_ctrl_left                             = mov.dHO_ctrl_left;
  ctx.approach_ctrl                             = mov.approach_ctrl;
  ctx.retreat_ctrl                              = mov.retreat_ctrl;
  ctx.i_tar_ctrl                                = mov.i_tar_ctrl;
  ctx.i_tar_ctrl_left                           = mov.i_tar_ctrl_left;

  trajectory_descriptions                       = mov.trajectory_descriptions;
  status                                        = mov.status;
  h_r_flag                                      = mov.h_r_flag;
  ctx.timesteps_mov                             = mov.timesteps_mov;
}


/**********************************************
*           Stage Control Logic               *
* Controls the logic for advancing in stages  *
* of the planned movement                     *
**********************************************/

void LocalPlanner::stage_ctrl_logic(){
  // position Koeff
  Vector3d error_abs_pos; Vector3d error_abs_or;
  error_abs_pos << abs(mes_pose_str.error_pos(0)),abs(mes_pose_str.error_pos(1)),abs(mes_pose_str.error_pos(2));
  error_abs_or << abs(mes_pose_str.error_or(0)),abs(mes_pose_str.error_or(1)),abs(mes_pose_str.error_or(2));
  mes_pose_str.e_n_pos = mes_pose_str.error_pos.norm(); mes_pose_str.e_n_or = mes_pose_str.error_or.norm();

  //nota para funcionar no cenario de controlo mudei err_pos para 4.5 e err_or para 0.9
  bool condition = (mes_pose_str.e_n_pos < 1.73*human_likeness_coef.error_pos_th) && (mes_pose_str.e_n_or < 1.73* human_likeness_coef.error_or_th) && ((ctx.curr_time-temp_time)>15);
  if(ui.checkBox_follow_target->isChecked()){
    // check proximity
    qnode.checkProximityObject(ctx.curr_mov,stage_descr);
    if(stages==3 && usr_interface_input.hl_en && stage_descr.compare("plan")==0)
    {
        condition = (time_var.g_map >= time_var.g_map_th_pa);

    }else if((stage_descr.compare("approach")==0) && usr_interface_input.follow_tar && (des_pose.hand_vel_vec_x.norm()<=5.0)){
        // the target was being followed and it has stopped
        usr_interface_input.follow_tar = false; ui.checkBox_follow_target->setChecked(false); // the target has stopped ????????? -> verificar se deixo ficar
        // reset the normalized time, but do not go to the next stage (the approach has to be performed)
        if(usr_interface_input.sim_robot){
            //this->qnode.openBarrettHand_to_pos(1,jointsInitPosition_hand_vec); // open the hand to the initial position of the approach stage
            ctx.t_past = ctx.curr_time;
        }else{
            ctx.t_past_ctrl = boost::chrono::duration_cast<msec>(ctx.curr_time_ctrl - ctx.start_time_point);
        }
    }else if(stages==3 && usr_interface_input.hl_en && stage_descr.compare("approach")==0){
        condition = (time_var.g_map >= time_var.g_map_th_rp);
    }else if(stages==3 && usr_interface_input.hl_en && stage_descr.compare("retreat")==0){
        if(mov_type==0 && !usr_interface_input.sim_robot){//pick
            // close the Barrett Hand of ARoS
            //this->qnode.open_close_BH(true); //corrigir !!
        }
    }
  }
  if (condition){
      //this->qnode.log(QNode::Info,string("Simulation Time: ")+boost::lexical_cast<std::string>(this->qnode.getSimTime()));
      // reset the normalized time, but do not go to the next stage (the approach has to be performed)
      if(usr_interface_input.sim_robot){
          ctx.t_past=ctx.curr_time;
          temp_time = ctx.curr_time;
      }else{
          ctx.t_past_ctrl = boost::chrono::duration_cast<msec>(ctx.curr_time_ctrl - ctx.start_time_point);
      }
      if(stages==3 && ctx.i_ctrl<2){
          if(!usr_interface_input.hl_en){
              if(stage_descr.compare("plan")==0){
                  //this->qnode.openBarrettHand_to_pos(1,jointsFinalPosition_hand_vec); //corrigir pos da mao
              }
              if(stage_descr.compare("approach")==0){
                  //this->qnode.closeBarrettHand_to_pos(1,jointsFinalPosition_hand_vec);
              }
          }
          ctx.i_ctrl++; // go to the next stage
      }else if(stages==2 && ctx.i_ctrl<1){
          // TO DO
          ctx.i_ctrl++;
          //temp_time = ctx.curr_time;
      }

      //passar para o proximo planeamento
      else if(stages == 2 && ctx.i_ctrl<2){
        mov_in_task_index++;
        ctx.i_ctrl = 0;
        //temp_time = ctx.curr_time;
        task_pre_processing();
        qnode.resetSimTime();
      }
  } // if condition

}





/**********************************************
*               Exec Vel Ctrl             		*
* Executs the main floow of the velocity      *
* controller                                  *
**********************************************/
void LocalPlanner::exec_vel_ctrl(){
  while(ctx.exec_control)
  {

      if(ctx.vel_control)
      //if(true)
      {
          boost::unique_lock<boost::mutex> lck(ctx.hh_control_mtx);

          double obsts_x_var = 100; // mm
          double obsts_y_var = 100; // mm
          double obsts_z_var = 100; // mm
          double obsts_q_x_var = 0.1;
          double obsts_q_y_var = 0.1;
          double obsts_q_z_var = 0.1;
          double obsts_q_w_var = 0.1;

          double des_hand_vel_x,des_hand_vel_y,des_hand_vel_z,des_hand_vel_wx,des_hand_vel_wy,des_hand_vel_wz;
          if (ui.checkBox_draw_ellipse->isChecked()){
              des_hand_vel_x = 200*cos(0.2*qnode.getSimTime()-M_PI/2);
              des_hand_vel_y = -300*sin(0.2*qnode.getSimTime()-M_PI/2);
              des_hand_vel_z = 0.1;
              des_hand_vel_wx = 0.0;
              des_hand_vel_wy = 0.0;
              des_hand_vel_wz = 0.0;
          }else{
              des_hand_vel_x = ui.lineEdit_des_right_hand_vel_x->text().toDouble();
              des_hand_vel_y = ui.lineEdit_des_right_hand_vel_y->text().toDouble();
              des_hand_vel_z = ui.lineEdit_des_right_hand_vel_z->text().toDouble();
              des_hand_vel_wx = ui.lineEdit_des_right_hand_vel_wx->text().toDouble();
              des_hand_vel_wy = ui.lineEdit_des_right_hand_vel_wy->text().toDouble();
              des_hand_vel_wz = ui.lineEdit_des_right_hand_vel_wz->text().toDouble();
          }

          if(!ui.checkBox_des_right_hand_vel_x->isChecked()){
              des_hand_vel_x = 0.0;
          }
          if(!ui.checkBox_des_right_hand_vel_y->isChecked()){
              des_hand_vel_y = 0.0;
          }
          if(!ui.checkBox_des_right_hand_vel_z->isChecked()){
              des_hand_vel_z = 0.0;
          }
          if(!ui.checkBox_des_right_hand_vel_wx->isChecked()){
              des_hand_vel_wx = 0.0;
          }
          if(!ui.checkBox_des_right_hand_vel_wy->isChecked()){
              des_hand_vel_wy = 0.0;
          }
          if(!ui.checkBox_des_right_hand_vel_wz->isChecked()){
              des_hand_vel_wz = 0.0;
          }
          vector<double> hand_vel_vec = {des_hand_vel_x,des_hand_vel_y,des_hand_vel_z, des_hand_vel_wx,des_hand_vel_wy,des_hand_vel_wz};
          VectorXd hand_vel = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(hand_vel_vec.data(), hand_vel_vec.size());

          double vel_max = ui.lineEdit_vel_max->text().toDouble()*M_PI/180;

          bool jlim_en = ui.checkBox_joints_limits_av->isChecked();
          double jlim_th = 0; double jlim_rate = 1;
          double jlim_coeff = 1; double jlim_damping = 0.001;
          if(jlim_en){
              jlim_th = ui.lineEdit_jlim_th->text().toDouble();
              jlim_rate = ui.lineEdit_jlim_rate->text().toDouble();
              jlim_coeff = ui.lineEdit_jlim_coeff->text().toDouble();
              jlim_damping = ui.lineEdit_jlim_damping->text().toDouble();
          }
          bool sing_en = ui.checkBox_sing_av->isChecked();
          double sing_coeff = 1; double sing_damping = 0.001;
          if(sing_en){
              sing_damping = ui.lineEdit_sing_damping->text().toDouble();
              sing_coeff = ui.lineEdit_sing_coeff->text().toDouble();
          }


//------------------------------ OBSTACLE AVOIDANCE -------------------------------------
          double obst_coeff = 1; double obst_damping = 0.001;
          double obst_coeff_torso = 1; double obst_damping_torso = 0.001;
          bool obsts_en = ui.checkBox_obsts_av->isChecked();
          vector<objectPtr> obsts_n; // obstacles in the scenario
          if(obsts_en){
              ctx.curr_scene->getObjects(obsts_n);
              obst_coeff = ui.lineEdit_obsts_coeff->text().toDouble();
              obst_damping = ui.lineEdit_obsts_damping->text().toDouble();
              obst_coeff_torso = ui.lineEdit_obsts_coeff_torso->text().toDouble();
              obst_damping_torso = ui.lineEdit_obsts_damping_torso->text().toDouble();
              if(ui.checkBox_obsts_noise->isChecked()){
                  vector<objectPtr> obsts; ctx.curr_scene->getObjects(obsts);
                  obsts_n.resize(obsts.size());
                  objectPtr obs_new;
                  for(size_t i=0; i<obsts.size(); ++i){
                      objectPtr obs = obsts.at(i);
                      string obs_name = obs->getName();
                      obs_new.reset(new Object(obs_name));
                      motion_manager::pos obs_pos;
                      Quaterniond obs_or_q;
                      obs_pos.Xpos = obs->getPos().Xpos - (obsts_x_var/2) + obsts_x_var*(rand() / double(RAND_MAX));
                      obs_pos.Ypos = obs->getPos().Ypos - (obsts_y_var/2) + obsts_y_var*(rand() / double(RAND_MAX));
                      obs_pos.Zpos = obs->getPos().Zpos - (obsts_z_var/2) + obsts_z_var*(rand() / double(RAND_MAX));
                      obs_or_q.x() = obs->getQuaternion().x() - (obsts_q_x_var/2) + obsts_q_x_var*(rand() / double(RAND_MAX));
                      obs_or_q.y() = obs->getQuaternion().y() - (obsts_q_y_var/2) + obsts_q_y_var*(rand() / double(RAND_MAX));
                      obs_or_q.z() = obs->getQuaternion().z() - (obsts_q_z_var/2) + obsts_q_z_var*(rand() / double(RAND_MAX));
                      obs_or_q.w() = obs->getQuaternion().w() - (obsts_q_w_var/2) + obsts_q_w_var*(rand() / double(RAND_MAX));
                      if(ui.checkBox_obsts_filter_noise->isChecked()){
                          obs_pos.Xpos = lpf.lpf_obsts_pos_x->update(obs_pos.Xpos);
                          obs_pos.Ypos = lpf.lpf_obsts_pos_y->update(obs_pos.Ypos);
                          obs_pos.Zpos = lpf.lpf_obsts_pos_z->update(obs_pos.Zpos);
                          obs_or_q.x() = lpf.lpf_obsts_or_q_x->update(obs_or_q.x());
                          obs_or_q.y() = lpf.lpf_obsts_or_q_y->update(obs_or_q.y());
                          obs_or_q.z() = lpf.lpf_obsts_or_q_z->update(obs_or_q.z());
                          obs_or_q.w() = lpf.lpf_obsts_or_q_w->update(obs_or_q.w());
                      }
                      obs_new->setPos(obs_pos,false);
                      obs_new->setOr(obs_or_q,false);
                      obs_new->setSize(obs->getSize());
                      obsts_n.at(i) = obs_new;
                  }// for loop obstacles
              }// noise on obstacles
          }//obstacle avoidance enable
//---------------------------------------------------------------------------------------------------------------------

          // ---------------- start the simulation --------------------------- //
          if(!qnode.isSimulationRunning() || qnode.isSimulationPaused())
          {
              // enable set joints subscriber
              qnode.enableSetJoints();

              // start the simulation
              qnode.startSim();
          }
          ros::spinOnce(); // handle ROS messages
          double time_step = qnode.getSimTimeStep(); // sec

          if(qnode.isSimulationRunning() && ((qnode.getSimTime()-ctx.t_j_past)>time_step))
          {
            //acrscentei +3 ao joints hand, solucao temporaria mudar depois!!!!!!!!
              // posture
              vector<double> r_arm_posture_mes(JOINTS_ARM,0.0); vector<double> r_hand_posture_mes(JOINTS_HAND,0.0);
              vector<double> r_arm_posture(JOINTS_ARM,0.0); vector<double> r_hand_posture(JOINTS_HAND,0.0);
              // velocities
              VectorXd r_arm_null_velocities = VectorXd::Zero(JOINTS_ARM);
              vector<double> r_arm_velocities(JOINTS_ARM,0.0); vector<double> r_hand_velocities(JOINTS_HAND,0.0);
              vector<double> r_arm_velocities_read(JOINTS_ARM,0.0); vector<double> r_hand_velocities_read(JOINTS_HAND,0.0);
              // accelerations
              vector<double> r_arm_accelerations(JOINTS_ARM,0.0); vector<double> r_hand_accelerations(JOINTS_HAND,0.0);

              ctx.curr_scene->getHumanoid()->getRightArmPosture(r_arm_posture_mes);
              ctx.curr_scene->getHumanoid()->getRightHandPosture(r_hand_posture_mes);

              // filtering the joint positions
              r_arm_posture.at(0) = lpf.lpf_joint_pos_1->update(r_arm_posture_mes.at(0));
              r_arm_posture.at(1) = lpf.lpf_joint_pos_2->update(r_arm_posture_mes.at(1));
              r_arm_posture.at(2) = lpf.lpf_joint_pos_3->update(r_arm_posture_mes.at(2));
              r_arm_posture.at(3) = lpf.lpf_joint_pos_4->update(r_arm_posture_mes.at(3));
              r_arm_posture.at(4) = lpf.lpf_joint_pos_5->update(r_arm_posture_mes.at(4));
              r_arm_posture.at(5) = lpf.lpf_joint_pos_6->update(r_arm_posture_mes.at(5));
              r_arm_posture.at(6) = lpf.lpf_joint_pos_7->update(r_arm_posture_mes.at(6));
#if HAND ==1
              r_hand_posture.at(0) = lpf.lpf_joint_pos_8->update(r_arm_posture_mes.at(0));
              r_hand_posture.at(1) = lpf.lpf_joint_pos_9->update(r_hand_posture_mes.at(1));
              r_hand_posture.at(2) = lpf.lpf_joint_pos_10->update(r_hand_posture_mes.at(2));
              r_hand_posture.at(3) = lpf.lpf_joint_pos_11->update(r_hand_posture_mes.at(3));
#elif HAND ==2
              r_hand_posture.at(0) = lpf.lpf_joint_pos_8->update(r_hand_posture_mes.at(0));
#endif

              // get the joint velocities
              ctx.arm_pos_buff->push(r_arm_posture);
              ctx.hand_pos_buff->push(r_hand_posture);
              if(ctx.samples_pos==lpf.N_filter_length-1 && ctx.arm_pos_buff->full() && ctx.hand_pos_buff->full()){
                  for(size_t i=0; i< r_arm_posture.size();++i)
                  {
                      r_arm_velocities_read.at(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_step,ctx.arm_pos_buff->at(i));
                  }
                  for(size_t i=0; i< r_hand_velocities_read.size();++i)
                  {
                      r_hand_velocities_read.at(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_step,ctx.hand_pos_buff->at(i));
                  }
              }else{ctx.samples_pos++;}

              // get the joint accelerations
              ctx.arm_vel_buff->push(r_arm_velocities_read);
              ctx.hand_vel_buff->push(r_hand_velocities_read);
              if(ctx.samples_vel==lpf.N_filter_length-1 && ctx.arm_vel_buff->full() && ctx.hand_vel_buff->full()){
                  for(size_t i=0; i< r_arm_velocities_read.size();++i)
                  {
                      r_arm_accelerations.at(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_step,ctx.arm_vel_buff->at(i));
                  }
                  for(size_t i=0; i< r_hand_velocities_read.size();++i)
                  {
                      r_hand_accelerations.at(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_step,ctx.hand_vel_buff->at(i));
                  }
              }else{ctx.samples_vel++;}

              vector<double> r_hand_pos; vector<double> r_wrist_pos; vector<double> r_elbow_pos; vector<double> r_shoulder_pos;
              ctx.curr_scene->getHumanoid()->getAllPos(1,r_hand_pos,r_wrist_pos,r_elbow_pos,r_shoulder_pos,r_arm_posture);
              VectorXd r_hand_pos_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(r_hand_pos.data(), r_hand_pos.size());

              vector<double> r_hand_lin_pos(r_hand_pos.begin(), r_hand_pos.begin()+3);
              vector<double> r_hand_ang_pos(r_hand_pos.begin()+3, r_hand_pos.begin()+6);
              vector<double> r_wrist_lin_pos(r_wrist_pos.begin(), r_wrist_pos.begin()+3);
              vector<double> r_wrist_ang_pos(r_wrist_pos.begin()+3, r_wrist_pos.begin()+6);
              vector<double> r_elbow_lin_pos(r_elbow_pos.begin(), r_elbow_pos.begin()+3);
              vector<double> r_elbow_ang_pos(r_elbow_pos.begin()+3, r_elbow_pos.begin()+6);
              vector<double> r_shoulder_lin_pos(r_shoulder_pos.begin(), r_shoulder_pos.begin()+3);
              vector<double> r_shoulder_ang_pos(r_shoulder_pos.begin()+3, r_shoulder_pos.begin()+6);
              // versor on the elbow point
              VectorXd r_wrist_lin_pos_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(r_wrist_lin_pos.data(), r_wrist_lin_pos.size());
              VectorXd r_elbow_lin_pos_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(r_elbow_lin_pos.data(), r_elbow_lin_pos.size());
              VectorXd r_shoulder_lin_pos_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(r_shoulder_lin_pos.data(), r_shoulder_lin_pos.size());
              Vector3d r_se_lin_pos_vec = r_shoulder_lin_pos_vec - r_elbow_lin_pos_vec;
              Vector3d r_ew_lin_pos_vec = r_elbow_lin_pos_vec - r_wrist_lin_pos_vec;
              Vector3d u_alpha_vec = r_se_lin_pos_vec.cross(r_ew_lin_pos_vec);
              Vector3d u_alpha = u_alpha_vec/u_alpha_vec.norm();

              vector<double> r_hand_vel; vector<double> r_wrist_vel; vector<double> r_elbow_vel; vector<double> r_shoulder_vel;
              ctx.curr_scene->getHumanoid()->getAllVel(1,r_hand_vel,r_wrist_vel,r_elbow_vel,r_shoulder_vel,r_arm_posture,r_arm_velocities_read);
              VectorXd r_hand_vel_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(r_hand_vel.data(), r_hand_vel.size());

              vector<double> r_hand_lin_vel(r_hand_vel.begin(), r_hand_vel.begin()+3);
              vector<double> r_hand_ang_vel(r_hand_vel.begin()+3, r_hand_vel.begin()+6);
              vector<double> r_wrist_lin_vel(r_wrist_vel.begin(), r_wrist_vel.begin()+3);
              vector<double> r_wrist_ang_vel(r_wrist_vel.begin()+3, r_wrist_vel.begin()+6);
              vector<double> r_elbow_lin_vel(r_elbow_vel.begin(), r_elbow_vel.begin()+3);
              vector<double> r_elbow_ang_vel(r_elbow_vel.begin()+3, r_elbow_vel.begin()+6);
              vector<double> r_shoulder_lin_vel(r_shoulder_vel.begin(), r_shoulder_vel.begin()+3);
              vector<double> r_shoulder_ang_vel(r_shoulder_vel.begin()+3, r_shoulder_vel.begin()+6);

              // get the wrist acceleration
              vector<double> r_wrist_acc(6,0.0);
              ctx.r_wrist_vel_buff->push(r_wrist_vel);
              if(ctx.samples_w_vel==lpf.N_filter_length-1 && ctx.r_wrist_vel_buff->full()){
                  for(size_t i=0; i< r_wrist_vel.size();++i)
                  {
                      r_wrist_acc.at(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_step,ctx.r_wrist_vel_buff->at(i));
                  }
              }else{ctx.samples_w_vel++;}

              // get the elbow acceleration
              vector<double> r_elbow_acc(6,0.0);
              ctx.r_elbow_vel_buff->push(r_elbow_vel);
              if(ctx.samples_e_vel==lpf.N_filter_length-1 && ctx.r_elbow_vel_buff->full()){
                  for(size_t i=0; i< r_elbow_vel.size();++i)
                  {
                      r_elbow_acc.at(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_step,ctx.r_elbow_vel_buff->at(i));
                  }
              }else{ctx.samples_e_vel++;}

              // get the shoulder acceleration
              vector<double> r_shoulder_acc(6,0.0);
              ctx.r_shoulder_vel_buff->push(r_shoulder_vel);
              if(ctx.samples_s_vel==lpf.N_filter_length-1 && ctx.r_shoulder_vel_buff->full()){
                  for(size_t i=0; i< r_shoulder_vel.size();++i)
                  {
                      r_shoulder_acc.at(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_step,ctx.r_shoulder_vel_buff->at(i));
                  }
              }else{ctx.samples_s_vel++;}

              // get the hand acceleration
              vector<double> r_hand_acc(6,0.0);
              ctx.r_hand_vel_buff->push(r_hand_vel);
              if(ctx.samples_h_vel==lpf.N_filter_length-1 && ctx.r_hand_vel_buff->full()){
                  for(size_t i=0; i< r_hand_vel.size();++i)
                  {
                      r_hand_acc.at(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_step,ctx.r_hand_vel_buff->at(i));
                  }
              }else{
                  ctx.samples_h_vel++;
                  ctx.t_der_past = qnode.getSimTime();
              }
              VectorXd r_hand_acc_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(r_hand_acc.data(), r_hand_acc.size());

              vector<double> r_hand_lin_acc(r_hand_acc.begin(), r_hand_acc.begin()+3);
              vector<double> r_hand_ang_acc(r_hand_acc.begin()+3, r_hand_acc.begin()+6);
              vector<double> r_wrist_lin_acc(r_wrist_acc.begin(), r_wrist_acc.begin()+3);
              vector<double> r_wrist_ang_acc(r_wrist_acc.begin()+3, r_wrist_acc.begin()+6);
              vector<double> r_elbow_lin_acc(r_elbow_acc.begin(), r_elbow_acc.begin()+3);
              vector<double> r_elbow_ang_acc(r_elbow_acc.begin()+3, r_elbow_acc.begin()+6);
              vector<double> r_shoulder_lin_acc(r_shoulder_acc.begin(), r_shoulder_acc.begin()+3);
              vector<double> r_shoulder_ang_acc(r_shoulder_acc.begin()+3, r_shoulder_acc.begin()+6);

              if(ctx.r_hand_init_pos.empty()){ctx.r_hand_init_pos=r_hand_pos;}
              VectorXd r_hand_init_pos_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(ctx.r_hand_init_pos.data(), ctx.r_hand_init_pos.size());
              VectorXd hand_pos(6); VectorXd hand_acc(6);
              if (ui.checkBox_draw_ellipse->isChecked()){
                  // desired position
                  hand_pos(0) = r_hand_init_pos_vec(0)/0.2 + (40/0.2)*sin(0.2*qnode.getSimTime() - M_PI/2);
                  hand_pos(1) = r_hand_init_pos_vec(1)/0.2 + (60/0.2)*cos(0.2*qnode.getSimTime() - M_PI/2);
                  hand_pos(2) = r_hand_init_pos_vec(2);
                  hand_pos(3) = r_hand_init_pos_vec(3);
                  hand_pos(4) = r_hand_init_pos_vec(4);
                  hand_pos(5) = r_hand_init_pos_vec(5);
                  //desired acceleration
                  hand_acc(0) = -40*0.2*sin(0.2*qnode.getSimTime() - M_PI/2);
                  hand_acc(1) = -60*0.2*cos(0.2*qnode.getSimTime() - M_PI/2);
                  hand_acc(2) = 0;
                  hand_acc(3) = 0.0;
                  hand_acc(4) = 0.0;
                  hand_acc(5) = 0.0;
              }else{
                  // desired position
                  hand_pos = r_hand_init_pos_vec + hand_vel*qnode.getSimTime();
                  // desired acceleration
                  hand_acc = hand_vel/qnode.getSimTime();
              }

              // inverse differential kinematics
              ctx.curr_scene->getHumanoid()->inverseDiffKinematicsSingleArm(1,r_arm_posture,hand_vel_vec,r_arm_velocities,r_arm_null_velocities,jlim_en,sing_en,obsts_en,
                                                                              vel_max,sing_coeff,sing_damping,obst_coeff,obst_damping,obst_coeff_torso,obst_damping_torso,
                                                                              jlim_th,jlim_rate,jlim_coeff,jlim_damping,obsts_n);


              // execute the control
              bool joints_arm_vel_ctrl = ui.checkBox_joints_velocity_ctrl->isChecked();
              qnode.execKinControl(1,r_arm_posture,r_arm_velocities,r_hand_posture,r_hand_velocities,joints_arm_vel_ctrl,false);
              //qnode.execKinControl(1,r_arm_posture,r_arm_velocities,r_hand_posture,r_hand_velocities,joints_arm_vel_ctrl,true); //mudei isto
              // -------------- Recording ---------------------------- //

              // record the positions of the joints
              //ctx.jointsPosition_ctrl.conservativeResize(ctx.jointsPosition_ctrl.rows()+1,11);
              ctx.jointsPosition_ctrl.conservativeResize(ctx.jointsPosition_ctrl.rows()+1,JOINTS_ARM+JOINTS_HAND);
              for(size_t jj=0; jj < r_arm_posture.size(); ++jj)
                  ctx.jointsPosition_ctrl(ctx.jointsPosition_ctrl.rows()-1,jj) = r_arm_posture.at(jj);
              for(size_t jj=0; jj < r_hand_posture.size(); ++jj)
                  ctx.jointsPosition_ctrl(ctx.jointsPosition_ctrl.rows()-1,r_arm_posture.size()+jj) = r_hand_posture.at(jj);

              // record the velocities of the joints
              //ctx.jointsVelocity_ctrl.conservativeResize(ctx.jointsVelocity_ctrl.rows()+1,11);
              ctx.jointsVelocity_ctrl.conservativeResize(ctx.jointsVelocity_ctrl.rows()+1,JOINTS_ARM+JOINTS_HAND);
              for(size_t jj=0; jj < r_arm_velocities_read.size(); ++jj)
                  ctx.jointsVelocity_ctrl(ctx.jointsVelocity_ctrl.rows()-1,jj) = r_arm_velocities_read.at(jj);
              for(size_t jj=0; jj < r_hand_velocities_read.size(); ++jj)
                  ctx.jointsVelocity_ctrl(ctx.jointsVelocity_ctrl.rows()-1,r_arm_velocities_read.size()+jj) = r_hand_velocities_read.at(jj);

              ctx.jointsVelocity_null_ctrl.conservativeResize(ctx.jointsVelocity_null_ctrl.rows()+1,JOINTS_ARM);
              for(size_t jj=0; jj < r_arm_null_velocities.size(); ++jj)
                  ctx.jointsVelocity_null_ctrl(ctx.jointsVelocity_null_ctrl.rows()-1,jj) = r_arm_null_velocities(jj);

              // record the acceleration of the joints
              //solucao temporaria mudar depois
              ctx.jointsAcceleration_ctrl.conservativeResize(ctx.jointsAcceleration_ctrl.rows()+1,JOINTS_ARM+JOINTS_HAND);
              for(size_t jj=0; jj < r_arm_accelerations.size(); ++jj)
                  ctx.jointsAcceleration_ctrl(ctx.jointsAcceleration_ctrl.rows()-1,jj) = r_arm_accelerations.at(jj);
              for(size_t jj=0; jj < r_hand_accelerations.size(); ++jj)
                  ctx.jointsAcceleration_ctrl(ctx.jointsAcceleration_ctrl.rows()-1,r_arm_accelerations.size()+jj) = r_hand_accelerations.at(jj);


              // operational space positions
              ctx.handPosition_ctrl.push_back(r_hand_lin_pos);
              ctx.handOrientation_ctrl.push_back(r_hand_ang_pos);
              ctx.wristPosition_ctrl.push_back(r_wrist_lin_pos);
              ctx.wristOrientation_ctrl.push_back(r_wrist_ang_pos);
              ctx.elbowPosition_ctrl.push_back(r_elbow_lin_pos);
              ctx.elbowOrientation_ctrl.push_back(r_elbow_ang_pos);
              ctx.shoulderPosition_ctrl.push_back(r_shoulder_lin_pos);
              ctx.shoulderOrientation_ctrl.push_back(r_shoulder_ang_pos);

              // operational space velocities
              ctx.handLinearVelocity_ctrl.push_back(r_hand_lin_vel);
              ctx.handAngularVelocity_ctrl.push_back(r_hand_ang_vel);
              ctx.wristLinearVelocity_ctrl.push_back(r_wrist_lin_vel);
              ctx.wristAngularVelocity_ctrl.push_back(r_wrist_ang_vel);
              ctx.elbowLinearVelocity_ctrl.push_back(r_elbow_lin_vel);
              ctx.elbowAngularVelocity_ctrl.push_back(r_elbow_ang_vel);
              ctx.shoulderLinearVelocity_ctrl.push_back(r_shoulder_lin_vel);
              ctx.shoulderAngularVelocity_ctrl.push_back(r_shoulder_ang_vel);
              ctx.handVelocityNorm_ctrl.push_back(ctx.curr_scene->getHumanoid()->getHandVelNorm(1,r_arm_posture,r_arm_velocities_read));

              // operational space accelerations
              ctx.handLinearAcceleration_ctrl.push_back(r_hand_lin_acc);
              ctx.handAngularAcceleration_ctrl.push_back(r_hand_ang_acc);
              ctx.wristLinearAcceleration_ctrl.push_back(r_wrist_lin_acc);
              ctx.wristAngularAcceleration_ctrl.push_back(r_wrist_ang_acc);
              ctx.elbowLinearAcceleration_ctrl.push_back(r_elbow_lin_acc);
              ctx.elbowAngularAcceleration_ctrl.push_back(r_elbow_ang_acc);
              ctx.shoulderLinearAcceleration_ctrl.push_back(r_shoulder_lin_acc);
              ctx.shoulderAngularAcceleration_ctrl.push_back(r_shoulder_ang_acc);
              ctx.handAccelerationNorm_ctrl.push_back(sqrt(pow(r_hand_lin_acc.at(0),2)+pow(r_hand_lin_acc.at(1),2)+pow(r_hand_lin_acc.at(2),2)));

              //errors
              VectorXd error_pos_tot = hand_pos - r_hand_pos_vec;
              ctx.error_pos_tot_norm.push_back(error_pos_tot.block<3,1>(0,0).norm());
              ctx.error_or_tot_norm.push_back(error_pos_tot.block<3,1>(3,0).norm());
              ctx.error_pos_or_tot_norm.push_back(error_pos_tot.norm());
              VectorXd error_vel_tot = hand_vel - r_hand_vel_vec;
              ctx.error_lin_vel_tot_norm.push_back(error_vel_tot.block<3,1>(0,0).norm());
              ctx.error_ang_vel_tot_norm.push_back(error_vel_tot.block<3,1>(3,0).norm());
              ctx.error_vel_tot_norm.push_back(error_vel_tot.norm());
              VectorXd error_acc_tot = hand_acc - r_hand_acc_vec;
              ctx.error_lin_acc_tot_norm.push_back(error_acc_tot.block<3,1>(0,0).norm());
              ctx.error_ang_acc_tot_norm.push_back(error_acc_tot.block<3,1>(3,0).norm());
              ctx.error_acc_tot_norm.push_back(error_acc_tot.norm());

              // time
              ctx.t_j_past = qnode.getSimTime();
              ctx.sim_time.push_back(qnode.getSimTime()-ctx.t_der_past);
          } // check simulation step
      } // if vel control
  }// while execcontrol
}

/**************************************************************
*                   Use Plan Hand Pose                        *
*                                                             *
* The code enters this function if the planed hand pose is    *
* to be used as the target for the controller                 *
*                                                             *
* Code execution:                                             *
*	1 - Movement type -  atributs the target (planed hand       *
*	pose and adds noise if needed.                              *
*	2 - Robot info update - updates  various informations       *
*	about the robot.                                            *
*	3 - Movement fase (start pose) - registers the first        *
*	pose of the robot.                                          *
*	4 - Movement fase (end pose) -                              *
*                                                             *
* Note - supports dual arm ✅                                  *
*                                                             *
*   ---- ALTERAÇÕES NECESSARIAS PARA TASK CONTROL ----        *
* - codigo repetido no read_usr_interface (stage_descr)       *
* - traj_move.size === stages = ctx.des_handPosition.size();  *
* `
*
**************************************************************/
void LocalPlanner::use_plan_hand_pos() {

  //single arm mov
  if(!(ui.radioButton_ctrl_task->isChecked()) && this->h_results != nullptr){
    stages = ctx.des_handPosition.size();
    stage_descr = h_results->trajectory_descriptions.at(ctx.i_ctrl);
  }
  //single arm task
  else if(ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 1)){
    stages = ctx.des_handPosition.size();
    stage_descr = this->trajectory_descriptions.at(ctx.i_ctrl);
  }
  //dual arm mov
  else if (!(ui.radioButton_ctrl_task->isChecked()) && this->h_dual_results != nullptr) {
    stages = ctx.des_handPosition.size();
    stage_descr = h_dual_results->trajectory_descriptions.at(ctx.i_ctrl);
    //acresentar para dual
    stages_left = ctx.des_handPosition_left.size();
  }
  //dual arm task
  else if(ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2)){
    stages = ctx.des_handPosition.size();
    stage_descr = this->trajectory_descriptions.at(ctx.i_ctrl);
    //acresentar para dual
    stages_left = ctx.des_handPosition_left.size();
  }

  double dist_app = 0.0; Vector3d vv_app; double dist_ret = 0.0; Vector3d vv_ret;
  double dist_app_left = 0.0; Vector3d vv_app_left; double dist_ret_left = 0.0; Vector3d vv_ret_left;
  mov_type = ctx.curr_mov->getType();
//---------------------- 1- Movement type --------------------------------------
  if(mov_type==0){ // pick
    targetPtr tar;
    targetPtr tar_left;
    if(usr_interface_input.sim_robot){
      tar = ctx.curr_scene->getObject(ctx.i_tar_ctrl)->getTargetRight();
      tar_pos(0) = tar->getPos().Xpos;
      tar_pos(1) = tar->getPos().Ypos;
      tar_pos(2) = tar->getPos().Zpos;
      tar_q = tar->getQuaternion();
       //acresentar para dual
      if ((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))) {
        tar_left = ctx.curr_scene->getObject(ctx.i_tar_ctrl_left)->getTargetLeft();
        tar_pos_left(0) = tar_left->getPos().Xpos;
        tar_pos_left(1) = tar_left->getPos().Ypos;
        tar_pos_left(2) = tar_left->getPos().Zpos;
        tar_q_left = tar_left->getQuaternion();
      }

    }else{
      tar = ctx.curr_scene->getHandTarget();
      tar_pos(0) = tar->getPos().Xpos;
      tar_pos(1) = tar->getPos().Ypos;
      tar_pos(2) = tar->getPos().Zpos;
      tar_q = tar->getQuaternion();

      //acresentar para dual
     if ((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))) {
       tar_left = ctx.curr_scene->getHandTarget(); //<-- indicar mao??!!
       tar_pos_left(0) = tar_left->getPos().Xpos;
       tar_pos_left(1) = tar_left->getPos().Ypos;
       tar_pos_left(2) = tar_left->getPos().Zpos;
       tar_q_left = tar_left->getQuaternion();
     }

     if(stage_descr.compare("plan")==0){mes_pose_str.tar_rec = targetPtr(new Target(*tar.get()));} // record the latest target of the plan stage
    }


    if ((this->h_results != nullptr) || (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 1))) {
        // Single arm
        dist_app = ctx.approach_ctrl.at(3);
        vv_app << ctx.approach_ctrl.at(0),ctx.approach_ctrl.at(1),ctx.approach_ctrl.at(2);
        dist_ret = ctx.retreat_ctrl.at(3);
        vv_ret << ctx.retreat_ctrl.at(0),ctx.retreat_ctrl.at(1),ctx.retreat_ctrl.at(2);
    }
    else if ((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))) {
        // Dual arm
        //Right
        dist_app = ctx.approach_ctrl.at(3);
        vv_app << ctx.approach_ctrl.at(0),ctx.approach_ctrl.at(1),ctx.approach_ctrl.at(2);
        dist_ret = ctx.retreat_ctrl.at(3);
        vv_ret << ctx.retreat_ctrl.at(0),ctx.retreat_ctrl.at(1),ctx.retreat_ctrl.at(2);
        //Left
        dist_app_left = ctx.approach_ctrl.at(7);
        vv_app_left << ctx.approach_ctrl.at(4),ctx.approach_ctrl.at(5),ctx.approach_ctrl.at(6);
        dist_ret_left = ctx.retreat_ctrl.at(7);
        vv_ret_left << ctx.retreat_ctrl.at(4),ctx.retreat_ctrl.at(5),ctx.retreat_ctrl.at(6);
    }


    if(ui.checkBox_tar_noise->isChecked() && usr_interface_input.sim_robot){
       tar_pos(0) = tar_pos(0) - (noise_sim_obj.obj_x_var/2) + noise_sim_obj.obj_x_var*(rand() / double(RAND_MAX));
       tar_pos(1) = tar_pos(1) - (noise_sim_obj.obj_y_var/2) + noise_sim_obj.obj_y_var*(rand() / double(RAND_MAX));
       tar_pos(2) = tar_pos(2) - (noise_sim_obj.obj_z_var/2) + noise_sim_obj.obj_z_var*(rand() / double(RAND_MAX));
       tar_q.x() = tar_q.x() - (noise_sim_obj.obj_q_x_var/2) + noise_sim_obj.obj_q_x_var*(rand() / double(RAND_MAX));
       tar_q.y() = tar_q.y() - (noise_sim_obj.obj_q_y_var/2) + noise_sim_obj.obj_q_y_var*(rand() / double(RAND_MAX));
       tar_q.z() = tar_q.z() - (noise_sim_obj.obj_q_z_var/2) + noise_sim_obj.obj_q_z_var*(rand() / double(RAND_MAX));
       tar_q.w() = tar_q.w() - (noise_sim_obj.obj_q_w_var/2) + noise_sim_obj.obj_q_w_var*(rand() / double(RAND_MAX));

       if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
         tar_pos_left(0) = tar_pos_left(0) - (noise_sim_obj.obj_x_var/2) + noise_sim_obj.obj_x_var*(rand() / double(RAND_MAX));
         tar_pos_left(1) = tar_pos_left(1) - (noise_sim_obj.obj_y_var/2) + noise_sim_obj.obj_y_var*(rand() / double(RAND_MAX));
         tar_pos_left(2) = tar_pos_left(2) - (noise_sim_obj.obj_z_var/2) + noise_sim_obj.obj_z_var*(rand() / double(RAND_MAX));
         tar_q_left.x() = tar_q_left.x() - (noise_sim_obj.obj_q_x_var/2) + noise_sim_obj.obj_q_x_var*(rand() / double(RAND_MAX));
         tar_q_left.y() = tar_q_left.y() - (noise_sim_obj.obj_q_y_var/2) + noise_sim_obj.obj_q_y_var*(rand() / double(RAND_MAX));
         tar_q_left.z() = tar_q_left.z() - (noise_sim_obj.obj_q_z_var/2) + noise_sim_obj.obj_q_z_var*(rand() / double(RAND_MAX));
         tar_q_left.w() = tar_q_left.w() - (noise_sim_obj.obj_q_w_var/2) + noise_sim_obj.obj_q_w_var*(rand() / double(RAND_MAX));
       }
    }
    if(ui.checkBox_tar_filter_noise->isChecked()){
       tar_pos(0) = lpf.lpf_tar_pos_x->update(tar_pos(0));
       tar_pos(1) = lpf.lpf_tar_pos_y->update(tar_pos(1));
       tar_pos(2) = lpf.lpf_tar_pos_z->update(tar_pos(2));
       tar_q.x() = lpf.lpf_tar_or_q_x->update(tar_q.x());
       tar_q.y() = lpf.lpf_tar_or_q_y->update(tar_q.y());
       tar_q.z() = lpf.lpf_tar_or_q_z->update(tar_q.z());
       tar_q.w() = lpf.lpf_tar_or_q_w->update(tar_q.w());
       if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
         tar_pos_left(0) = lpf.lpf_tar_pos_x->update(tar_pos_left(0));
         tar_pos_left(1) = lpf.lpf_tar_pos_y->update(tar_pos_left(1));
         tar_pos_left(2) = lpf.lpf_tar_pos_z->update(tar_pos_left(2));
         tar_q_left.x() = lpf.lpf_tar_or_q_x->update(tar_q_left.x());
         tar_q_left.y() = lpf.lpf_tar_or_q_y->update(tar_q_left.y());
         tar_q_left.z() = lpf.lpf_tar_or_q_z->update(tar_q_left.z());
         tar_q_left.w() = lpf.lpf_tar_or_q_w->update(tar_q_left.w());
       }
    }
  }else if(mov_type==2 || mov_type==3 || mov_type==4){ // place
    posePtr tar;
    posePtr tar_left;

    //da para arrumar um pou
    tar = ctx.curr_scene->getPose(ctx.i_tar_ctrl);
    tar_pos(0) = tar->getPos().Xpos;
    tar_pos(1) = tar->getPos().Ypos;
    tar_pos(2) = tar->getPos().Zpos;
    tar_q = tar->getQuaternion();

    if ((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))) {
      tar_left = ctx.curr_scene->getPose(ctx.i_tar_ctrl_left);
      tar_pos_left(0) = tar_left->getPos().Xpos;
      tar_pos_left(1) = tar_left->getPos().Ypos;
      tar_pos_left(2) = tar_left->getPos().Zpos;
      tar_q_left = tar_left->getQuaternion();
    }

    if ((this->h_results != nullptr) || (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 1))) {
        // Single arm
        dist_app = ctx.approach_ctrl.at(3);
        vv_app << ctx.approach_ctrl.at(0),ctx.approach_ctrl.at(1),ctx.approach_ctrl.at(2);
        dist_ret = ctx.retreat_ctrl.at(3);
        vv_ret << ctx.retreat_ctrl.at(0),ctx.retreat_ctrl.at(1),ctx.retreat_ctrl.at(2);
    }
    else if ((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
        // Dual arm
        //Right
        vv_app << ctx.approach_ctrl.at(0),ctx.approach_ctrl.at(1),ctx.approach_ctrl.at(2);
        dist_app = ctx.approach_ctrl.at(3);
        vv_ret << ctx.retreat_ctrl.at(0),ctx.retreat_ctrl.at(1),ctx.retreat_ctrl.at(2);
        dist_ret = ctx.retreat_ctrl.at(3);
        //Left
        vv_app_left << ctx.approach_ctrl.at(4),ctx.approach_ctrl.at(5),ctx.approach_ctrl.at(6);
        dist_app_left = ctx.approach_ctrl.at(7);
        vv_ret_left << ctx.retreat_ctrl.at(4),ctx.retreat_ctrl.at(5),ctx.retreat_ctrl.at(6);
        dist_ret_left = ctx.retreat_ctrl.at(7);
    }


    if(ui.checkBox_tar_noise->isChecked() && usr_interface_input.sim_robot){
       tar_pos(0) = tar_pos(0) - (noise_sim_obj.obj_x_var/2) + noise_sim_obj.obj_x_var*(rand() / double(RAND_MAX));
       tar_pos(1) = tar_pos(1) - (noise_sim_obj.obj_y_var/2) + noise_sim_obj.obj_y_var*(rand() / double(RAND_MAX));
       tar_pos(2) = tar_pos(2) - (noise_sim_obj.obj_z_var/2) + noise_sim_obj.obj_z_var*(rand() / double(RAND_MAX));
       tar_q.x() = tar_q.x() - (noise_sim_obj.obj_q_x_var/2) + noise_sim_obj.obj_q_x_var*(rand() / double(RAND_MAX));
       tar_q.y() = tar_q.y() - (noise_sim_obj.obj_q_y_var/2) + noise_sim_obj.obj_q_y_var*(rand() / double(RAND_MAX));
       tar_q.z() = tar_q.z() - (noise_sim_obj.obj_q_z_var/2) + noise_sim_obj.obj_q_z_var*(rand() / double(RAND_MAX));
       tar_q.w() = tar_q.w() - (noise_sim_obj.obj_q_w_var/2) + noise_sim_obj.obj_q_w_var*(rand() / double(RAND_MAX));
       if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
         tar_pos_left(0) = tar_pos_left(0) - (noise_sim_obj.obj_x_var/2) + noise_sim_obj.obj_x_var*(rand() / double(RAND_MAX));
         tar_pos_left(1) = tar_pos_left(1) - (noise_sim_obj.obj_y_var/2) + noise_sim_obj.obj_y_var*(rand() / double(RAND_MAX));
         tar_pos_left(2) = tar_pos_left(2) - (noise_sim_obj.obj_z_var/2) + noise_sim_obj.obj_z_var*(rand() / double(RAND_MAX));
         tar_q_left.x() = tar_q_left.x() - (noise_sim_obj.obj_q_x_var/2) + noise_sim_obj.obj_q_x_var*(rand() / double(RAND_MAX));
         tar_q_left.y() = tar_q_left.y() - (noise_sim_obj.obj_q_y_var/2) + noise_sim_obj.obj_q_y_var*(rand() / double(RAND_MAX));
         tar_q_left.z() = tar_q_left.z() - (noise_sim_obj.obj_q_z_var/2) + noise_sim_obj.obj_q_z_var*(rand() / double(RAND_MAX));
         tar_q_left.w() = tar_q_left.w() - (noise_sim_obj.obj_q_w_var/2) + noise_sim_obj.obj_q_w_var*(rand() / double(RAND_MAX));
       }
    }
    if(ui.checkBox_tar_filter_noise->isChecked()){
       tar_pos(0) = lpf.lpf_tar_pos_x->update(tar_pos(0));
       tar_pos(1) = lpf.lpf_tar_pos_y->update(tar_pos(1));
       tar_pos(2) = lpf.lpf_tar_pos_z->update(tar_pos(2));
       tar_q.x() = lpf.lpf_tar_or_q_x->update(tar_q.x());
       tar_q.y() = lpf.lpf_tar_or_q_y->update(tar_q.y());
       tar_q.z() = lpf.lpf_tar_or_q_z->update(tar_q.z());
       tar_q.w() = lpf.lpf_tar_or_q_w->update(tar_q.w());
       if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
         tar_pos_left(0) = lpf.lpf_tar_pos_x->update(tar_pos_left(0));
         tar_pos_left(1) = lpf.lpf_tar_pos_y->update(tar_pos_left(1));
         tar_pos_left(2) = lpf.lpf_tar_pos_z->update(tar_pos_left(2));
         tar_q_left.x() = lpf.lpf_tar_or_q_x->update(tar_q_left.x());
         tar_q_left.y() = lpf.lpf_tar_or_q_y->update(tar_q_left.y());
         tar_q_left.z() = lpf.lpf_tar_or_q_z->update(tar_q_left.z());
         tar_q_left.w() = lpf.lpf_tar_or_q_w->update(tar_q_left.w());
       }
    }
  }
//-------------------------------------------------------------------------------------


//-------------- 2- Robot info update --------------------------------------------------
  Matrix3d Rot_tar = tar_q.toRotationMatrix();
  Vector3d zt_vec = Rot_tar.col(2);
  Vector3d vv_app_w = Rot_tar*vv_app;
  Vector3d vv_ret_w = Rot_tar*vv_ret;

  Matrix3d Rot_tar_left = tar_q_left.toRotationMatrix();
  Vector3d zt_vec_left = Rot_tar_left.col(2);
  Vector3d vv_app_w_left = Rot_tar_left*vv_app_left;
  Vector3d vv_ret_w_left = Rot_tar_left*vv_ret_left;

  vector<double> timesteps;

  JointState.hand_h_positions = ctx.handPosition_mov_stages.at(ctx.i_ctrl);
  JointState.hand_h_orientations = ctx.handOrientation_mov_stages.at(ctx.i_ctrl);
  JointState.hand_h_orientations_q = ctx.handOrientation_q_mov_stages.at(ctx.i_ctrl);
  JointState.hand_h_lin_velocities = ctx.handLinearVelocity_mov_stages.at(ctx.i_ctrl);
  JointState.hand_h_ang_velocities = ctx.handAngularVelocity_mov_stages.at(ctx.i_ctrl);
  JointState.hand_h_lin_accelerations = ctx.handLinearAcceleration_mov_stages.at(ctx.i_ctrl);
  JointState.hand_h_ang_accelerations = ctx.handAngularAcceleration_mov_stages.at(ctx.i_ctrl);
  JointState.alpha_positions = ctx.swivel_angle_mov_stages.at(ctx.i_ctrl);
  JointState.alpha_velocities = ctx.der_swivel_angle_mov_stages.at(ctx.i_ctrl);
  JointState.alpha_accelerations = ctx.der_der_swivel_angle_mov_stages.at(ctx.i_ctrl);

  if ((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
    //left
    JointState.hand_h_positions_left = ctx.handPosition_mov_stages_left.at(ctx.i_ctrl);
    JointState.hand_h_orientations_left = ctx.handOrientation_mov_stages_left.at(ctx.i_ctrl);
    JointState.hand_h_orientations_q_left = ctx.handOrientation_q_mov_stages_left.at(ctx.i_ctrl);
    JointState.hand_h_lin_velocities_left = ctx.handLinearVelocity_mov_stages_left.at(ctx.i_ctrl);
    JointState.hand_h_ang_velocities_left = ctx.handAngularVelocity_mov_stages_left.at(ctx.i_ctrl);
    JointState.hand_h_lin_accelerations_left = ctx.handLinearAcceleration_mov_stages_left.at(ctx.i_ctrl);
    JointState.hand_h_ang_accelerations_left = ctx.handAngularAcceleration_mov_stages_left.at(ctx.i_ctrl);
    JointState.alpha_positions_left = ctx.swivel_angle_mov_stages_left.at(ctx.i_ctrl);
    JointState.alpha_velocities_left = ctx.der_swivel_angle_mov_stages_left.at(ctx.i_ctrl);
    JointState.alpha_accelerations_left = ctx.der_der_swivel_angle_mov_stages_left.at(ctx.i_ctrl);
  }

  timesteps = ctx.timesteps_mov.at(ctx.i_ctrl);
  JointState.period_T = std::accumulate(timesteps.begin(), timesteps.end(), 0.0);

  //estudar se necessito de alterar n steps!!!!!!!!!!!!
  JointState.n_steps = JointState.hand_h_positions.size();
//--------------------------------------------------------------------------------------


//---------------- 3- Movement fase (Start position) -----------------------------------
  // initial pose of the human-like hand
  JointState.h_hand_pos_init.resize(3); Vector3d hand_pos_tmp;
  JointState.h_hand_pos_init_left.resize(3); Vector3d hand_pos_tmp_left;

  if(stage_descr.compare("plan")==0){
    //extrai a primeira posição e orientações do movimento planeado
    vector<double> h_vec = JointState.hand_h_positions.at(0);
    hand_pos_tmp(0) = h_vec.at(0);
    hand_pos_tmp(1) = h_vec.at(1);
    hand_pos_tmp(2) = h_vec.at(2);

    JointState.h_hand_or_init = JointState.hand_h_orientations.at(0); // initial human-like hand orientation (RPY)
    JointState.h_hand_or_q_init = JointState.hand_h_orientations_q.at(0); // initial human-like hand orientation (quaternion)

    if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
      vector<double> h_vec_left = JointState.hand_h_positions_left.at(0);
      hand_pos_tmp_left(0) = h_vec_left.at(0);
      hand_pos_tmp_left(1) = h_vec_left.at(1);
      hand_pos_tmp_left(2) = h_vec_left.at(2);

      JointState.h_hand_or_init_left = JointState.hand_h_orientations_left.at(0); // initial human-like hand orientation (RPY)
      JointState.h_hand_or_q_init_left = JointState.hand_h_orientations_q_left.at(0); // initial human-like hand orientation (quaternion)

    }

  }else if(stage_descr.compare("approach")==0){
    if(usr_interface_input.sim_robot){
      hand_pos_tmp = tar_pos - ctx.dHO_ctrl*zt_vec + dist_app*vv_app_w;
      Vector3d tar_rpy = tar_q.toRotationMatrix().eulerAngles(0,1,2);
      JointState.h_hand_or_init.at(0) = tar_rpy(0);
      JointState.h_hand_or_init.at(1) = tar_rpy(1);
      JointState.h_hand_or_init.at(2) = tar_rpy(2);

      JointState.h_hand_or_q_init.at(0) = tar_q.x();
      JointState.h_hand_or_q_init.at(1) = tar_q.y();
      JointState.h_hand_or_q_init.at(2) = tar_q.z();
      JointState.h_hand_or_q_init.at(3) = tar_q.w();

      if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
        //resolver
        hand_pos_tmp_left = tar_pos_left - ctx.dHO_ctrl_left*zt_vec_left + dist_app_left*vv_app_w_left;
        Vector3d tar_rpy_left = tar_q_left.toRotationMatrix().eulerAngles(0,1,2);

        JointState.h_hand_or_init_left.at(0) = tar_rpy_left(0);
        JointState.h_hand_or_init_left.at(1) = tar_rpy_left(1);
        JointState.h_hand_or_init_left.at(2) = tar_rpy_left(2);

        JointState.h_hand_or_q_init_left.at(0) = tar_q_left.x();
        JointState.h_hand_or_q_init_left.at(1) = tar_q_left.y();
        JointState.h_hand_or_q_init_left.at(2) = tar_q_left.z();
        JointState.h_hand_or_q_init_left.at(3) = tar_q_left.w();
      }

    }else{
      if(mov_type==0){ // pick
        Rot_tar = mes_pose_str.tar_rec->getQuaternion().toRotationMatrix();
        Vector3d tar_rpy = Rot_tar.eulerAngles(0,1,2);
        zt_vec = Rot_tar.col(2); vv_app_w = Rot_tar*vv_app; vv_ret_w = Rot_tar*vv_ret;
        mes_pose_str.tar_rec_pos(0) = mes_pose_str.tar_rec->getPos().Xpos;
        mes_pose_str.tar_rec_pos(1) = mes_pose_str.tar_rec->getPos().Ypos;
        mes_pose_str.tar_rec_pos(2) = mes_pose_str.tar_rec->getPos().Zpos;
        hand_pos_tmp = mes_pose_str.tar_rec_pos - ctx.dHO_ctrl*zt_vec + dist_app*vv_app_w;

        JointState.h_hand_or_init.at(0) = tar_rpy(0);
        JointState.h_hand_or_init.at(1) = tar_rpy(1);
        JointState.h_hand_or_init.at(2) = tar_rpy(2);
        JointState.h_hand_or_q_init.at(0) = mes_pose_str.tar_rec->getQuaternion().x();
        JointState.h_hand_or_q_init.at(1) = mes_pose_str.tar_rec->getQuaternion().y();
        JointState.h_hand_or_q_init.at(2) = mes_pose_str.tar_rec->getQuaternion().z();
        JointState.h_hand_or_q_init.at(3) = mes_pose_str.tar_rec->getQuaternion().w();

        if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
          Rot_tar_left = mes_pose_str.tar_rec->getQuaternion().toRotationMatrix();//tratar!!!
          Vector3d tar_rpy_left = Rot_tar_left.eulerAngles(0,1,2);
          zt_vec_left = Rot_tar_left.col(2); vv_app_w_left = Rot_tar_left*vv_app_left; vv_ret_w_left = Rot_tar_left*vv_ret_left;
          mes_pose_str.tar_rec_pos(0) = mes_pose_str.tar_rec->getPos().Xpos;
          mes_pose_str.tar_rec_pos(1) = mes_pose_str.tar_rec->getPos().Ypos;
          mes_pose_str.tar_rec_pos(2) = mes_pose_str.tar_rec->getPos().Zpos;
          hand_pos_tmp = mes_pose_str.tar_rec_pos - ctx.dHO_ctrl*zt_vec + dist_app*vv_app_w;

          JointState.h_hand_or_init_left.at(0) = tar_rpy(0);
          JointState.h_hand_or_init_left.at(1) = tar_rpy(1);
          JointState.h_hand_or_init_left.at(2) = tar_rpy(2);
          JointState.h_hand_or_q_init_left.at(0) = mes_pose_str.tar_rec->getQuaternion().x();
          JointState.h_hand_or_q_init_left.at(1) = mes_pose_str.tar_rec->getQuaternion().y();
          JointState.h_hand_or_q_init_left.at(2) = mes_pose_str.tar_rec->getQuaternion().z();
          JointState.h_hand_or_q_init_left.at(3) = mes_pose_str.tar_rec->getQuaternion().w();
        }
      }else{
        hand_pos_tmp = tar_pos - ctx.dHO_ctrl*zt_vec + dist_app*vv_app_w;
        Vector3d tar_rpy = tar_q.toRotationMatrix().eulerAngles(0,1,2);
        JointState.h_hand_or_init.at(0) = tar_rpy(0);
        JointState.h_hand_or_init.at(1) = tar_rpy(1);
        JointState.h_hand_or_init.at(2) = tar_rpy(2);
        JointState.h_hand_or_q_init.at(0) = tar_q.x();
        JointState.h_hand_or_q_init.at(1) = tar_q.y();
        JointState.h_hand_or_q_init.at(2) = tar_q.z();
        JointState.h_hand_or_q_init.at(3) = tar_q.w();
        if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
          hand_pos_tmp_left = tar_pos_left - ctx.dHO_ctrl_left*zt_vec_left + dist_app_left*vv_app_w_left;
          Vector3d tar_rpy_left = tar_q_left.toRotationMatrix().eulerAngles(0,1,2);
          JointState.h_hand_or_init_left.at(0) = tar_rpy_left(0);
          JointState.h_hand_or_init_left.at(1) = tar_rpy_left(1);
          JointState.h_hand_or_init_left.at(2) = tar_rpy_left(2);
          JointState.h_hand_or_q_init_left.at(0) = tar_q_left.x();
          JointState.h_hand_or_q_init_left.at(1) = tar_q_left.y();
          JointState.h_hand_or_q_init_left.at(2) = tar_q_left.z();
          JointState.h_hand_or_q_init_left.at(3) = tar_q_left.w();
        }
      }

    }
  }else if(stage_descr.compare("retreat")==0){
    if(usr_interface_input.sim_robot){
      hand_pos_tmp = tar_pos - ctx.dHO_ctrl*zt_vec;
      Vector3d tar_rpy = tar_q.toRotationMatrix().eulerAngles(0,1,2);
      JointState.h_hand_or_init.at(0) = tar_rpy(0);
      JointState.h_hand_or_init.at(1) = tar_rpy(1);
      JointState.h_hand_or_init.at(2) = tar_rpy(2);
      JointState.h_hand_or_q_init.at(0) = tar_q.x();
      JointState.h_hand_or_q_init.at(1) = tar_q.y();
      JointState.h_hand_or_q_init.at(2) = tar_q.z();
      JointState.h_hand_or_q_init.at(3) = tar_q.w();
    }else{
      if(mov_type==0){ // pick
        Rot_tar = mes_pose_str.tar_rec->getQuaternion().toRotationMatrix();
        Vector3d tar_rpy = Rot_tar.eulerAngles(0,1,2);
        zt_vec = Rot_tar.col(2); vv_app_w = Rot_tar*vv_app; vv_ret_w = Rot_tar*vv_ret;
        mes_pose_str.tar_rec_pos(0) = mes_pose_str.tar_rec->getPos().Xpos;
        mes_pose_str.tar_rec_pos(1) = mes_pose_str.tar_rec->getPos().Ypos;
        mes_pose_str.tar_rec_pos(2) = mes_pose_str.tar_rec->getPos().Zpos;
        hand_pos_tmp = mes_pose_str.tar_rec_pos - ctx.dHO_ctrl*zt_vec;

        JointState.h_hand_or_init.at(0) = tar_rpy(0);
        JointState.h_hand_or_init.at(1) = tar_rpy(1);
        JointState.h_hand_or_init.at(2) = tar_rpy(2);
        JointState.h_hand_or_q_init.at(0) = mes_pose_str.tar_rec->getQuaternion().x();
        JointState.h_hand_or_q_init.at(1) = mes_pose_str.tar_rec->getQuaternion().y();
        JointState.h_hand_or_q_init.at(2) = mes_pose_str.tar_rec->getQuaternion().z();
        JointState.h_hand_or_q_init.at(3) = mes_pose_str.tar_rec->getQuaternion().w();

        if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
          Rot_tar_left = mes_pose_str.tar_rec->getQuaternion().toRotationMatrix();//tratar!!!
          Vector3d tar_rpy_left = Rot_tar_left.eulerAngles(0,1,2);
          zt_vec_left = Rot_tar_left.col(2); vv_app_w_left = Rot_tar_left*vv_app_left; vv_ret_w_left = Rot_tar_left*vv_ret_left;
          mes_pose_str.tar_rec_pos(0) = mes_pose_str.tar_rec->getPos().Xpos;
          mes_pose_str.tar_rec_pos(1) = mes_pose_str.tar_rec->getPos().Ypos;
          mes_pose_str.tar_rec_pos(2) = mes_pose_str.tar_rec->getPos().Zpos;
          hand_pos_tmp = mes_pose_str.tar_rec_pos - ctx.dHO_ctrl*zt_vec + dist_app*vv_app_w;

          JointState.h_hand_or_init_left.at(0) = tar_rpy(0);
          JointState.h_hand_or_init_left.at(1) = tar_rpy(1);
          JointState.h_hand_or_init_left.at(2) = tar_rpy(2);
          JointState.h_hand_or_q_init_left.at(0) = mes_pose_str.tar_rec->getQuaternion().x();
          JointState.h_hand_or_q_init_left.at(1) = mes_pose_str.tar_rec->getQuaternion().y();
          JointState.h_hand_or_q_init_left.at(2) = mes_pose_str.tar_rec->getQuaternion().z();
          JointState.h_hand_or_q_init_left.at(3) = mes_pose_str.tar_rec->getQuaternion().w();
        }
      }else{
        hand_pos_tmp = tar_pos - ctx.dHO_ctrl*zt_vec;
        Vector3d tar_rpy = tar_q.toRotationMatrix().eulerAngles(0,1,2);
        JointState.h_hand_or_init.at(0) = tar_rpy(0);
        JointState.h_hand_or_init.at(1) = tar_rpy(1);
        JointState.h_hand_or_init.at(2) = tar_rpy(2);
        JointState.h_hand_or_q_init.at(0) = tar_q.x();
        JointState.h_hand_or_q_init.at(1) = tar_q.y();
        JointState.h_hand_or_q_init.at(2) = tar_q.z();
        JointState.h_hand_or_q_init.at(3) = tar_q.w();
        if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
          hand_pos_tmp_left = tar_pos_left - ctx.dHO_ctrl_left*zt_vec_left;
          Vector3d tar_rpy_left = tar_q_left.toRotationMatrix().eulerAngles(0,1,2);
          JointState.h_hand_or_init_left.at(0) = tar_rpy_left(0);
          JointState.h_hand_or_init_left.at(1) = tar_rpy_left(1);
          JointState.h_hand_or_init_left.at(2) = tar_rpy_left(2);
          JointState.h_hand_or_q_init_left.at(0) = tar_q_left.x();
          JointState.h_hand_or_q_init_left.at(1) = tar_q_left.y();
          JointState.h_hand_or_q_init_left.at(2) = tar_q_left.z();
          JointState.h_hand_or_q_init_left.at(3) = tar_q_left.w();
        }
      }
    }
  }


  JointState.h_hand_pos_init.at(0) = hand_pos_tmp(0);
  JointState.h_hand_pos_init.at(1) = hand_pos_tmp(1);
  JointState.h_hand_pos_init.at(2) = hand_pos_tmp(2);

  // initial orientation of the human-like hand
  JointState.h_hand_or_init_vec << JointState.h_hand_or_init.at(0), JointState.h_hand_or_init.at(1), JointState.h_hand_or_init.at(2);
  JointState.h_hand_or_q_e_init << JointState.h_hand_or_q_init.at(0), JointState.h_hand_or_q_init.at(1), JointState.h_hand_or_q_init.at(2);
  JointState.h_hand_or_q_w_init = JointState.h_hand_or_q_init.at(3);


  if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
    JointState.h_hand_pos_init_left.at(0) = hand_pos_tmp_left(0);
    JointState.h_hand_pos_init_left.at(1) = hand_pos_tmp_left(1);
    JointState.h_hand_pos_init_left.at(2) = hand_pos_tmp_left(2);

    JointState.h_hand_or_init_vec_left << JointState.h_hand_or_init_left.at(0), JointState.h_hand_or_init_left.at(1), JointState.h_hand_or_init_left.at(2);
    JointState.h_hand_or_q_e_init_left << JointState.h_hand_or_q_init_left.at(0), JointState.h_hand_or_q_init_left.at(1), JointState.h_hand_or_q_init_left.at(2);
    JointState.h_hand_or_q_w_init_left = JointState.h_hand_or_q_init_left.at(3);

  }

  //---------------------- Movement fase (end position) --------------------------
  // end hand pose
  // position  //esta parte ficou a meio!! tar_rec?
  Vector3d hand_pos_vec;
  Vector3d hand_pos_vec_left;
  if(stage_descr.compare("plan")==0){
    if(stages>1){
      string stage_succ;

      //single arm move
      if(!(ui.radioButton_ctrl_task->isChecked()) && h_results != nullptr){
        stage_succ = this->h_results->trajectory_descriptions.at(ctx.i_ctrl+1);
      }
      //single arm task
      else if(ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 1)){
        stage_succ = this->trajectory_descriptions.at(ctx.i_ctrl+1);
      }
      //dual arm move
      else if(!(ui.radioButton_ctrl_task->isChecked()) && h_dual_results != nullptr){
        stage_succ = this->h_dual_results->trajectory_descriptions.at(ctx.i_ctrl+1);
      }
      //dual arm task
      else if (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2)){
        stage_succ = this->trajectory_descriptions.at(ctx.i_ctrl+1);
      }

      if(stage_succ.compare("approach")==0){
          if(usr_interface_input.sim_robot){
              hand_pos_vec = tar_pos - ctx.dHO_ctrl*zt_vec + dist_app*vv_app_w;
              if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
                hand_pos_vec_left = tar_pos_left - ctx.dHO_ctrl_left*zt_vec_left + dist_app_left*vv_app_w_left;
              }
          }else{
            if(mov_type==0){//pick
                mes_pose_str.tar_rec_pos(0) = mes_pose_str.tar_rec->getPos().Xpos;
                mes_pose_str.tar_rec_pos(1) = mes_pose_str.tar_rec->getPos().Ypos;
                mes_pose_str.tar_rec_pos(2) = mes_pose_str.tar_rec->getPos().Zpos;
                hand_pos_vec = mes_pose_str.tar_rec_pos - ctx.dHO_ctrl*zt_vec + dist_app*vv_app_w;
                if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
                  mes_pose_str.tar_rec_pos_left(0) = mes_pose_str.tar_rec->getPos().Xpos;
                  mes_pose_str.tar_rec_pos_left(1) = mes_pose_str.tar_rec->getPos().Ypos;
                  mes_pose_str.tar_rec_pos_left(2) = mes_pose_str.tar_rec->getPos().Zpos;
                  hand_pos_vec_left = mes_pose_str.tar_rec_pos_left - ctx.dHO_ctrl_left*zt_vec_left + dist_app_left*vv_app_w_left;
                }
            }else{
                hand_pos_vec = tar_pos - ctx.dHO_ctrl*zt_vec + dist_app*vv_app_w;
                if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
                  hand_pos_vec_left = tar_pos_left - ctx.dHO_ctrl_left*zt_vec_left + dist_app_left*vv_app_w_left;
                }
            }
          }
      }else{
        hand_pos_vec = tar_pos - ctx.dHO_ctrl*zt_vec;
        if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
          hand_pos_vec_left = tar_pos_left - ctx.dHO_ctrl_left*zt_vec_left;
        }
      }
    }else{
      hand_pos_vec = tar_pos - ctx.dHO_ctrl*zt_vec;
      if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
        hand_pos_vec_left = tar_pos_left - ctx.dHO_ctrl_left*zt_vec_left;
      }
    }
  }else if(stage_descr.compare("approach")==0){
    if(usr_interface_input.follow_tar){
      hand_pos_vec = tar_pos - ctx.dHO_ctrl*zt_vec + dist_app*vv_app_w;
      if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
        hand_pos_vec_left = tar_pos_left - ctx.dHO_ctrl_left*zt_vec_left + dist_app_left*vv_app_w_left;
      }
    }else{
      if(usr_interface_input.sim_robot){
          hand_pos_vec = tar_pos - ctx.dHO_ctrl*zt_vec;
          if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
            hand_pos_vec_left = tar_pos_left - ctx.dHO_ctrl_left*zt_vec_left;
          }
      }else{
         if(mov_type==0){//pick
             Rot_tar = mes_pose_str.tar_rec->getQuaternion().toRotationMatrix();
             zt_vec = Rot_tar.col(2); vv_app_w = Rot_tar*vv_app; vv_ret_w = Rot_tar*vv_ret;
             mes_pose_str.tar_rec_pos(0) = mes_pose_str.tar_rec->getPos().Xpos;
             mes_pose_str.tar_rec_pos(1) = mes_pose_str.tar_rec->getPos().Ypos;
             mes_pose_str.tar_rec_pos(2) = mes_pose_str.tar_rec->getPos().Zpos;
             hand_pos_vec = mes_pose_str.tar_rec_pos - ctx.dHO_ctrl*zt_vec;
             if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
               Rot_tar_left = mes_pose_str.tar_rec->getQuaternion().toRotationMatrix();
               zt_vec_left = Rot_tar_left.col(2); vv_app_w_left = Rot_tar_left*vv_app_left; vv_ret_w_left = Rot_tar_left*vv_ret_left;
               mes_pose_str.tar_rec_pos_left(0) = mes_pose_str.tar_rec->getPos().Xpos; //passar o VAR
               mes_pose_str.tar_rec_pos_left(1) = mes_pose_str.tar_rec->getPos().Ypos;
               mes_pose_str.tar_rec_pos_left(2) = mes_pose_str.tar_rec->getPos().Zpos;
               hand_pos_vec_left = mes_pose_str.tar_rec_pos_left - ctx.dHO_ctrl_left*zt_vec_left;
             }
         }else{
            hand_pos_vec = tar_pos - ctx.dHO_ctrl*zt_vec;
            if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
              hand_pos_vec_left = tar_pos_left - ctx.dHO_ctrl_left*zt_vec_left;
            }
         }
      }
    }
  }else if(stage_descr.compare("retreat")==0){
    if(usr_interface_input.sim_robot){
      hand_pos_vec = tar_pos - ctx.dHO_ctrl*zt_vec + dist_ret*vv_ret_w;
      if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
        hand_pos_vec_left = tar_pos_left - ctx.dHO_ctrl_left*zt_vec_left + dist_ret_left*vv_ret_w_left;
      }
    }else{
      if(mov_type==0){//pick
        Rot_tar = mes_pose_str.tar_rec->getQuaternion().toRotationMatrix();
        zt_vec = Rot_tar.col(2); vv_app_w = Rot_tar*vv_app; vv_ret_w = Rot_tar*vv_ret;
        //Vector3d tar_rec_pos; ->coloquei no .hpp
        mes_pose_str.tar_rec_pos(0) = mes_pose_str.tar_rec->getPos().Xpos;
        mes_pose_str.tar_rec_pos(1) = mes_pose_str.tar_rec->getPos().Ypos;
        mes_pose_str.tar_rec_pos(2) = mes_pose_str.tar_rec->getPos().Zpos;
        hand_pos_vec = mes_pose_str.tar_rec_pos - ctx.dHO_ctrl*zt_vec + dist_ret*vv_ret_w;
        if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
          Rot_tar_left = mes_pose_str.tar_rec->getQuaternion().toRotationMatrix();
          zt_vec_left = Rot_tar_left.col(2); vv_app_w_left = Rot_tar_left*vv_app_left; vv_ret_w_left = Rot_tar_left*vv_ret_left;
          mes_pose_str.tar_rec_pos_left(0) = mes_pose_str.tar_rec->getPos().Xpos;
          mes_pose_str.tar_rec_pos_left(1) = mes_pose_str.tar_rec->getPos().Ypos;
          mes_pose_str.tar_rec_pos_left(2) = mes_pose_str.tar_rec->getPos().Zpos;
          hand_pos_vec_left = mes_pose_str.tar_rec_pos_left - ctx.dHO_ctrl_left*zt_vec_left + dist_ret_left*vv_ret_w_left;
        }
      }else{
        hand_pos_vec = tar_pos - ctx.dHO_ctrl*zt_vec + dist_ret*vv_ret_w;
        if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
          hand_pos_vec_left = tar_pos_left - ctx.dHO_ctrl_left*zt_vec_left + dist_ret_left*vv_ret_w_left;
        }
      }
    }
  }

//---------------------------------------------------------------------------------

  //Variables definitions

  Vector3d h_hand_ang_vel_init_vec;
  Vector3d h_hand_ang_vel_end_vec;
  Vector3d h_hand_ang_acc_init_vec;
  Vector3d h_hand_ang_acc_end_vec;
  Vector3d h_hand_ang_vel_init_vec_left;
  Vector3d h_hand_ang_vel_end_vec_left;
  Vector3d h_hand_ang_acc_init_vec_left;
  Vector3d h_hand_ang_acc_end_vec_left;


  // position
  JointState.h_hand_pos_end.resize(3);
  JointState.h_hand_pos_end.at(0) = hand_pos_vec(0);
  JointState.h_hand_pos_end.at(1) = hand_pos_vec(1);
  JointState.h_hand_pos_end.at(2) = hand_pos_vec(2);
  // orientation
  Vector3d end_rpy = Rot_tar.eulerAngles(0,1,2);
  JointState.h_hand_or_end.resize(3);
  JointState.h_hand_or_end.at(0) = end_rpy(0);
  JointState.h_hand_or_end.at(1) = end_rpy(1);
  JointState.h_hand_or_end.at(2) = end_rpy(2);
  Vector3d h_hand_or_end; h_hand_or_end << JointState.h_hand_or_end.at(0), JointState.h_hand_or_end.at(1), JointState.h_hand_or_end.at(2);
  Quaterniond h_hand_or_q_end_vec(Rot_tar);
  JointState.h_hand_or_q_end.resize(4);
  JointState.h_hand_or_q_end.at(0) = h_hand_or_q_end_vec.x();
  JointState.h_hand_or_q_end.at(1) = h_hand_or_q_end_vec.y();
  JointState.h_hand_or_q_end.at(2) = h_hand_or_q_end_vec.z();
  JointState.h_hand_or_q_end.at(3) = h_hand_or_q_end_vec.w();
  JointState.h_hand_or_q_e_end << JointState.h_hand_or_q_end.at(0), JointState.h_hand_or_q_end.at(1), JointState.h_hand_or_q_end.at(2);
  JointState.h_hand_or_q_w_end = JointState.h_hand_or_q_end.at(3);
  des_pose.hand_pos_vec_x(0) = JointState.h_hand_pos_end.at(0);
  des_pose.hand_pos_vec_x(1) = JointState.h_hand_pos_end.at(1);
  des_pose.hand_pos_vec_x(2) = JointState.h_hand_pos_end.at(2);
  des_pose.hand_pos_vec_x(3) = JointState.h_hand_or_q_end.at(0);
  des_pose.hand_pos_vec_x(4) = JointState.h_hand_or_q_end.at(1);
  des_pose.hand_pos_vec_x(5) = JointState.h_hand_or_q_end.at(2);
  des_pose.hand_pos_vec_x(6) = JointState.h_hand_or_q_end.at(3);

  JointState.h_hand_lin_vel_init = JointState.hand_h_lin_velocities.at(0); // initial human-like hand linear velocities
  JointState.h_hand_ang_vel_init = JointState.hand_h_ang_velocities.at(0); // initial human-like hand angular velocities
  JointState.h_hand_lin_acc_init = JointState.hand_h_lin_accelerations.at(0); // initial human-like hand linear accelerations
  JointState.h_hand_ang_acc_init = JointState.hand_h_ang_accelerations.at(0); // initial human-like hand angular accelerations

  JointState.h_hand_lin_vel_end = JointState.hand_h_lin_velocities.at(JointState.n_steps-1); // end human-like hand linear velocities
  JointState.h_hand_ang_vel_end = JointState.hand_h_ang_velocities.at(JointState.n_steps-1); // end human-like hand angular velocities
  JointState.h_hand_lin_acc_end = JointState.hand_h_lin_accelerations.at(JointState.n_steps-1); // end human-like hand linear accelerations
  JointState.h_hand_ang_acc_end = JointState.hand_h_ang_accelerations.at(JointState.n_steps-1); // end human-like hand angular accelerations

  h_hand_ang_vel_init_vec << JointState.h_hand_ang_vel_init.at(0), JointState.h_hand_ang_vel_init.at(1), JointState.h_hand_ang_vel_init.at(2);
  h_hand_ang_vel_end_vec << JointState.h_hand_ang_vel_end.at(0), JointState.h_hand_ang_vel_end.at(1), JointState.h_hand_ang_vel_end.at(2);
  h_hand_ang_acc_init_vec << JointState.h_hand_ang_acc_init.at(0), JointState.h_hand_ang_acc_init.at(1), JointState.h_hand_ang_acc_init.at(2);
  h_hand_ang_acc_end_vec << JointState.h_hand_ang_acc_end.at(0), JointState.h_hand_ang_acc_end.at(1), JointState.h_hand_ang_acc_end.at(2);

  //Quarternion propagation
  // initial angular velocity (quaternion)
  JointState.h_hand_ang_vel_q_e_init = 0.5*(JointState.h_hand_or_q_w_init*JointState.I_3*h_hand_ang_vel_init_vec-JointState.h_hand_or_q_e_init.cross(h_hand_ang_vel_init_vec));
  JointState.h_hand_ang_vel_q_w_init = -0.5*(JointState.h_hand_or_q_e_init.dot(h_hand_ang_vel_init_vec));

  // end angular velocity (quaternion)
  JointState.h_hand_ang_vel_q_e_end = 0.5*(JointState.h_hand_or_q_w_end*JointState.I_3*h_hand_ang_vel_end_vec-JointState.h_hand_or_q_e_end.cross(h_hand_ang_vel_end_vec));
  JointState.h_hand_ang_vel_q_w_end = -0.5*(JointState.h_hand_or_q_e_end.dot(h_hand_ang_vel_end_vec));

  // initial angular acceleration (quaternion)
  JointState.h_hand_ang_acc_q_e_init = 0.5*JointState.I_3*(JointState.h_hand_ang_vel_q_w_init*h_hand_ang_vel_init_vec+JointState.h_hand_or_q_w_init*h_hand_ang_acc_init_vec)
                -0.5*(JointState.h_hand_ang_vel_q_e_init.cross(h_hand_ang_vel_init_vec)+JointState.h_hand_or_q_e_init.cross(h_hand_ang_acc_init_vec));
  JointState.h_hand_ang_acc_q_w_init = -0.5*(JointState.h_hand_ang_vel_q_e_init.dot(h_hand_ang_vel_init_vec)+JointState.h_hand_or_q_e_init.dot(h_hand_ang_acc_init_vec));

  // end angular acceleration (quaternion)
  JointState.h_hand_ang_acc_q_e_end = 0.5*JointState.I_3*(JointState.h_hand_ang_vel_q_w_end*h_hand_ang_vel_end_vec+JointState.h_hand_or_q_w_end*h_hand_ang_acc_end_vec)
                -0.5*(JointState.h_hand_ang_vel_q_e_end.cross(h_hand_ang_vel_end_vec)+JointState.h_hand_or_q_e_end.cross(h_hand_ang_acc_end_vec));
  JointState.h_hand_ang_acc_q_w_end = -0.5*(JointState.h_hand_ang_vel_q_e_end.dot(h_hand_ang_vel_end_vec)+JointState.h_hand_or_q_e_end.dot(h_hand_ang_acc_end_vec));


  Eigen::MatrixXd jointsPosition;
  Eigen::MatrixXd jointsVelocity;
  Eigen::MatrixXd jointsAcceleration;

  //arm selection ctrl~
  //right
  jointsPosition = ctx.jointsPosition_mov_ctrl.at(ctx.i_ctrl);
  jointsVelocity = ctx.jointsVelocity_mov_ctrl.at(ctx.i_ctrl);
  jointsAcceleration = ctx.jointsAcceleration_mov_ctrl.at(ctx.i_ctrl);

  //position
  JointState.jointsInitPosition_hand = jointsPosition.row(0).tail<JOINTS_HAND>();
  VectorXd::Map(&JointState.jointsInitPosition_hand_vec[0], JointState.jointsInitPosition_hand.size()) = JointState.jointsInitPosition_hand;
  JointState.jointsFinalPosition_hand = jointsPosition.row(jointsPosition.rows()-1).tail<JOINTS_HAND>();
  VectorXd::Map(&JointState.jointsFinalPosition_hand_vec[0], JointState.jointsFinalPosition_hand.size()) = JointState.jointsFinalPosition_hand;
  //velocity
  JointState.jointsInitVelocity_hand = jointsVelocity.row(0).tail<JOINTS_HAND>();
  VectorXd::Map(&JointState.jointsInitVelocity_hand_vec[0], JointState.jointsInitVelocity_hand.size()) = JointState.jointsInitVelocity_hand;
  JointState.jointsFinalVelocity_hand = jointsVelocity.row(jointsVelocity.rows()-1).tail<JOINTS_HAND>();
  VectorXd::Map(&JointState.jointsFinalVelocity_hand_vec[0], JointState.jointsFinalVelocity_hand.size()) = JointState.jointsFinalVelocity_hand;
  //acceleration
  JointState.jointsInitAcceleration_hand = jointsAcceleration.row(0).tail<JOINTS_HAND>();
  VectorXd::Map(&JointState.jointsInitAcceleration_hand_vec[0], JointState.jointsInitAcceleration_hand.size()) = JointState.jointsInitAcceleration_hand;
  JointState.jointsFinalAcceleration_hand = jointsAcceleration.row(jointsAcceleration.rows()-1).tail<JOINTS_HAND>();
  VectorXd::Map(&JointState.jointsFinalAcceleration_hand_vec[0], JointState.jointsFinalAcceleration_hand.size()) = JointState.jointsFinalAcceleration_hand;

  des_pose.des_hand_pos_x = JointState.h_hand_pos_end.at(0);
  des_pose.des_hand_pos_y = JointState.h_hand_pos_end.at(1);
  des_pose.des_hand_pos_z = JointState.h_hand_pos_end.at(2);
  des_pose.des_hand_or_x = JointState.h_hand_or_end.at(0);
  des_pose.des_hand_or_y = JointState.h_hand_or_end.at(1);
  des_pose.des_hand_or_z = JointState.h_hand_or_end.at(2);
  des_pose.des_hand_or_q_x = JointState.h_hand_or_q_end.at(0);
  des_pose.des_hand_or_q_y = JointState.h_hand_or_q_end.at(1);
  des_pose.des_hand_or_q_z = JointState.h_hand_or_q_end.at(2);
  des_pose.des_hand_or_q_w = JointState.h_hand_or_q_end.at(3);

  if ((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
    // position
    JointState.h_hand_pos_end_left.resize(3);
    JointState.h_hand_pos_end_left.at(0) = hand_pos_vec_left(0);
    JointState.h_hand_pos_end_left.at(1) = hand_pos_vec_left(1);
    JointState.h_hand_pos_end_left.at(2) = hand_pos_vec_left(2);
    // orientation
    Vector3d end_rpy_left = Rot_tar_left.eulerAngles(0,1,2);
    JointState.h_hand_or_end_left.resize(3);
    JointState.h_hand_or_end_left.at(0) = end_rpy_left(0);
    JointState.h_hand_or_end_left.at(1) = end_rpy_left(1);
    JointState.h_hand_or_end_left.at(2) = end_rpy_left(2);
    Vector3d h_hand_or_end_left;
    h_hand_or_end_left << JointState.h_hand_or_end_left.at(0), JointState.h_hand_or_end_left.at(1), JointState.h_hand_or_end_left.at(2);
    Quaterniond h_hand_or_q_end_vec_left(Rot_tar_left);
    JointState.h_hand_or_q_end_left.resize(4);
    JointState.h_hand_or_q_end_left.at(0) = h_hand_or_q_end_vec_left.x();
    JointState.h_hand_or_q_end_left.at(1) = h_hand_or_q_end_vec_left.y();
    JointState.h_hand_or_q_end_left.at(2) = h_hand_or_q_end_vec_left.z();
    JointState.h_hand_or_q_end_left.at(3) = h_hand_or_q_end_vec_left.w();
    JointState.h_hand_or_q_e_end_left << JointState.h_hand_or_q_end_left.at(0), JointState.h_hand_or_q_end_left.at(1), JointState.h_hand_or_q_end_left.at(2);
    JointState.h_hand_or_q_w_end_left = JointState.h_hand_or_q_end_left.at(3);
    des_pose.hand_pos_vec_x_left(0) = JointState.h_hand_pos_end_left.at(0);
    des_pose.hand_pos_vec_x_left(1) = JointState.h_hand_pos_end_left.at(1);
    des_pose.hand_pos_vec_x_left(2) = JointState.h_hand_pos_end_left.at(2);
    des_pose.hand_pos_vec_x_left(3) = JointState.h_hand_or_q_end_left.at(0);
    des_pose.hand_pos_vec_x_left(4) = JointState.h_hand_or_q_end_left.at(1);
    des_pose.hand_pos_vec_x_left(5) = JointState.h_hand_or_q_end_left.at(2);
    des_pose.hand_pos_vec_x_left(6) = JointState.h_hand_or_q_end_left.at(3);

    JointState.h_hand_lin_vel_init_left = JointState.hand_h_lin_velocities_left.at(0); // initial human-like hand linear velocities
    JointState.h_hand_ang_vel_init_left = JointState.hand_h_ang_velocities_left.at(0); // initial human-like hand angular velocities
    JointState.h_hand_lin_acc_init_left = JointState.hand_h_lin_accelerations_left.at(0); // initial human-like hand linear accelerations
    JointState.h_hand_ang_acc_init_left = JointState.hand_h_ang_accelerations_left.at(0); // initial human-like hand angular accelerations

    JointState.h_hand_lin_vel_end_left = JointState.hand_h_lin_velocities_left.at(JointState.n_steps-1); // end human-like hand linear velocities
    JointState.h_hand_ang_vel_end_left = JointState.hand_h_ang_velocities_left.at(JointState.n_steps-1); // end human-like hand angular velocities
    JointState.h_hand_lin_acc_end_left = JointState.hand_h_lin_accelerations_left.at(JointState.n_steps-1); // end human-like hand linear accelerations
    JointState.h_hand_ang_acc_end_left = JointState.hand_h_ang_accelerations_left.at(JointState.n_steps-1); // end human-like hand angular accelerations

    h_hand_ang_vel_init_vec_left << JointState.h_hand_ang_vel_init_left.at(0), JointState.h_hand_ang_vel_init_left.at(1), JointState.h_hand_ang_vel_init_left.at(2);
    h_hand_ang_vel_end_vec_left << JointState.h_hand_ang_vel_end_left.at(0), JointState.h_hand_ang_vel_end_left.at(1), JointState.h_hand_ang_vel_end_left.at(2);
    h_hand_ang_acc_init_vec_left << JointState.h_hand_ang_acc_init_left.at(0), JointState.h_hand_ang_acc_init_left.at(1), JointState.h_hand_ang_acc_init_left.at(2);
    h_hand_ang_acc_end_vec_left << JointState.h_hand_ang_acc_end_left.at(0), JointState.h_hand_ang_acc_end_left.at(1), JointState.h_hand_ang_acc_end_left.at(2);

    //Quarternion propagation
    // initial angular velocity (quaternion)
    JointState.h_hand_ang_vel_q_e_init_left = 0.5*(JointState.h_hand_or_q_w_init_left*JointState.I_3*h_hand_ang_vel_init_vec_left-JointState.h_hand_or_q_e_init_left.cross(h_hand_ang_vel_init_vec_left));
    JointState.h_hand_ang_vel_q_w_init_left = -0.5*(JointState.h_hand_or_q_e_init_left.dot(h_hand_ang_vel_init_vec_left));

    // end angular velocity (quaternion)
    JointState.h_hand_ang_vel_q_e_end_left = 0.5*(JointState.h_hand_or_q_w_end_left*JointState.I_3*h_hand_ang_vel_end_vec_left-JointState.h_hand_or_q_e_end_left.cross(h_hand_ang_vel_end_vec_left));
    JointState.h_hand_ang_vel_q_w_end_left = -0.5*(JointState.h_hand_or_q_e_end_left.dot(h_hand_ang_vel_end_vec_left));

    // initial angular acceleration (quaternion)
    JointState.h_hand_ang_acc_q_e_init_left = 0.5*JointState.I_3*(JointState.h_hand_ang_vel_q_w_init_left*h_hand_ang_vel_init_vec_left+JointState.h_hand_or_q_w_init_left*h_hand_ang_acc_init_vec_left)
                  -0.5*(JointState.h_hand_ang_vel_q_e_init_left.cross(h_hand_ang_vel_init_vec_left)+JointState.h_hand_or_q_e_init_left.cross(h_hand_ang_acc_init_vec_left));
    JointState.h_hand_ang_acc_q_w_init_left = -0.5*(JointState.h_hand_ang_vel_q_e_init_left.dot(h_hand_ang_vel_init_vec_left)+JointState.h_hand_or_q_e_init_left.dot(h_hand_ang_acc_init_vec_left));

    // end angular acceleration (quaternion)
    JointState.h_hand_ang_acc_q_e_end_left = 0.5*JointState.I_3*(JointState.h_hand_ang_vel_q_w_end_left*h_hand_ang_vel_end_vec_left+JointState.h_hand_or_q_w_end_left*h_hand_ang_acc_end_vec_left)
                  -0.5*(JointState.h_hand_ang_vel_q_e_end_left.cross(h_hand_ang_vel_end_vec_left)+JointState.h_hand_or_q_e_end_left.cross(h_hand_ang_acc_end_vec_left));
    JointState.h_hand_ang_acc_q_w_end_left = -0.5*(JointState.h_hand_ang_vel_q_e_end_left.dot(h_hand_ang_vel_end_vec_left)+JointState.h_hand_or_q_e_end_left.dot(h_hand_ang_acc_end_vec_left));


    Eigen::MatrixXd jointsPosition_left = ctx.jointsPosition_mov_ctrl_left.at(ctx.i_ctrl);
    Eigen::MatrixXd jointsVelocity_left = ctx.jointsVelocity_mov_ctrl_left.at(ctx.i_ctrl);
    Eigen::MatrixXd jointsAcceleration_left = ctx.jointsAcceleration_mov_ctrl_left.at(ctx.i_ctrl);


    //position
    JointState.jointsInitPosition_hand_left = jointsPosition_left.row(0).tail<JOINTS_HAND>();
    VectorXd::Map(&JointState.jointsInitPosition_hand_vec_left[0], JointState.jointsInitPosition_hand_left.size()) = JointState.jointsInitPosition_hand_left;
    JointState.jointsFinalPosition_hand_left = jointsPosition_left.row(jointsPosition_left.rows()-1).tail<JOINTS_HAND>();
    VectorXd::Map(&JointState.jointsFinalPosition_hand_vec_left[0], JointState.jointsFinalPosition_hand_left.size()) = JointState.jointsFinalPosition_hand_left;
    //velocity
    JointState.jointsInitVelocity_hand_left = jointsVelocity_left.row(0).tail<JOINTS_HAND>();
    VectorXd::Map(&JointState.jointsInitVelocity_hand_vec_left[0], JointState.jointsInitVelocity_hand_left.size()) = JointState.jointsInitVelocity_hand_left;
    JointState.jointsFinalVelocity_hand_left = jointsVelocity_left.row(jointsVelocity_left.rows()-1).tail<JOINTS_HAND>();
    VectorXd::Map(&JointState.jointsFinalVelocity_hand_vec_left[0], JointState.jointsFinalVelocity_hand_left.size()) = JointState.jointsFinalVelocity_hand_left;
    //acceleration
    JointState.jointsInitAcceleration_hand_left = jointsAcceleration_left.row(0).tail<JOINTS_HAND>();
    VectorXd::Map(&JointState.jointsInitAcceleration_hand_vec_left[0], JointState.jointsInitAcceleration_hand_left.size()) = JointState.jointsInitAcceleration_hand_left;
    JointState.jointsFinalAcceleration_hand_left = jointsAcceleration_left.row(jointsVelocity_left.rows()-1).tail<JOINTS_HAND>();
    VectorXd::Map(&JointState.jointsFinalAcceleration_hand_vec_left[0], JointState.jointsFinalAcceleration_hand_left.size()) = JointState.jointsFinalAcceleration_hand_left;

    des_pose.des_hand_pos_x_left = JointState.h_hand_pos_end_left.at(0);
    des_pose.des_hand_pos_y_left = JointState.h_hand_pos_end_left.at(1);
    des_pose.des_hand_pos_z_left = JointState.h_hand_pos_end_left.at(2);
    des_pose.des_hand_or_x_left = JointState.h_hand_or_end_left.at(0);
    des_pose.des_hand_or_y_left = JointState.h_hand_or_end_left.at(1);
    des_pose.des_hand_or_z_left = JointState.h_hand_or_end_left.at(2);
    des_pose.des_hand_or_q_x_left = JointState.h_hand_or_q_end_left.at(0);
    des_pose.des_hand_or_q_y_left = JointState.h_hand_or_q_end_left.at(1);
    des_pose.des_hand_or_q_z_left = JointState.h_hand_or_q_end_left.at(2);
    des_pose.des_hand_or_q_w_left = JointState.h_hand_or_q_end_left.at(3);
  }
}


void LocalPlanner::obs_avoidance() {
  vector<objectPtr> obsts; ctx.curr_scene->getObjects(obsts);

  //esta condição deveria ser corrida mesmo sem obst en!!!
  //em principio as variaveis afetadas pela mm estao circunscritas a esta condição
  //não devendo ser um caso dramático

  if(ui.checkBox_use_plan_hand_pos->isChecked())
  {
    if((mov_type==0) || (mov_type==2) || (mov_type==3) || (mov_type==4)){
      /*
        // pick or place
        string obj_tar_name = ctx.curr_mov->getObject()->getName();
        string obj_tar_name_left = ctx.curr_mov->getObjectLeft()->getName();
        for(size_t i=0;i<obsts.size();++i)
        {
          if(obj_tar_name.compare(obsts.at(i)->getName())==0)
          {
              obsts.erase(obsts.begin()+i);
          }
          else if(obj_tar_name_left.compare(obsts.at(i)->getName())==0){
              obsts.erase(obsts.begin()+i);
          }
        }
        */
        string obj_tar_name = ctx.curr_mov->getObject()->getName();
        string obj_tar_name_left = ctx.curr_mov->getObjectLeft()->getName();
        size_t i = 0;
        while (i < obsts.size())
        {
            const std::string& name = obsts[i]->getName();
            if (name == obj_tar_name || name == obj_tar_name_left)
            {
                obsts.erase(obsts.begin() + i);
                // não incrementa i — o vector foi encurtado
            }
            else
            {
                ++i;
            }
        }
    }
  }
  //vector<objectPtr>  obsts_n = obsts;
  avoidance_var.obsts_n = obsts;

  //poderia ir para usr_interface
  avoidance_var.obst_coeff = ui.lineEdit_obsts_coeff->text().toDouble();
  avoidance_var.obst_damping = ui.lineEdit_obsts_damping->text().toDouble();
  avoidance_var.obst_coeff_torso = ui.lineEdit_obsts_coeff_torso->text().toDouble();
  avoidance_var.obst_damping_torso = ui.lineEdit_obsts_damping_torso->text().toDouble();


  if(ui.checkBox_obsts_noise->isChecked()){
    objectPtr obs_new;

    // ajusta o tamanho antes do loop
  //  avoidance_var.obsts_n.resize(obsts.size());

      for(size_t i=0; i<obsts.size(); ++i){
        objectPtr obs = obsts.at(i);
        string obs_name = obs->getName();
        obs_new.reset(new Object(obs_name));
        motion_manager::pos obs_pos; Quaterniond obs_or_q;
        obs_pos.Xpos = obs->getPos().Xpos - (noise_sim_obj.obj_x_var/2) + noise_sim_obj.obj_x_var*(rand() / double(RAND_MAX));
        obs_pos.Ypos = obs->getPos().Ypos - (noise_sim_obj.obj_y_var/2) + noise_sim_obj.obj_y_var*(rand() / double(RAND_MAX));
        obs_pos.Zpos = obs->getPos().Zpos - (noise_sim_obj.obj_z_var/2) + noise_sim_obj.obj_z_var*(rand() / double(RAND_MAX));
        obs_or_q.x() = obs->getQuaternion().x() - (noise_sim_obj.obj_q_x_var/2) + noise_sim_obj.obj_q_x_var*(rand() / double(RAND_MAX));
        obs_or_q.y() = obs->getQuaternion().y() - (noise_sim_obj.obj_q_y_var/2) + noise_sim_obj.obj_q_y_var*(rand() / double(RAND_MAX));
        obs_or_q.z() = obs->getQuaternion().z() - (noise_sim_obj.obj_q_z_var/2) + noise_sim_obj.obj_q_z_var*(rand() / double(RAND_MAX));
        obs_or_q.w() = obs->getQuaternion().w() - (noise_sim_obj.obj_q_w_var/2) + noise_sim_obj.obj_q_w_var*(rand() / double(RAND_MAX));
        if(ui.checkBox_obsts_filter_noise->isChecked()){
            obs_pos.Xpos = lpf.lpf_obsts_pos_x->update(obs_pos.Xpos);
            obs_pos.Ypos = lpf.lpf_obsts_pos_y->update(obs_pos.Ypos);
            obs_pos.Zpos = lpf.lpf_obsts_pos_z->update(obs_pos.Zpos);
            obs_or_q.x() = lpf.lpf_obsts_or_q_x->update(obs_or_q.x());
            obs_or_q.y() = lpf.lpf_obsts_or_q_y->update(obs_or_q.y());
            obs_or_q.z() = lpf.lpf_obsts_or_q_z->update(obs_or_q.z());
            obs_or_q.w() = lpf.lpf_obsts_or_q_w->update(obs_or_q.w());
        }
        obs_new->setPos(obs_pos,false);
        obs_new->setOr(obs_or_q,false);
        obs_new->setSize(obs->getSize());
        avoidance_var.obsts_n.at(i) = obs_new;
      } // for loop obstacles
  }// noise on obstacles

}


/**********************************************
*		Human-likeness                            *
* Superimposes human-likeness principles in   *
* in the present controller                   *
*                                               *
* To Do                                         *
* Bounce ainda nao esta tratado no planning     *
* Warm start so funciona para uma mao           *
*************************************************/

void LocalPlanner::human_likeness_pre_process() {
  //os coefficientes vao ser iguais para ambos os braços por enquanto

  if((this->h_results!=nullptr && this->h_results->status==0) || (this->h_dual_results!=nullptr && this->h_dual_results->status==0) ||
    (ui.radioButton_ctrl_task->isChecked() && ((h_r_flag == 1 && status == 0) || (h_r_flag == 2 && status == 0)))) //por enquanto pode ficar assim
    {
    // Plan
    ctx.mPlanCoeffsdlg->getPositionCoeffs(human_likeness_coef.hl_p_x_pos_coeff_plan,human_likeness_coef.hl_p_y_pos_coeff_plan,human_likeness_coef.hl_p_z_pos_coeff_plan,
                                human_likeness_coef.hl_p_x_or_coeff_plan,human_likeness_coef.hl_p_y_or_coeff_plan,human_likeness_coef.hl_p_z_or_coeff_plan);
    ctx.mPlanCoeffsdlg->getVelocityCoeffs(human_likeness_coef.hl_d_x_pos_coeff_plan,human_likeness_coef.hl_d_y_pos_coeff_plan,human_likeness_coef.hl_d_z_pos_coeff_plan,
                                human_likeness_coef.hl_d_x_or_coeff_plan,human_likeness_coef.hl_d_y_or_coeff_plan,human_likeness_coef.hl_d_z_or_coeff_plan);
    // Approach
    ctx.mAppCoeffsdlg->getPositionCoeffs(human_likeness_coef.hl_p_x_pos_coeff_app,human_likeness_coef.hl_p_y_pos_coeff_app,human_likeness_coef.hl_p_z_pos_coeff_app,
                                human_likeness_coef.hl_p_x_or_coeff_app,human_likeness_coef.hl_p_y_or_coeff_app,human_likeness_coef.hl_p_z_or_coeff_app);
    ctx.mAppCoeffsdlg->getVelocityCoeffs(human_likeness_coef.hl_d_x_pos_coeff_app,human_likeness_coef.hl_d_y_pos_coeff_app,human_likeness_coef.hl_d_z_pos_coeff_app,
                                human_likeness_coef.hl_d_x_or_coeff_app,human_likeness_coef.hl_d_y_or_coeff_app,human_likeness_coef.hl_d_z_or_coeff_app);
    // Retreat
    ctx.mRetCoeffsdlg->getPositionCoeffs(human_likeness_coef.hl_p_x_pos_coeff_ret,human_likeness_coef.hl_p_y_pos_coeff_ret,human_likeness_coef.hl_p_z_pos_coeff_ret,
                                human_likeness_coef.hl_p_x_or_coeff_ret,human_likeness_coef.hl_p_y_or_coeff_ret,human_likeness_coef.hl_p_z_or_coeff_ret);
    ctx.mRetCoeffsdlg->getVelocityCoeffs(human_likeness_coef.hl_d_x_pos_coeff_ret,human_likeness_coef.hl_d_y_pos_coeff_ret,human_likeness_coef.hl_d_z_pos_coeff_ret,
                                human_likeness_coef.hl_d_x_or_coeff_ret,human_likeness_coef.hl_d_y_or_coeff_ret,human_likeness_coef.hl_d_z_or_coeff_ret);

    //poderia ser obtido do usr_interface!!!
    // Swivel angle
    human_likeness_coef.hl_alpha_pos_coeff = ui.lineEdit_hl_alpha_pos_coeff->text().toDouble();
    human_likeness_coef.hl_alpha_vel_coeff = ui.lineEdit_hl_alpha_vel_coeff->text().toDouble();

    // Fingers - estudar possibilidade de retirar!!!
    human_likeness_coef.fing_p_coeff = ui.lineEdit_fing_p_coeff->text().toDouble();
    human_likeness_coef.fing_d_coeff = ui.lineEdit_fing_d_coeff->text().toDouble();

    time_var.g_map_th_pa = ui.lineEdit_g_th_plan_app->text().toDouble();

    if(usr_interface_input.follow_tar){
        time_var.g_map_th_rp = 2; // never go to retreat or to the subsequent movement
    }else{
        time_var.g_map_th_rp = ui.lineEdit_g_th_ret_plan->text().toDouble();
    }

    time_var.phi = ctx.curr_task->getProblem(ui.listWidget_movs->currentRow())->getHUMPlanner()->getPHI();
    time_var.tb = ctx.curr_task->getProblem(ui.listWidget_movs->currentRow())->getHUMPlanner()->getTB();



    vector<double> bounce_hand_pos;
    vector<double> bounce_hand_orr;
    vector<double> bounce_hand_orr_q;
    vector<double> bounce_hand_pos_left;
    vector<double> bounce_hand_orr_left;
    vector<double> bounce_hand_orr_q_left;

    bounce_hand_pos = ctx.bounce_handPosition;
    bounce_hand_orr = ctx.bounce_handOrientation;
    bounce_hand_orr_q = ctx.bounce_handOrientation_q;



    bounce_pose.bounce_hand_pos_x = bounce_hand_pos.at(0);
    bounce_pose.bounce_hand_pos_y = bounce_hand_pos.at(1);
    bounce_pose.bounce_hand_pos_z = bounce_hand_pos.at(2);
    bounce_pose.bounce_hand_or_x = bounce_hand_orr.at(0);
    bounce_pose.bounce_hand_or_y = bounce_hand_orr.at(1);
    bounce_pose.bounce_hand_or_z = bounce_hand_orr.at(2);
    bounce_pose.bounce_hand_q_x = bounce_hand_orr_q.at(0);
    bounce_pose.bounce_hand_q_y = bounce_hand_orr_q.at(1);
    bounce_pose.bounce_hand_q_z = bounce_hand_orr_q.at(2);
    bounce_pose.bounce_hand_q_w = bounce_hand_orr_q.at(3);

    if ((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))) {
      bounce_hand_pos_left = ctx.bounce_handPosition_left;
      bounce_hand_orr_left = ctx.bounce_handOrientation_left;
      bounce_hand_orr_q_left = ctx.bounce_handOrientation_q_left;


      bounce_pose.bounce_hand_pos_x_left = bounce_hand_pos_left.at(0);
      bounce_pose.bounce_hand_pos_y_left = bounce_hand_pos_left.at(1);
      bounce_pose.bounce_hand_pos_z_left = bounce_hand_pos_left.at(2);
      bounce_pose.bounce_hand_or_x_left = bounce_hand_orr_left.at(0);
      bounce_pose.bounce_hand_or_y_left = bounce_hand_orr_left.at(1);
      bounce_pose.bounce_hand_or_z_left = bounce_hand_orr_left.at(2);
      bounce_pose.bounce_hand_q_x_left = bounce_hand_orr_q_left.at(0);
      bounce_pose.bounce_hand_q_y_left = bounce_hand_orr_q_left.at(1);
      bounce_pose.bounce_hand_q_z_left = bounce_hand_orr_q_left.at(2);
      bounce_pose.bounce_hand_q_w_left = bounce_hand_orr_q_left.at(3);

    }

    //std::vector<double> bounce_posture = this->h_results->bounce_warm_start_res.x;
    //std::vector<double> bounce_hand_posture = std::vector<double>(bounce_posture.begin() + JOINTS_ARM, bounce_posture.end());

    vector<double> bounce_hand_posture(ctx.bounce_posture.begin(),ctx.bounce_posture.begin()+JOINTS_ARM);
    JointState.jointsBouncePosition_hand_vec = bounce_hand_posture;

    if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
      vector<double> bounce_hand_posture_left(ctx.bounce_posture_left.begin(),ctx.bounce_posture_left.begin()+JOINTS_ARM);
      JointState.jointsBouncePosition_hand_vec_left = bounce_hand_posture_left;
    }


    if(mov_type==2 || mov_type==3 ||mov_type==4){ // place
      JointState.jointsBouncePosition_hand = JointState.jointsInitPosition_hand;
        if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
          JointState.jointsBouncePosition_hand_left = JointState.jointsInitPosition_hand_left;
        }
    }else{
      JointState.jointsBouncePosition_hand << JointState.jointsBouncePosition_hand_vec.at(0);
        if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
          JointState.jointsBouncePosition_hand_left << JointState.jointsBouncePosition_hand_vec_left.at(0);
        }
    }
  }else if(this->h_results==nullptr || this->h_results->status!=0){ //nessecito de melhorar esta condição
    qnode.log(QNode::Error,string("The planning has failed."));
    return;
  }

  // desired position
   //Vector3d des_hand_or; VectorXd des_hand_or_q(4); Vector3d des_hand_or_q_e;
  des_pose.des_hand_pos << des_pose.des_hand_pos_x,des_pose.des_hand_pos_y,des_pose.des_hand_pos_z;
  mes_pose_str.des_hand_or << des_pose.des_hand_or_x,des_pose.des_hand_or_y,des_pose.des_hand_or_z;
  mes_pose_str.des_hand_or_q_e << des_pose.des_hand_or_q_x,des_pose.des_hand_or_q_y,des_pose.des_hand_or_q_z;
  mes_pose_str.des_hand_or_q << des_pose.des_hand_or_q_x,des_pose.des_hand_or_q_y,des_pose.des_hand_or_q_z,des_pose.des_hand_or_q_w;

  des_pose.des_hand_pos_left << des_pose.des_hand_pos_x_left,des_pose.des_hand_pos_y_left,des_pose.des_hand_pos_z_left;
  mes_pose_str.des_hand_or_left << des_pose.des_hand_or_x_left,des_pose.des_hand_or_y_left,des_pose.des_hand_or_z_left;
  mes_pose_str.des_hand_or_q_e_left << des_pose.des_hand_or_q_x_left,des_pose.des_hand_or_q_y_left,des_pose.des_hand_or_q_z_left;
  mes_pose_str.des_hand_or_q_left << des_pose.des_hand_or_q_x_left,des_pose.des_hand_or_q_y_left,des_pose.des_hand_or_q_z_left,des_pose.des_hand_or_q_w_left;


  /*
  human_likeness_coef.error_pos_th = ui.lineEdit_err_p_pos->text().toDouble();
  human_likeness_coef.error_or_th = ui.lineEdit_err_p_or->text().toDouble();
  human_likeness_coef.coeff_p_pos = ui.lineEdit_coeff_p_pos->text().toDouble();
  human_likeness_coef.coeff_p_or = ui.lineEdit_coeff_p_or->text().toDouble();
  human_likeness_coef.coeff_d_pos = ui.lineEdit_coeff_d_pos->text().toDouble();
  human_likeness_coef.coeff_d_or = ui.lineEdit_coeff_d_or->text().toDouble();*/
}

void LocalPlanner::human_like_exec() {
    // human-like profile
    double tau = 0.1; double dec_rate = 0.1; double diff_w = 0.1; //pode ser inicializado antes

    if(stage_descr.compare("plan")==0)
    {
        ctx.mTimeMapdlg->getPlanTimeMapping(tau,dec_rate,diff_w);
        // normalized mapped time
        if(usr_interface_input.sim_robot){
            time_var.g_map = 1 - exp((-dec_rate*ctx.curr_time)/(tau*(1+diff_w* error_var.error_tot.squaredNorm())));
        }else{
            double d_curr_time = (boost::chrono::duration_cast<msec>(ctx.curr_time_ctrl - ctx.start_time_point)).count();
            time_var.g_map = 1 - exp((-dec_rate*(d_curr_time/1000))/(tau*(1+diff_w* error_var.error_tot.squaredNorm())));
        }
        time_var.index = static_cast<int>(0.5+(JointState.n_steps-1)*time_var.g_map);
        // desired swivel angle references
        error_var.des_alpha_pos = JointState.alpha_positions.at(time_var.index);
        error_var.des_alpha_vel = JointState.alpha_velocities.at(time_var.index);
        error_var.des_alpha_acc = JointState.alpha_accelerations.at(time_var.index);

        // position
        human_likeness_coef.hl_p_x_pos_coeff = human_likeness_coef.hl_p_x_pos_coeff_plan;
        human_likeness_coef.hl_p_y_pos_coeff = human_likeness_coef.hl_p_y_pos_coeff_plan;
        human_likeness_coef.hl_p_z_pos_coeff = human_likeness_coef.hl_p_z_pos_coeff_plan;
        human_likeness_coef.hl_p_x_or_coeff = human_likeness_coef.hl_p_x_or_coeff_plan;
        human_likeness_coef.hl_p_y_or_coeff = human_likeness_coef.hl_p_y_or_coeff_plan;
        human_likeness_coef.hl_p_z_or_coeff = human_likeness_coef.hl_p_z_or_coeff_plan;

        // velocity
        human_likeness_coef.hl_d_x_pos_coeff = human_likeness_coef.hl_d_x_pos_coeff_plan;
        human_likeness_coef.hl_d_y_pos_coeff = human_likeness_coef.hl_d_y_pos_coeff_plan;
        human_likeness_coef.hl_d_z_pos_coeff = human_likeness_coef.hl_d_z_pos_coeff_plan;
        human_likeness_coef.hl_d_x_or_coeff = human_likeness_coef.hl_d_x_or_coeff_plan;
        human_likeness_coef.hl_d_y_or_coeff = human_likeness_coef.hl_d_y_or_coeff_plan;
        human_likeness_coef.hl_d_z_or_coeff = human_likeness_coef.hl_d_z_or_coeff_plan;

        Vector3d bounce_hand_pos; bounce_hand_pos << bounce_pose.bounce_hand_pos_x,bounce_pose.bounce_hand_pos_y,bounce_pose.bounce_hand_pos_z;
        Vector3d bounce_hand_or; bounce_hand_or << bounce_pose.bounce_hand_or_x,bounce_pose.bounce_hand_or_y,bounce_pose.bounce_hand_or_z;
        Vector3d error_b_pos = bounce_hand_pos - des_pose.hand_pos_init_vec;

        VectorXd error_b_orr(4);
        error_b_orr(0) = bounce_pose.bounce_hand_q_x - des_pose.hand_or_q_e_init_vec(0);
        error_b_orr(1) = bounce_pose.bounce_hand_q_y - des_pose.hand_or_q_e_init_vec(1);
        error_b_orr(2) = bounce_pose.bounce_hand_q_z - des_pose.hand_or_q_e_init_vec(2);
        error_b_orr(3) = bounce_pose.bounce_hand_q_w - des_pose.hand_or_q_w_init_vec;
        VectorXd error_b_tot(7); error_b_tot << error_b_pos(0),error_b_pos(1),error_b_pos(2),
                                error_b_orr(0),error_b_orr(1),error_b_orr(2),error_b_orr(3);

        Vector3d bounce_hand_orr_q_e_vec; bounce_hand_orr_q_e_vec << bounce_pose.bounce_hand_q_x,bounce_pose.bounce_hand_q_y,bounce_pose.bounce_hand_q_z;

        VectorXd error_b_fing_tot = JointState.jointsBouncePosition_hand - JointState.jointsInitPosition_hand;

        VectorXd error_b_tot_left(7);
        VectorXd error_b_fing_tot_left;
        if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
          // desired swivel angle references
          error_var.des_alpha_pos_left = JointState.alpha_positions_left.at(time_var.index);
          error_var.des_alpha_vel_left = JointState.alpha_velocities_left.at(time_var.index);
          error_var.des_alpha_acc_left = JointState.alpha_accelerations_left.at(time_var.index);


          Vector3d bounce_hand_pos_left; bounce_hand_pos_left << bounce_pose.bounce_hand_pos_x_left,bounce_pose.bounce_hand_pos_y_left,bounce_pose.bounce_hand_pos_z_left;
          Vector3d bounce_hand_or_left; bounce_hand_or_left << bounce_pose.bounce_hand_or_x_left,bounce_pose.bounce_hand_or_y_left,bounce_pose.bounce_hand_or_z_left;
          Vector3d error_b_pos_left = bounce_hand_pos_left - des_pose.hand_pos_init_vec_left;

          VectorXd error_b_orr_left(4);
          error_b_orr_left(0) = bounce_pose.bounce_hand_q_x_left - des_pose.hand_or_q_e_init_vec_left(0);
          error_b_orr_left(1) = bounce_pose.bounce_hand_q_y_left - des_pose.hand_or_q_e_init_vec_left(1);
          error_b_orr_left(2) = bounce_pose.bounce_hand_q_z_left - des_pose.hand_or_q_e_init_vec_left(2);
          error_b_orr_left(3) = bounce_pose.bounce_hand_q_w_left - des_pose.hand_or_q_w_init_vec_left;
          error_b_tot_left << error_b_pos_left(0),error_b_pos_left(1),error_b_pos_left(2),
                                  error_b_orr_left(0),error_b_orr_left(1),error_b_orr_left(2),error_b_orr_left(3);


          //estas sao usadas??
          Vector3d bounce_hand_orr_q_e_vec_left; bounce_hand_orr_q_e_vec_left << bounce_pose.bounce_hand_q_x_left,bounce_pose.bounce_hand_q_y_left,bounce_pose.bounce_hand_q_z_left;

          error_b_fing_tot_left = JointState.jointsBouncePosition_hand_left - JointState.jointsInitPosition_hand_left;
        }


        //Define o comportamento offline (dissertação Gianpaollo 202, 203, 204)
        // human-like desired hand pose
        des_pose.h_hand_pose(0) = des_pose.hand_pos_init_vec(0) + error_var.error_f_tot(0)*(10*pow(time_var.g_map,3)-15*pow(time_var.g_map,4)+6*pow(time_var.g_map,5))
                         +JointState.h_hand_lin_vel_init.at(0)*JointState.period_T*(time_var.g_map-6*pow(time_var.g_map,3)+8*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                         +JointState.h_hand_lin_vel_end.at(0)*JointState.period_T*(-4*pow(time_var.g_map,3)+7*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                         +0.5*JointState.h_hand_lin_acc_init.at(0)*pow(JointState.period_T,2)*(pow(time_var.g_map,2)-3*pow(time_var.g_map,3)+3*pow(time_var.g_map,4)-pow(time_var.g_map,5))
                         +0.5*JointState.h_hand_lin_acc_end.at(0)*pow(JointState.period_T,2)*(pow(time_var.g_map,3)-2*pow(time_var.g_map,4)+pow(time_var.g_map,5))
                         +error_b_tot(0)*((time_var.g_map*(1-time_var.g_map))/(time_var.tb*(1-time_var.tb)))*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2);

        des_pose.h_hand_pose(1) = des_pose.hand_pos_init_vec(1) + error_var.error_f_tot(1)*(10*pow(time_var.g_map,3)-15*pow(time_var.g_map,4)+6*pow(time_var.g_map,5))
                         +JointState.h_hand_lin_vel_init.at(1)*JointState.period_T*(time_var.g_map-6*pow(time_var.g_map,3)+8*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                         +JointState.h_hand_lin_vel_end.at(1)*JointState.period_T*(-4*pow(time_var.g_map,3)+7*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                         +0.5*JointState.h_hand_lin_acc_init.at(1)*pow(JointState.period_T,2)*(pow(time_var.g_map,2)-3*pow(time_var.g_map,3)+3*pow(time_var.g_map,4)-pow(time_var.g_map,5))
                         +0.5*JointState.h_hand_lin_acc_end.at(1)*pow(JointState.period_T,2)*(pow(time_var.g_map,3)-2*pow(time_var.g_map,4)+pow(time_var.g_map,5))
                         +error_b_tot(1)*((time_var.g_map*(1-time_var.g_map))/(time_var.tb*(1-time_var.tb)))*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2);

        des_pose.h_hand_pose(2) = des_pose.hand_pos_init_vec(2) + error_var.error_f_tot(2)*(10*pow(time_var.g_map,3)-15*pow(time_var.g_map,4)+6*pow(time_var.g_map,5))
                         +JointState.h_hand_lin_vel_init.at(2)*JointState.period_T*(time_var.g_map-6*pow(time_var.g_map,3)+8*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                         +JointState.h_hand_lin_vel_end.at(2)*JointState.period_T*(-4*pow(time_var.g_map,3)+7*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                         +0.5*JointState.h_hand_lin_acc_init.at(2)*pow(JointState.period_T,2)*(pow(time_var.g_map,2)-3*pow(time_var.g_map,3)+3*pow(time_var.g_map,4)-pow(time_var.g_map,5))
                         +0.5*JointState.h_hand_lin_acc_end.at(2)*pow(JointState.period_T,2)*(pow(time_var.g_map,3)-2*pow(time_var.g_map,4)+pow(time_var.g_map,5))
                         +error_b_tot(2)*((time_var.g_map*(1-time_var.g_map))/(time_var.tb*(1-time_var.tb)))*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2);

        Vector3d h_or_q_e; double h_or_q_w;
        h_or_q_e(0) = des_pose.hand_or_q_e_init_vec(0) + error_var.error_f_tot(3)*(10*pow(time_var.g_map,3)-15*pow(time_var.g_map,4)+6*pow(time_var.g_map,5))
                         +JointState.h_hand_ang_vel_q_e_init(0)*JointState.period_T*(time_var.g_map-6*pow(time_var.g_map,3)+8*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                         +JointState.h_hand_ang_vel_q_e_end(0)*JointState.period_T*(-4*pow(time_var.g_map,3)+7*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                         +0.5*JointState.h_hand_ang_acc_q_e_init(0)*pow(JointState.period_T,2)*(pow(time_var.g_map,2)-3*pow(time_var.g_map,3)+3*pow(time_var.g_map,4)-pow(time_var.g_map,5))
                         +0.5*JointState.h_hand_ang_acc_q_e_end(0)*pow(JointState.period_T,2)*(pow(time_var.g_map,3)-2*pow(time_var.g_map,4)+pow(time_var.g_map,5))
                         +error_b_tot(3)*((time_var.g_map*(1-time_var.g_map))/(time_var.tb*(1-time_var.tb)))*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2);

        h_or_q_e(1) = des_pose.hand_or_q_e_init_vec(1) + error_var.error_f_tot(4)*(10*pow(time_var.g_map,3)-15*pow(time_var.g_map,4)+6*pow(time_var.g_map,5))
                         +JointState.h_hand_ang_vel_q_e_init(1)*JointState.period_T*(time_var.g_map-6*pow(time_var.g_map,3)+8*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                         +JointState.h_hand_ang_vel_q_e_end(1)*JointState.period_T*(-4*pow(time_var.g_map,3)+7*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                         +0.5*JointState.h_hand_ang_acc_q_e_init(1)*pow(JointState.period_T,2)*(pow(time_var.g_map,2)-3*pow(time_var.g_map,3)+3*pow(time_var.g_map,4)-pow(time_var.g_map,5))
                         +0.5*JointState.h_hand_ang_acc_q_e_end(1)*pow(JointState.period_T,2)*(pow(time_var.g_map,3)-2*pow(time_var.g_map,4)+pow(time_var.g_map,5))
                         +error_b_tot(4)*((time_var.g_map*(1-time_var.g_map))/(time_var.tb*(1-time_var.tb)))*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2);

        h_or_q_e(2) = des_pose.hand_or_q_e_init_vec(2) + error_var.error_f_tot(5)*(10*pow(time_var.g_map,3)-15*pow(time_var.g_map,4)+6*pow(time_var.g_map,5))
                         +JointState.h_hand_ang_vel_q_e_init(2)*JointState.period_T*(time_var.g_map-6*pow(time_var.g_map,3)+8*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                         +JointState.h_hand_ang_vel_q_e_end(2)*JointState.period_T*(-4*pow(time_var.g_map,3)+7*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                         +0.5*JointState.h_hand_ang_acc_q_e_init(2)*pow(JointState.period_T,2)*(pow(time_var.g_map,2)-3*pow(time_var.g_map,3)+3*pow(time_var.g_map,4)-pow(time_var.g_map,5))
                         +0.5*JointState.h_hand_ang_acc_q_e_end(2)*pow(JointState.period_T,2)*(pow(time_var.g_map,3)-2*pow(time_var.g_map,4)+pow(time_var.g_map,5))
                         +error_b_tot(5)*((time_var.g_map*(1-time_var.g_map))/(time_var.tb*(1-time_var.tb)))*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2);

        h_or_q_w = des_pose.hand_or_q_w_init_vec + error_var.error_f_tot(6)*(10*pow(time_var.g_map,3)-15*pow(time_var.g_map,4)+6*pow(time_var.g_map,5))
                         +JointState.h_hand_ang_vel_q_w_init*JointState.period_T*(time_var.g_map-6*pow(time_var.g_map,3)+8*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                         +JointState.h_hand_ang_vel_q_w_end*JointState.period_T*(-4*pow(time_var.g_map,3)+7*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                         +0.5*JointState.h_hand_ang_acc_q_w_init*pow(JointState.period_T,2)*(pow(time_var.g_map,2)-3*pow(time_var.g_map,3)+3*pow(time_var.g_map,4)-pow(time_var.g_map,5))
                         +0.5*JointState.h_hand_ang_acc_q_w_end*pow(JointState.period_T,2)*(pow(time_var.g_map,3)-2*pow(time_var.g_map,4)+pow(time_var.g_map,5))
                         +error_b_tot(6)*((time_var.g_map*(1-time_var.g_map))/(time_var.tb*(1-time_var.tb)))*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2);

        des_pose.h_hand_pose(3) = h_or_q_e(0);
        des_pose.h_hand_pose(4) = h_or_q_e(1);
        des_pose.h_hand_pose(5) = h_or_q_e(2);
        des_pose.h_hand_pose(6) = h_or_q_w;

        // human-like desired finger positions

        des_pose.h_fing_pos(0) = JointState.jointsInitPosition_hand(0) + error_var.error_f_fing_tot(0)*(10*pow(time_var.g_map,3)-15*pow(time_var.g_map,4)+6*pow(time_var.g_map,5))
            +JointState.jointsInitVelocity_hand(0)*JointState.period_T*(time_var.g_map-6*pow(time_var.g_map,3)+8*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
            +JointState.jointsFinalVelocity_hand(0)*JointState.period_T*(-4*pow(time_var.g_map,3)+7*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
            +0.5*JointState.jointsInitAcceleration_hand(0)*pow(JointState.period_T,2)*(pow(time_var.g_map,2)-3*pow(time_var.g_map,3)+3*pow(time_var.g_map,4)-pow(time_var.g_map,5))
            +0.5*JointState.jointsFinalAcceleration_hand(0)*pow(JointState.period_T,2)*(pow(time_var.g_map,3)-2*pow(time_var.g_map,4)+pow(time_var.g_map,5))
            +error_b_fing_tot(0)*((time_var.g_map*(1-time_var.g_map))/(time_var.tb*(1-time_var.tb)))*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2);


        // human-like desired hand velocity
        des_pose.h_hand_vel(0) = (30/JointState.period_T)*error_var.error_f_tot(0)*(pow(time_var.g_map,2)-2*pow(time_var.g_map,3)+pow(time_var.g_map,4))
                        +JointState.h_hand_lin_vel_init.at(0)*(1-18*pow(time_var.g_map,2)+32*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                        +JointState.h_hand_lin_vel_end.at(0)*(-12*pow(time_var.g_map,2)+28*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                        +(0.5*JointState.h_hand_lin_acc_init.at(0)*JointState.period_T)*(2*time_var.g_map-9*pow(time_var.g_map,2)+12*pow(time_var.g_map,3)-5*pow(time_var.g_map,4))
                        +(0.5*JointState.h_hand_lin_acc_end.at(0)*JointState.period_T)*(3*pow(time_var.g_map,3)-8*pow(time_var.g_map,3)+5*pow(time_var.g_map,4))
                        +(1/(JointState.period_T*time_var.tb*(1-time_var.tb)))*error_b_tot(0)*((1-2*time_var.g_map)*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)+(1-time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

        des_pose.h_hand_vel(1) = (30/JointState.period_T)*error_var.error_f_tot(1)*(pow(time_var.g_map,2)-2*pow(time_var.g_map,3)+pow(time_var.g_map,4))
                        +JointState.h_hand_lin_vel_init.at(1)*(1-18*pow(time_var.g_map,2)+32*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                        +JointState.h_hand_lin_vel_end.at(1)*(-12*pow(time_var.g_map,2)+28*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                        +(0.5*JointState.h_hand_lin_acc_init.at(1)*JointState.period_T)*(2*time_var.g_map-9*pow(time_var.g_map,2)+12*pow(time_var.g_map,3)-5*pow(time_var.g_map,4))
                        +(0.5*JointState.h_hand_lin_acc_end.at(1)*JointState.period_T)*(3*pow(time_var.g_map,3)-8*pow(time_var.g_map,3)+5*pow(time_var.g_map,4))
                        +(1/(JointState.period_T*time_var.tb*(1-time_var.tb)))*error_b_tot(1)*((1-2*time_var.g_map)*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)+(1-time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

        des_pose.h_hand_vel(2) = (30/JointState.period_T)*error_var.error_f_tot(2)*(pow(time_var.g_map,2)-2*pow(time_var.g_map,3)+pow(time_var.g_map,4))
                        +JointState.h_hand_lin_vel_init.at(2)*(1-18*pow(time_var.g_map,2)+32*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                        +JointState.h_hand_lin_vel_end.at(2)*(-12*pow(time_var.g_map,2)+28*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                        +(0.5*JointState.h_hand_lin_acc_init.at(2)*JointState.period_T)*(2*time_var.g_map-9*pow(time_var.g_map,2)+12*pow(time_var.g_map,3)-5*pow(time_var.g_map,4))
                        +(0.5*JointState.h_hand_lin_acc_end.at(2)*JointState.period_T)*(3*pow(time_var.g_map,3)-8*pow(time_var.g_map,3)+5*pow(time_var.g_map,4))
                        +(1/(JointState.period_T*time_var.tb*(1-time_var.tb)))*error_b_tot(2)*((1-2*time_var.g_map)*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)+(1-time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

        Vector3d h_vel_q_e; double h_vel_q_w;
        h_vel_q_e(0) = (30/JointState.period_T)*error_var.error_f_tot(3)*(pow(time_var.g_map,2)-2*pow(time_var.g_map,3)+pow(time_var.g_map,4))
                        +JointState.h_hand_ang_vel_q_e_init(0)*(1-18*pow(time_var.g_map,2)+32*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                        +JointState.h_hand_ang_vel_q_e_end(0)*(-12*pow(time_var.g_map,2)+28*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                        +(0.5*JointState.h_hand_ang_acc_q_e_init(0)*JointState.period_T)*(2*time_var.g_map-9*pow(time_var.g_map,2)+12*pow(time_var.g_map,3)-5*pow(time_var.g_map,4))
                        +(0.5*JointState.h_hand_ang_acc_q_e_end(0)*JointState.period_T)*(3*pow(time_var.g_map,3)-8*pow(time_var.g_map,3)+5*pow(time_var.g_map,4))
                        +(1/(JointState.period_T*time_var.tb*(1-time_var.tb)))*error_b_tot(3)*((1-2*time_var.g_map)*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)+(1-time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

        h_vel_q_e(1) = (30/JointState.period_T)*error_var.error_f_tot(4)*(pow(time_var.g_map,2)-2*pow(time_var.g_map,3)+pow(time_var.g_map,4))
                  +JointState.h_hand_ang_vel_q_e_init(1)*(1-18*pow(time_var.g_map,2)+32*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                  +JointState.h_hand_ang_vel_q_e_end(1)*(-12*pow(time_var.g_map,2)+28*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                  +(0.5*JointState.h_hand_ang_acc_q_e_init(1)*JointState.period_T)*(2*time_var.g_map-9*pow(time_var.g_map,2)+12*pow(time_var.g_map,3)-5*pow(time_var.g_map,4))
                  +(0.5*JointState.h_hand_ang_acc_q_e_end(1)*JointState.period_T)*(3*pow(time_var.g_map,3)-8*pow(time_var.g_map,3)+5*pow(time_var.g_map,4))
                  +(1/(JointState.period_T*time_var.tb*(1-time_var.tb)))*error_b_tot(4)*((1-2*time_var.g_map)*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)+(1-time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

        h_vel_q_e(2) = (30/JointState.period_T)*error_var.error_f_tot(5)*(pow(time_var.g_map,2)-2*pow(time_var.g_map,3)+pow(time_var.g_map,4))
                  +JointState.h_hand_ang_vel_q_e_init(2)*(1-18*pow(time_var.g_map,2)+32*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                  +JointState.h_hand_ang_vel_q_e_end(2)*(-12*pow(time_var.g_map,2)+28*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                  +(0.5*JointState.h_hand_ang_acc_q_e_init(2)*JointState.period_T)*(2*time_var.g_map-9*pow(time_var.g_map,2)+12*pow(time_var.g_map,3)-5*pow(time_var.g_map,4))
                  +(0.5*JointState.h_hand_ang_acc_q_e_end(2)*JointState.period_T)*(3*pow(time_var.g_map,3)-8*pow(time_var.g_map,3)+5*pow(time_var.g_map,4))
                  +(1/(JointState.period_T*time_var.tb*(1-time_var.tb)))*error_b_tot(5)*((1-2*time_var.g_map)*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)+(1-time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

        h_vel_q_w = (30/JointState.period_T)*error_var.error_f_tot(6)*(pow(time_var.g_map,2)-2*pow(time_var.g_map,3)+pow(time_var.g_map,4))
                  +JointState.h_hand_ang_vel_q_w_init*(1-18*pow(time_var.g_map,2)+32*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                  +JointState.h_hand_ang_vel_q_w_end*(-12*pow(time_var.g_map,2)+28*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                  +(0.5*JointState.h_hand_ang_acc_q_w_init*JointState.period_T)*(2*time_var.g_map-9*pow(time_var.g_map,2)+12*pow(time_var.g_map,3)-5*pow(time_var.g_map,4))
                  +(0.5*JointState.h_hand_ang_acc_q_w_end*JointState.period_T)*(3*pow(time_var.g_map,3)-8*pow(time_var.g_map,3)+5*pow(time_var.g_map,4))
                  +(1/(JointState.period_T*time_var.tb*(1-time_var.tb)))*error_b_tot(6)*((1-2*time_var.g_map)*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)+(1-time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));


              Vector3d h_ang_vel;
              h_ang_vel = 2*(h_or_q_e.cross(h_vel_q_e)+h_or_q_w*h_vel_q_e-h_vel_q_w*h_or_q_e);
              des_pose.h_hand_vel(3) = h_ang_vel(0);
              des_pose.h_hand_vel(4) = h_ang_vel(1);
              des_pose.h_hand_vel(5) = h_ang_vel(2);
              //h_velq_w não é  usado????

        // human-like desired finger velocity

        des_pose.h_fing_vel(0) = (30/JointState.period_T)*error_var.error_f_fing_tot(0)*(pow(time_var.g_map,2)-2*pow(time_var.g_map,3)+pow(time_var.g_map,4))
            +JointState.jointsInitVelocity_hand(0)*(1-18*pow(time_var.g_map,2)+32*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
            +JointState.jointsFinalVelocity_hand(0)*(-12*pow(time_var.g_map,2)+28*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
            +(0.5*JointState.jointsInitAcceleration_hand(0)*JointState.period_T)*(2*time_var.g_map-9*pow(time_var.g_map,2)+12*pow(time_var.g_map,3)-5*pow(time_var.g_map,4))
            +(0.5*JointState.jointsFinalAcceleration_hand(0)*JointState.period_T)*(3*pow(time_var.g_map,3)-8*pow(time_var.g_map,3)+5*pow(time_var.g_map,4))
            +(1/(JointState.period_T*time_var.tb*(1-time_var.tb)))*error_b_fing_tot(0)*((1-2*time_var.g_map)*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)+(1-time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));


        // human-like desired hand acceleration
        des_pose.h_hand_acc(0) = (60/pow(JointState.period_T,2))*error_var.error_f_tot(0)*(time_var.g_map-3*pow(time_var.g_map,2)+2*pow(time_var.g_map,3))
                  +(12/JointState.period_T)*JointState.h_hand_lin_vel_init.at(0)*(-3*time_var.g_map+8*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                  +(12/JointState.period_T)*JointState.h_hand_lin_vel_end.at(0)*(-2*time_var.g_map+7*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                  +JointState.h_hand_lin_acc_init.at(0)*(1-9*time_var.g_map+18*pow(time_var.g_map,2)-10*pow(time_var.g_map,3))
                  +JointState.h_hand_lin_acc_end.at(0)*(3*time_var.g_map-12*pow(time_var.g_map,2)+10*pow(time_var.g_map,3))
                  +(2/(pow(JointState.period_T,2)*time_var.tb*(1-time_var.tb)))*error_b_tot(0)*
                   (-pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)
                   +pow(M_PI,2)*pow(time_var.phi,2)*(1-time_var.g_map)*pow(time_var.g_map,2*time_var.phi-1)*cos(2*M_PI*pow(time_var.g_map,time_var.phi))
                   +(1-2*time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi-1)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

        des_pose.h_hand_acc(1) = (60/pow(JointState.period_T,2))*error_var.error_f_tot(1)*(time_var.g_map-3*pow(time_var.g_map,2)+2*pow(time_var.g_map,3))
                  +(12/JointState.period_T)*JointState.h_hand_lin_vel_init.at(1)*(-3*time_var.g_map+8*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                  +(12/JointState.period_T)*JointState.h_hand_lin_vel_end.at(1)*(-2*time_var.g_map+7*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                  +JointState.h_hand_lin_acc_init.at(1)*(1-9*time_var.g_map+18*pow(time_var.g_map,2)-10*pow(time_var.g_map,3))
                  +JointState.h_hand_lin_acc_end.at(1)*(3*time_var.g_map-12*pow(time_var.g_map,2)+10*pow(time_var.g_map,3))
                  +(2/(pow(JointState.period_T,2)*time_var.tb*(1-time_var.tb)))*error_b_tot(1)*
                   (-pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)
                   +pow(M_PI,2)*pow(time_var.phi,2)*(1-time_var.g_map)*pow(time_var.g_map,2*time_var.phi-1)*cos(2*M_PI*pow(time_var.g_map,time_var.phi))
                   +(1-2*time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi-1)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

        des_pose.h_hand_acc(2) = (60/pow(JointState.period_T,2))*error_var.error_f_tot(2)*(time_var.g_map-3*pow(time_var.g_map,2)+2*pow(time_var.g_map,3))
                  +(12/JointState.period_T)*JointState.h_hand_lin_vel_init.at(2)*(-3*time_var.g_map+8*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                  +(12/JointState.period_T)*JointState.h_hand_lin_vel_end.at(2)*(-2*time_var.g_map+7*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                  +JointState.h_hand_lin_acc_init.at(2)*(1-9*time_var.g_map+18*pow(time_var.g_map,2)-10*pow(time_var.g_map,3))
                  +JointState.h_hand_lin_acc_end.at(2)*(3*time_var.g_map-12*pow(time_var.g_map,2)+10*pow(time_var.g_map,3))
                  +(2/(pow(JointState.period_T,2)*time_var.tb*(1-time_var.tb)))*error_b_tot(2)*
                   (-pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)
                   +pow(M_PI,2)*pow(time_var.phi,2)*(1-time_var.g_map)*pow(time_var.g_map,2*time_var.phi-1)*cos(2*M_PI*pow(time_var.g_map,time_var.phi))
                   +(1-2*time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi-1)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

        Vector3d h_acc_q_e; double h_acc_q_w;
        h_acc_q_e(0) = (60/pow(JointState.period_T,2))*error_var.error_f_tot(3)*(time_var.g_map-3*pow(time_var.g_map,2)+2*pow(time_var.g_map,3))
                  +(12/JointState.period_T)*JointState.h_hand_ang_vel_q_e_init(0)*(-3*time_var.g_map+8*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                  +(12/JointState.period_T)*JointState.h_hand_ang_vel_q_e_end(0)*(-2*time_var.g_map+7*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                  +JointState.h_hand_ang_acc_q_e_init(0)*(1-9*time_var.g_map+18*pow(time_var.g_map,2)-10*pow(time_var.g_map,3))
                  +JointState.h_hand_ang_acc_q_e_end(0)*(3*time_var.g_map-12*pow(time_var.g_map,2)+10*pow(time_var.g_map,3))
                  +(2/(pow(JointState.period_T,2)*time_var.tb*(1-time_var.tb)))*error_b_tot(3)*
                   (-pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)
                   +pow(M_PI,2)*pow(time_var.phi,2)*(1-time_var.g_map)*pow(time_var.g_map,2*time_var.phi-1)*cos(2*M_PI*pow(time_var.g_map,time_var.phi))
                   +(1-2*time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi-1)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

        h_acc_q_e(1) = (60/pow(JointState.period_T,2))*error_var.error_f_tot(4)*(time_var.g_map-3*pow(time_var.g_map,2)+2*pow(time_var.g_map,3))
                  +(12/JointState.period_T)*JointState.h_hand_ang_vel_q_e_init(1)*(-3*time_var.g_map+8*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                  +(12/JointState.period_T)*JointState.h_hand_ang_vel_q_e_end(1)*(-2*time_var.g_map+7*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                  +JointState.h_hand_ang_acc_q_e_init(1)*(1-9*time_var.g_map+18*pow(time_var.g_map,2)-10*pow(time_var.g_map,3))
                  +JointState.h_hand_ang_acc_q_e_end(1)*(3*time_var.g_map-12*pow(time_var.g_map,2)+10*pow(time_var.g_map,3))
                  +(2/(pow(JointState.period_T,2)*time_var.tb*(1-time_var.tb)))*error_b_tot(4)*
                   (-pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)
                   +pow(M_PI,2)*pow(time_var.phi,2)*(1-time_var.g_map)*pow(time_var.g_map,2*time_var.phi-1)*cos(2*M_PI*pow(time_var.g_map,time_var.phi))
                   +(1-2*time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi-1)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

        //h_hand_ang_vel_q_e_init e h_hand_ang_vel_q_e_end estavam com indice 0, mudei para 2
        h_acc_q_e(2) = (60/pow(JointState.period_T,2))*error_var.error_f_tot(5)*(time_var.g_map-3*pow(time_var.g_map,2)+2*pow(time_var.g_map,3))
                  +(12/JointState.period_T)*JointState.h_hand_ang_vel_q_e_init(2)*(-3*time_var.g_map+8*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                  +(12/JointState.period_T)*JointState.h_hand_ang_vel_q_e_end(2)*(-2*time_var.g_map+7*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                  +JointState.h_hand_ang_acc_q_e_init(2)*(1-9*time_var.g_map+18*pow(time_var.g_map,2)-10*pow(time_var.g_map,3))
                  +JointState.h_hand_ang_acc_q_e_end(2)*(3*time_var.g_map-12*pow(time_var.g_map,2)+10*pow(time_var.g_map,3))
                  +(2/(pow(JointState.period_T,2)*time_var.tb*(1-time_var.tb)))*error_b_tot(5)*
                   (-pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)
                   +pow(M_PI,2)*pow(time_var.phi,2)*(1-time_var.g_map)*pow(time_var.g_map,2*time_var.phi-1)*cos(2*M_PI*pow(time_var.g_map,time_var.phi))
                   +(1-2*time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi-1)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

        h_acc_q_w = (60/pow(JointState.period_T,2))*error_var.error_f_tot(6)*(time_var.g_map-3*pow(time_var.g_map,2)+2*pow(time_var.g_map,3))
              +(12/JointState.period_T)*JointState.h_hand_ang_vel_q_w_init*(-3*time_var.g_map+8*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
              +(12/JointState.period_T)*JointState.h_hand_ang_vel_q_w_end*(-2*time_var.g_map+7*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
              +JointState.h_hand_ang_acc_q_w_init*(1-9*time_var.g_map+18*pow(time_var.g_map,2)-10*pow(time_var.g_map,3))
              +JointState.h_hand_ang_acc_q_w_end*(3*time_var.g_map-12*pow(time_var.g_map,2)+10*pow(time_var.g_map,3))
              +(2/(pow(JointState.period_T,2)*time_var.tb*(1-time_var.tb)))*error_b_tot(6)*
               (-pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)
               +pow(M_PI,2)*pow(time_var.phi,2)*(1-time_var.g_map)*pow(time_var.g_map,2*time_var.phi-1)*cos(2*M_PI*pow(time_var.g_map,time_var.phi))
               +(1-2*time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi-1)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

              Vector3d h_ang_acc;
              h_ang_acc = 2*(h_or_q_e.cross(h_acc_q_e)+h_or_q_w*h_acc_q_e-h_acc_q_w*h_or_q_e);
              des_pose.h_hand_acc(3) = h_ang_acc(0);
              des_pose.h_hand_acc(4) = h_ang_acc(1);
              des_pose.h_hand_acc(5) = h_ang_acc(2);//h_acc_q_w não está a ser usado



        // human-like desired finger acceleration
        des_pose.h_fing_acc(0) = (60/pow(JointState.period_T,2))*error_var.error_f_fing_tot(0)*(time_var.g_map-3*pow(time_var.g_map,2)+2*pow(time_var.g_map,3))
            +(12/JointState.period_T)*JointState.jointsInitVelocity_hand(0)*(-3*time_var.g_map+8*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
            +(12/JointState.period_T)*JointState.jointsFinalVelocity_hand(0)*(-2*time_var.g_map+7*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
            +JointState.jointsInitAcceleration_hand(0)*(1-9*time_var.g_map+18*pow(time_var.g_map,2)-10*pow(time_var.g_map,3))
            +JointState.jointsFinalAcceleration_hand(0)*(3*time_var.g_map-12*pow(time_var.g_map,2)+10*pow(time_var.g_map,3))
            +(2/(pow(JointState.period_T,2)*time_var.tb*(1-time_var.tb)))*error_b_fing_tot(0)*
                (-pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)
                +pow(M_PI,2)*pow(time_var.phi,2)*(1-time_var.g_map)*pow(time_var.g_map,2*time_var.phi-1)*cos(2*M_PI*pow(time_var.g_map,time_var.phi))
                +(1-2*time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi-1)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

        //---------------------------- DUAL ARM --------------------
        if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){

          // human-like desired hand pose
          des_pose.h_hand_pose_left(0) = des_pose.hand_pos_init_vec_left(0) + error_var.error_f_tot_left(0)*(10*pow(time_var.g_map,3)-15*pow(time_var.g_map,4)+6*pow(time_var.g_map,5))
                           +JointState.h_hand_lin_vel_init_left.at(0)*JointState.period_T*(time_var.g_map-6*pow(time_var.g_map,3)+8*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                           +JointState.h_hand_lin_vel_end_left.at(0)*JointState.period_T*(-4*pow(time_var.g_map,3)+7*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                           +0.5*JointState.h_hand_lin_acc_init_left.at(0)*pow(JointState.period_T,2)*(pow(time_var.g_map,2)-3*pow(time_var.g_map,3)+3*pow(time_var.g_map,4)-pow(time_var.g_map,5))
                           +0.5*JointState.h_hand_lin_acc_end_left.at(0)*pow(JointState.period_T,2)*(pow(time_var.g_map,3)-2*pow(time_var.g_map,4)+pow(time_var.g_map,5))
                           +error_b_tot_left(0)*((time_var.g_map*(1-time_var.g_map))/(time_var.tb*(1-time_var.tb)))*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2);

          des_pose.h_hand_pose_left(1) = des_pose.hand_pos_init_vec_left(1) + error_var.error_f_tot_left(1)*(10*pow(time_var.g_map,3)-15*pow(time_var.g_map,4)+6*pow(time_var.g_map,5))
                           +JointState.h_hand_lin_vel_init_left.at(1)*JointState.period_T*(time_var.g_map-6*pow(time_var.g_map,3)+8*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                           +JointState.h_hand_lin_vel_end_left.at(1)*JointState.period_T*(-4*pow(time_var.g_map,3)+7*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                           +0.5*JointState.h_hand_lin_acc_init_left.at(1)*pow(JointState.period_T,2)*(pow(time_var.g_map,2)-3*pow(time_var.g_map,3)+3*pow(time_var.g_map,4)-pow(time_var.g_map,5))
                           +0.5*JointState.h_hand_lin_acc_end_left.at(1)*pow(JointState.period_T,2)*(pow(time_var.g_map,3)-2*pow(time_var.g_map,4)+pow(time_var.g_map,5))
                           +error_b_tot_left(1)*((time_var.g_map*(1-time_var.g_map))/(time_var.tb*(1-time_var.tb)))*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2);

          des_pose.h_hand_pose_left(2) = des_pose.hand_pos_init_vec_left(2) + error_var.error_f_tot_left(2)*(10*pow(time_var.g_map,3)-15*pow(time_var.g_map,4)+6*pow(time_var.g_map,5))
                           +JointState.h_hand_lin_vel_init_left.at(2)*JointState.period_T*(time_var.g_map-6*pow(time_var.g_map,3)+8*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                           +JointState.h_hand_lin_vel_end_left.at(2)*JointState.period_T*(-4*pow(time_var.g_map,3)+7*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                           +0.5*JointState.h_hand_lin_acc_init_left.at(2)*pow(JointState.period_T,2)*(pow(time_var.g_map,2)-3*pow(time_var.g_map,3)+3*pow(time_var.g_map,4)-pow(time_var.g_map,5))
                           +0.5*JointState.h_hand_lin_acc_end_left.at(2)*pow(JointState.period_T,2)*(pow(time_var.g_map,3)-2*pow(time_var.g_map,4)+pow(time_var.g_map,5))
                           +error_b_tot_left(2)*((time_var.g_map*(1-time_var.g_map))/(time_var.tb*(1-time_var.tb)))*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2);

          Vector3d h_or_q_e_left; double h_or_q_w_left;
          h_or_q_e_left(0) = des_pose.hand_or_q_e_init_vec_left(0) + error_var.error_f_tot_left(3)*(10*pow(time_var.g_map,3)-15*pow(time_var.g_map,4)+6*pow(time_var.g_map,5))
                           +JointState.h_hand_ang_vel_q_e_init_left(0)*JointState.period_T*(time_var.g_map-6*pow(time_var.g_map,3)+8*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                           +JointState.h_hand_ang_vel_q_e_end_left(0)*JointState.period_T*(-4*pow(time_var.g_map,3)+7*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                           +0.5*JointState.h_hand_ang_acc_q_e_init_left(0)*pow(JointState.period_T,2)*(pow(time_var.g_map,2)-3*pow(time_var.g_map,3)+3*pow(time_var.g_map,4)-pow(time_var.g_map,5))
                           +0.5*JointState.h_hand_ang_acc_q_e_end_left(0)*pow(JointState.period_T,2)*(pow(time_var.g_map,3)-2*pow(time_var.g_map,4)+pow(time_var.g_map,5))
                           +error_b_tot_left(3)*((time_var.g_map*(1-time_var.g_map))/(time_var.tb*(1-time_var.tb)))*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2);

          h_or_q_e_left(1) = des_pose.hand_or_q_e_init_vec_left(1) + error_var.error_f_tot_left(4)*(10*pow(time_var.g_map,3)-15*pow(time_var.g_map,4)+6*pow(time_var.g_map,5))
                           +JointState.h_hand_ang_vel_q_e_init_left(1)*JointState.period_T*(time_var.g_map-6*pow(time_var.g_map,3)+8*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                           +JointState.h_hand_ang_vel_q_e_end_left(1)*JointState.period_T*(-4*pow(time_var.g_map,3)+7*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                           +0.5*JointState.h_hand_ang_acc_q_e_init_left(1)*pow(JointState.period_T,2)*(pow(time_var.g_map,2)-3*pow(time_var.g_map,3)+3*pow(time_var.g_map,4)-pow(time_var.g_map,5))
                           +0.5*JointState.h_hand_ang_acc_q_e_end_left(1)*pow(JointState.period_T,2)*(pow(time_var.g_map,3)-2*pow(time_var.g_map,4)+pow(time_var.g_map,5))
                           +error_b_tot_left(4)*((time_var.g_map*(1-time_var.g_map))/(time_var.tb*(1-time_var.tb)))*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2);

          h_or_q_e_left(2) = des_pose.hand_or_q_e_init_vec_left(2) + error_var.error_f_tot_left(5)*(10*pow(time_var.g_map,3)-15*pow(time_var.g_map,4)+6*pow(time_var.g_map,5))
                           +JointState.h_hand_ang_vel_q_e_init_left(2)*JointState.period_T*(time_var.g_map-6*pow(time_var.g_map,3)+8*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                           +JointState.h_hand_ang_vel_q_e_end_left(2)*JointState.period_T*(-4*pow(time_var.g_map,3)+7*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                           +0.5*JointState.h_hand_ang_acc_q_e_init_left(2)*pow(JointState.period_T,2)*(pow(time_var.g_map,2)-3*pow(time_var.g_map,3)+3*pow(time_var.g_map,4)-pow(time_var.g_map,5))
                           +0.5*JointState.h_hand_ang_acc_q_e_end_left(2)*pow(JointState.period_T,2)*(pow(time_var.g_map,3)-2*pow(time_var.g_map,4)+pow(time_var.g_map,5))
                           +error_b_tot_left(5)*((time_var.g_map*(1-time_var.g_map))/(time_var.tb*(1-time_var.tb)))*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2);

          h_or_q_w_left = des_pose.hand_or_q_w_init_vec_left + error_var.error_f_tot_left(6)*(10*pow(time_var.g_map,3)-15*pow(time_var.g_map,4)+6*pow(time_var.g_map,5))
                           +JointState.h_hand_ang_vel_q_w_init_left*JointState.period_T*(time_var.g_map-6*pow(time_var.g_map,3)+8*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                           +JointState.h_hand_ang_vel_q_w_end_left*JointState.period_T*(-4*pow(time_var.g_map,3)+7*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
                           +0.5*JointState.h_hand_ang_acc_q_w_init_left*pow(JointState.period_T,2)*(pow(time_var.g_map,2)-3*pow(time_var.g_map,3)+3*pow(time_var.g_map,4)-pow(time_var.g_map,5))
                           +0.5*JointState.h_hand_ang_acc_q_w_end_left*pow(JointState.period_T,2)*(pow(time_var.g_map,3)-2*pow(time_var.g_map,4)+pow(time_var.g_map,5))
                           +error_b_tot_left(6)*((time_var.g_map*(1-time_var.g_map))/(time_var.tb*(1-time_var.tb)))*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2);

          des_pose.h_hand_pose_left(3) = h_or_q_e_left(0);
          des_pose.h_hand_pose_left(4) = h_or_q_e_left(1);
          des_pose.h_hand_pose_left(5) = h_or_q_e_left(2);
          des_pose.h_hand_pose_left(6) = h_or_q_w_left;

          // human-like desired finger positions

          des_pose.h_fing_pos_left(0) = JointState.jointsInitPosition_hand_left(0) + error_var.error_f_fing_tot_left(0)*(10*pow(time_var.g_map,3)-15*pow(time_var.g_map,4)+6*pow(time_var.g_map,5))
              +JointState.jointsInitVelocity_hand_left(0)*JointState.period_T*(time_var.g_map-6*pow(time_var.g_map,3)+8*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
              +JointState.jointsFinalVelocity_hand_left(0)*JointState.period_T*(-4*pow(time_var.g_map,3)+7*pow(time_var.g_map,4)-3*pow(time_var.g_map,5))
              +0.5*JointState.jointsInitAcceleration_hand_left(0)*pow(JointState.period_T,2)*(pow(time_var.g_map,2)-3*pow(time_var.g_map,3)+3*pow(time_var.g_map,4)-pow(time_var.g_map,5))
              +0.5*JointState.jointsFinalAcceleration_hand_left(0)*pow(JointState.period_T,2)*(pow(time_var.g_map,3)-2*pow(time_var.g_map,4)+pow(time_var.g_map,5))
              +error_b_fing_tot_left(0)*((time_var.g_map*(1-time_var.g_map))/(time_var.tb*(1-time_var.tb)))*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2);


          // human-like desired hand velocity
          des_pose.h_hand_vel_left(0) = (30/JointState.period_T)*error_var.error_f_tot_left(0)*(pow(time_var.g_map,2)-2*pow(time_var.g_map,3)+pow(time_var.g_map,4))
                          +JointState.h_hand_lin_vel_init_left.at(0)*(1-18*pow(time_var.g_map,2)+32*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                          +JointState.h_hand_lin_vel_end_left.at(0)*(-12*pow(time_var.g_map,2)+28*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                          +(0.5*JointState.h_hand_lin_acc_init_left.at(0)*JointState.period_T)*(2*time_var.g_map-9*pow(time_var.g_map,2)+12*pow(time_var.g_map,3)-5*pow(time_var.g_map,4))
                          +(0.5*JointState.h_hand_lin_acc_end_left.at(0)*JointState.period_T)*(3*pow(time_var.g_map,3)-8*pow(time_var.g_map,3)+5*pow(time_var.g_map,4))
                          +(1/(JointState.period_T*time_var.tb*(1-time_var.tb)))*error_b_tot_left(0)*((1-2*time_var.g_map)*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)+(1-time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

          des_pose.h_hand_vel_left(1) = (30/JointState.period_T)*error_var.error_f_tot_left(1)*(pow(time_var.g_map,2)-2*pow(time_var.g_map,3)+pow(time_var.g_map,4))
                          +JointState.h_hand_lin_vel_init_left.at(1)*(1-18*pow(time_var.g_map,2)+32*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                          +JointState.h_hand_lin_vel_end_left.at(1)*(-12*pow(time_var.g_map,2)+28*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                          +(0.5*JointState.h_hand_lin_acc_init_left.at(1)*JointState.period_T)*(2*time_var.g_map-9*pow(time_var.g_map,2)+12*pow(time_var.g_map,3)-5*pow(time_var.g_map,4))
                          +(0.5*JointState.h_hand_lin_acc_end_left.at(1)*JointState.period_T)*(3*pow(time_var.g_map,3)-8*pow(time_var.g_map,3)+5*pow(time_var.g_map,4))
                          +(1/(JointState.period_T*time_var.tb*(1-time_var.tb)))*error_b_tot_left(1)*((1-2*time_var.g_map)*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)+(1-time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

          des_pose.h_hand_vel_left(2) = (30/JointState.period_T)*error_var.error_f_tot_left(2)*(pow(time_var.g_map,2)-2*pow(time_var.g_map,3)+pow(time_var.g_map,4))
                          +JointState.h_hand_lin_vel_init_left.at(2)*(1-18*pow(time_var.g_map,2)+32*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                          +JointState.h_hand_lin_vel_end_left.at(2)*(-12*pow(time_var.g_map,2)+28*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                          +(0.5*JointState.h_hand_lin_acc_init_left.at(2)*JointState.period_T)*(2*time_var.g_map-9*pow(time_var.g_map,2)+12*pow(time_var.g_map,3)-5*pow(time_var.g_map,4))
                          +(0.5*JointState.h_hand_lin_acc_end_left.at(2)*JointState.period_T)*(3*pow(time_var.g_map,3)-8*pow(time_var.g_map,3)+5*pow(time_var.g_map,4))
                          +(1/(JointState.period_T*time_var.tb*(1-time_var.tb)))*error_b_tot_left(2)*((1-2*time_var.g_map)*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)+(1-time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

          Vector3d h_vel_q_e_left; double h_vel_q_w_left;
          h_vel_q_e_left(0) = (30/JointState.period_T)*error_var.error_f_tot_left(3)*(pow(time_var.g_map,2)-2*pow(time_var.g_map,3)+pow(time_var.g_map,4))
                          +JointState.h_hand_ang_vel_q_e_init_left(0)*(1-18*pow(time_var.g_map,2)+32*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                          +JointState.h_hand_ang_vel_q_e_end_left(0)*(-12*pow(time_var.g_map,2)+28*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                          +(0.5*JointState.h_hand_ang_acc_q_e_init_left(0)*JointState.period_T)*(2*time_var.g_map-9*pow(time_var.g_map,2)+12*pow(time_var.g_map,3)-5*pow(time_var.g_map,4))
                          +(0.5*JointState.h_hand_ang_acc_q_e_end_left(0)*JointState.period_T)*(3*pow(time_var.g_map,3)-8*pow(time_var.g_map,3)+5*pow(time_var.g_map,4))
                          +(1/(JointState.period_T*time_var.tb*(1-time_var.tb)))*error_b_tot_left(3)*((1-2*time_var.g_map)*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)+(1-time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

          h_vel_q_e_left(1) = (30/JointState.period_T)*error_var.error_f_tot_left(4)*(pow(time_var.g_map,2)-2*pow(time_var.g_map,3)+pow(time_var.g_map,4))
                    +JointState.h_hand_ang_vel_q_e_init_left(1)*(1-18*pow(time_var.g_map,2)+32*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                    +JointState.h_hand_ang_vel_q_e_end_left(1)*(-12*pow(time_var.g_map,2)+28*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                    +(0.5*JointState.h_hand_ang_acc_q_e_init_left(1)*JointState.period_T)*(2*time_var.g_map-9*pow(time_var.g_map,2)+12*pow(time_var.g_map,3)-5*pow(time_var.g_map,4))
                    +(0.5*JointState.h_hand_ang_acc_q_e_end_left(1)*JointState.period_T)*(3*pow(time_var.g_map,3)-8*pow(time_var.g_map,3)+5*pow(time_var.g_map,4))
                    +(1/(JointState.period_T*time_var.tb*(1-time_var.tb)))*error_b_tot_left(4)*((1-2*time_var.g_map)*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)+(1-time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

          h_vel_q_e_left(2) = (30/JointState.period_T)*error_var.error_f_tot_left(5)*(pow(time_var.g_map,2)-2*pow(time_var.g_map,3)+pow(time_var.g_map,4))
                    +JointState.h_hand_ang_vel_q_e_init_left(2)*(1-18*pow(time_var.g_map,2)+32*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                    +JointState.h_hand_ang_vel_q_e_end_left(2)*(-12*pow(time_var.g_map,2)+28*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                    +(0.5*JointState.h_hand_ang_acc_q_e_init_left(2)*JointState.period_T)*(2*time_var.g_map-9*pow(time_var.g_map,2)+12*pow(time_var.g_map,3)-5*pow(time_var.g_map,4))
                    +(0.5*JointState.h_hand_ang_acc_q_e_end_left(2)*JointState.period_T)*(3*pow(time_var.g_map,3)-8*pow(time_var.g_map,3)+5*pow(time_var.g_map,4))
                    +(1/(JointState.period_T*time_var.tb*(1-time_var.tb)))*error_b_tot_left(5)*((1-2*time_var.g_map)*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)+(1-time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

          h_vel_q_w_left = (30/JointState.period_T)*error_var.error_f_tot_left(6)*(pow(time_var.g_map,2)-2*pow(time_var.g_map,3)+pow(time_var.g_map,4))
                    +JointState.h_hand_ang_vel_q_w_init_left*(1-18*pow(time_var.g_map,2)+32*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                    +JointState.h_hand_ang_vel_q_w_end_left*(-12*pow(time_var.g_map,2)+28*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
                    +(0.5*JointState.h_hand_ang_acc_q_w_init_left*JointState.period_T)*(2*time_var.g_map-9*pow(time_var.g_map,2)+12*pow(time_var.g_map,3)-5*pow(time_var.g_map,4))
                    +(0.5*JointState.h_hand_ang_acc_q_w_end_left*JointState.period_T)*(3*pow(time_var.g_map,3)-8*pow(time_var.g_map,3)+5*pow(time_var.g_map,4))
                    +(1/(JointState.period_T*time_var.tb*(1-time_var.tb)))*error_b_tot_left(6)*((1-2*time_var.g_map)*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)+(1-time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));


                Vector3d h_ang_vel_left;

                h_ang_vel_left = 2*(h_or_q_e_left.cross(h_vel_q_e_left)+h_or_q_w_left*h_vel_q_e_left-h_vel_q_w_left*h_or_q_e_left);
                des_pose.h_hand_vel_left(3) = h_ang_vel_left(0);
                des_pose.h_hand_vel_left(4) = h_ang_vel_left(1);
                des_pose.h_hand_vel_left(5) = h_ang_vel_left(2);

          // human-like desired finger velocity

          des_pose.h_fing_vel_left(0) = (30/JointState.period_T)*error_var.error_f_fing_tot_left(0)*(pow(time_var.g_map,2)-2*pow(time_var.g_map,3)+pow(time_var.g_map,4))
              +JointState.jointsInitVelocity_hand_left(0)*(1-18*pow(time_var.g_map,2)+32*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
              +JointState.jointsFinalVelocity_hand_left(0)*(-12*pow(time_var.g_map,2)+28*pow(time_var.g_map,3)-15*pow(time_var.g_map,4))
              +(0.5*JointState.jointsInitAcceleration_hand_left(0)*JointState.period_T)*(2*time_var.g_map-9*pow(time_var.g_map,2)+12*pow(time_var.g_map,3)-5*pow(time_var.g_map,4))
              +(0.5*JointState.jointsFinalAcceleration_hand_left(0)*JointState.period_T)*(3*pow(time_var.g_map,3)-8*pow(time_var.g_map,3)+5*pow(time_var.g_map,4))
              +(1/(JointState.period_T*time_var.tb*(1-time_var.tb)))*error_b_fing_tot_left(0)*((1-2*time_var.g_map)*pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)+(1-time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));


          // human-like desired hand acceleration
          des_pose.h_hand_acc_left(0) = (60/pow(JointState.period_T,2))*error_var.error_f_tot_left(0)*(time_var.g_map-3*pow(time_var.g_map,2)+2*pow(time_var.g_map,3))
                    +(12/JointState.period_T)*JointState.h_hand_lin_vel_init_left.at(0)*(-3*time_var.g_map+8*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                    +(12/JointState.period_T)*JointState.h_hand_lin_vel_end_left.at(0)*(-2*time_var.g_map+7*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                    +JointState.h_hand_lin_acc_init_left.at(0)*(1-9*time_var.g_map+18*pow(time_var.g_map,2)-10*pow(time_var.g_map,3))
                    +JointState.h_hand_lin_acc_end_left.at(0)*(3*time_var.g_map-12*pow(time_var.g_map,2)+10*pow(time_var.g_map,3))
                    +(2/(pow(JointState.period_T,2)*time_var.tb*(1-time_var.tb)))*error_b_tot_left(0)*
                     (-pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)
                     +pow(M_PI,2)*pow(time_var.phi,2)*(1-time_var.g_map)*pow(time_var.g_map,2*time_var.phi-1)*cos(2*M_PI*pow(time_var.g_map,time_var.phi))
                     +(1-2*time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi-1)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

          des_pose.h_hand_acc_left(1) = (60/pow(JointState.period_T,2))*error_var.error_f_tot_left(1)*(time_var.g_map-3*pow(time_var.g_map,2)+2*pow(time_var.g_map,3))
                    +(12/JointState.period_T)*JointState.h_hand_lin_vel_init_left.at(1)*(-3*time_var.g_map+8*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                    +(12/JointState.period_T)*JointState.h_hand_lin_vel_end_left.at(1)*(-2*time_var.g_map+7*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                    +JointState.h_hand_lin_acc_init_left.at(1)*(1-9*time_var.g_map+18*pow(time_var.g_map,2)-10*pow(time_var.g_map,3))
                    +JointState.h_hand_lin_acc_end_left.at(1)*(3*time_var.g_map-12*pow(time_var.g_map,2)+10*pow(time_var.g_map,3))
                    +(2/(pow(JointState.period_T,2)*time_var.tb*(1-time_var.tb)))*error_b_tot_left(1)*
                     (-pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)
                     +pow(M_PI,2)*pow(time_var.phi,2)*(1-time_var.g_map)*pow(time_var.g_map,2*time_var.phi-1)*cos(2*M_PI*pow(time_var.g_map,time_var.phi))
                     +(1-2*time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi-1)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

          des_pose.h_hand_acc_left(2) = (60/pow(JointState.period_T,2))*error_var.error_f_tot_left(2)*(time_var.g_map-3*pow(time_var.g_map,2)+2*pow(time_var.g_map,3))
                    +(12/JointState.period_T)*JointState.h_hand_lin_vel_init_left.at(2)*(-3*time_var.g_map+8*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                    +(12/JointState.period_T)*JointState.h_hand_lin_vel_end_left.at(2)*(-2*time_var.g_map+7*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                    +JointState.h_hand_lin_acc_init_left.at(2)*(1-9*time_var.g_map+18*pow(time_var.g_map,2)-10*pow(time_var.g_map,3))
                    +JointState.h_hand_lin_acc_end_left.at(2)*(3*time_var.g_map-12*pow(time_var.g_map,2)+10*pow(time_var.g_map,3))
                    +(2/(pow(JointState.period_T,2)*time_var.tb*(1-time_var.tb)))*error_b_tot_left(2)*
                     (-pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)
                     +pow(M_PI,2)*pow(time_var.phi,2)*(1-time_var.g_map)*pow(time_var.g_map,2*time_var.phi-1)*cos(2*M_PI*pow(time_var.g_map,time_var.phi))
                     +(1-2*time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi-1)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

          Vector3d h_acc_q_e_left; double h_acc_q_w_left;
          h_acc_q_e_left(0) = (60/pow(JointState.period_T,2))*error_var.error_f_tot_left(3)*(time_var.g_map-3*pow(time_var.g_map,2)+2*pow(time_var.g_map,3))
                    +(12/JointState.period_T)*JointState.h_hand_ang_vel_q_e_init_left(0)*(-3*time_var.g_map+8*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                    +(12/JointState.period_T)*JointState.h_hand_ang_vel_q_e_end_left(0)*(-2*time_var.g_map+7*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                    +JointState.h_hand_ang_acc_q_e_init_left(0)*(1-9*time_var.g_map+18*pow(time_var.g_map,2)-10*pow(time_var.g_map,3))
                    +JointState.h_hand_ang_acc_q_e_end_left(0)*(3*time_var.g_map-12*pow(time_var.g_map,2)+10*pow(time_var.g_map,3))
                    +(2/(pow(JointState.period_T,2)*time_var.tb*(1-time_var.tb)))*error_b_tot_left(3)*
                     (-pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)
                     +pow(M_PI,2)*pow(time_var.phi,2)*(1-time_var.g_map)*pow(time_var.g_map,2*time_var.phi-1)*cos(2*M_PI*pow(time_var.g_map,time_var.phi))
                     +(1-2*time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi-1)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

          h_acc_q_e_left(1) = (60/pow(JointState.period_T,2))*error_var.error_f_tot_left(4)*(time_var.g_map-3*pow(time_var.g_map,2)+2*pow(time_var.g_map,3))
                    +(12/JointState.period_T)*JointState.h_hand_ang_vel_q_e_init_left(1)*(-3*time_var.g_map+8*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                    +(12/JointState.period_T)*JointState.h_hand_ang_vel_q_e_end_left(1)*(-2*time_var.g_map+7*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                    +JointState.h_hand_ang_acc_q_e_init_left(1)*(1-9*time_var.g_map+18*pow(time_var.g_map,2)-10*pow(time_var.g_map,3))
                    +JointState.h_hand_ang_acc_q_e_end_left(1)*(3*time_var.g_map-12*pow(time_var.g_map,2)+10*pow(time_var.g_map,3))
                    +(2/(pow(JointState.period_T,2)*time_var.tb*(1-time_var.tb)))*error_b_tot_left(4)*
                     (-pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)
                     +pow(M_PI,2)*pow(time_var.phi,2)*(1-time_var.g_map)*pow(time_var.g_map,2*time_var.phi-1)*cos(2*M_PI*pow(time_var.g_map,time_var.phi))
                     +(1-2*time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi-1)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

          //h_hand_ang_vel_q_e_init e h_hand_ang_vel_q_e_end estavam com indice 0, mudei para 2
          h_acc_q_e_left(2) = (60/pow(JointState.period_T,2))*error_var.error_f_tot_left(5)*(time_var.g_map-3*pow(time_var.g_map,2)+2*pow(time_var.g_map,3))
                    +(12/JointState.period_T)*JointState.h_hand_ang_vel_q_e_init_left(2)*(-3*time_var.g_map+8*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                    +(12/JointState.period_T)*JointState.h_hand_ang_vel_q_e_end_left(2)*(-2*time_var.g_map+7*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                    +JointState.h_hand_ang_acc_q_e_init_left(2)*(1-9*time_var.g_map+18*pow(time_var.g_map,2)-10*pow(time_var.g_map,3))
                    +JointState.h_hand_ang_acc_q_e_end_left(2)*(3*time_var.g_map-12*pow(time_var.g_map,2)+10*pow(time_var.g_map,3))
                    +(2/(pow(JointState.period_T,2)*time_var.tb*(1-time_var.tb)))*error_b_tot_left(5)*
                     (-pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)
                     +pow(M_PI,2)*pow(time_var.phi,2)*(1-time_var.g_map)*pow(time_var.g_map,2*time_var.phi-1)*cos(2*M_PI*pow(time_var.g_map,time_var.phi))
                     +(1-2*time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi-1)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

          h_acc_q_w_left = (60/pow(JointState.period_T,2))*error_var.error_f_tot_left(6)*(time_var.g_map-3*pow(time_var.g_map,2)+2*pow(time_var.g_map,3))
                +(12/JointState.period_T)*JointState.h_hand_ang_vel_q_w_init_left*(-3*time_var.g_map+8*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                +(12/JointState.period_T)*JointState.h_hand_ang_vel_q_w_end_left*(-2*time_var.g_map+7*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
                +JointState.h_hand_ang_acc_q_w_init_left*(1-9*time_var.g_map+18*pow(time_var.g_map,2)-10*pow(time_var.g_map,3))
                +JointState.h_hand_ang_acc_q_w_end_left*(3*time_var.g_map-12*pow(time_var.g_map,2)+10*pow(time_var.g_map,3))
                +(2/(pow(JointState.period_T,2)*time_var.tb*(1-time_var.tb)))*error_b_tot_left(6)*
                 (-pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)
                 +pow(M_PI,2)*pow(time_var.phi,2)*(1-time_var.g_map)*pow(time_var.g_map,2*time_var.phi-1)*cos(2*M_PI*pow(time_var.g_map,time_var.phi))
                 +(1-2*time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi-1)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));

                Vector3d h_ang_acc_left;
                h_ang_acc_left = 2*(h_or_q_e_left.cross(h_acc_q_e_left)+h_or_q_w_left*h_acc_q_e_left-h_acc_q_w_left*h_or_q_e_left);
                des_pose.h_hand_acc_left(3) = h_ang_acc_left(0);
                des_pose.h_hand_acc_left(4) = h_ang_acc_left(1);
                des_pose.h_hand_acc_left(5) = h_ang_acc_left(2);



          // human-like desired finger acceleration
          des_pose.h_fing_acc_left(0) = (60/pow(JointState.period_T,2))*error_var.error_f_fing_tot_left(0)*(time_var.g_map-3*pow(time_var.g_map,2)+2*pow(time_var.g_map,3))
              +(12/JointState.period_T)*JointState.jointsInitVelocity_hand_left(0)*(-3*time_var.g_map+8*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
              +(12/JointState.period_T)*JointState.jointsFinalVelocity_hand_left(0)*(-2*time_var.g_map+7*pow(time_var.g_map,2)-5*pow(time_var.g_map,3))
              +JointState.jointsInitAcceleration_hand_left(0)*(1-9*time_var.g_map+18*pow(time_var.g_map,2)-10*pow(time_var.g_map,3))
              +JointState.jointsFinalAcceleration_hand_left(0)*(3*time_var.g_map-12*pow(time_var.g_map,2)+10*pow(time_var.g_map,3))
              +(2/(pow(JointState.period_T,2)*time_var.tb*(1-time_var.tb)))*error_b_fing_tot_left(0)*
                  (-pow(sin(M_PI*pow(time_var.g_map,time_var.phi)),2)
                  +pow(M_PI,2)*pow(time_var.phi,2)*(1-time_var.g_map)*pow(time_var.g_map,2*time_var.phi-1)*cos(2*M_PI*pow(time_var.g_map,time_var.phi))
                  +(1-2*time_var.g_map)*M_PI*time_var.phi*pow(time_var.g_map,time_var.phi-1)*sin(2*M_PI*pow(time_var.g_map,time_var.phi)));
        }

    }else if(stage_descr.compare("approach")==0){

        ctx.mTimeMapdlg->getApproachTimeMapping(tau,dec_rate,diff_w);
        // normalized mapped time
        if(usr_interface_input.sim_robot){
            time_var.g_map = 1 - exp((-dec_rate*ctx.curr_time)/(tau*(1+diff_w* error_var.error_tot.squaredNorm())));
        }else{
            double d_curr_time = (boost::chrono::duration_cast<msec>(ctx.curr_time_ctrl-ctx.start_time_point)).count();
            time_var.g_map = 1 - exp((-dec_rate*(d_curr_time/1000))/(tau*(1+diff_w* error_var.error_tot.squaredNorm())));
        }
        time_var.index = static_cast<int>(0.5+(JointState.n_steps-1)*time_var.g_map);
        // desired swivel angle references
        error_var.des_alpha_pos = JointState.alpha_positions.at(time_var.index);
        error_var.des_alpha_vel = JointState.alpha_velocities.at(time_var.index);
        error_var.des_alpha_acc = JointState.alpha_accelerations.at(time_var.index);

        // position
        human_likeness_coef.hl_p_x_pos_coeff = human_likeness_coef.hl_p_x_pos_coeff_app;
        human_likeness_coef.hl_p_y_pos_coeff = human_likeness_coef.hl_p_y_pos_coeff_app;
        human_likeness_coef.hl_p_z_pos_coeff = human_likeness_coef.hl_p_z_pos_coeff_app;
        human_likeness_coef.hl_p_x_or_coeff = human_likeness_coef.hl_p_x_or_coeff_app;
        human_likeness_coef.hl_p_y_or_coeff = human_likeness_coef.hl_p_y_or_coeff_app;
        human_likeness_coef.hl_p_z_or_coeff = human_likeness_coef.hl_p_z_or_coeff_app;
        // velocity
        human_likeness_coef.hl_d_x_pos_coeff = human_likeness_coef.hl_d_x_pos_coeff_app;
        human_likeness_coef.hl_d_y_pos_coeff = human_likeness_coef.hl_d_y_pos_coeff_app;
        human_likeness_coef.hl_d_z_pos_coeff = human_likeness_coef.hl_d_z_pos_coeff_app;
        human_likeness_coef.hl_d_x_or_coeff = human_likeness_coef.hl_d_x_or_coeff_app;
        human_likeness_coef.hl_d_y_or_coeff = human_likeness_coef.hl_d_y_or_coeff_app;
        human_likeness_coef.hl_d_z_or_coeff = human_likeness_coef.hl_d_z_or_coeff_app;

        // human-like desired hand pose
        des_pose.h_hand_pose(0) = des_pose.hand_pos_init_vec(0) + 0.25*error_var.error_f_tot(0)*(5*time_var.g_map-pow(time_var.g_map,5));
        des_pose.h_hand_pose(1) = des_pose.hand_pos_init_vec(1) + 0.25*error_var.error_f_tot(1)*(5*time_var.g_map-pow(time_var.g_map,5));
        des_pose.h_hand_pose(2) = des_pose.hand_pos_init_vec(2) + 0.25*error_var.error_f_tot(2)*(5*time_var.g_map-pow(time_var.g_map,5));
        Vector3d h_or_q_e; double h_or_q_w;
        h_or_q_e(0) = des_pose.hand_or_q_e_init_vec(0) + 0.25*error_var.error_f_tot(3)*(5*time_var.g_map-pow(time_var.g_map,5));
        h_or_q_e(1) = des_pose.hand_or_q_e_init_vec(1) + 0.25*error_var.error_f_tot(4)*(5*time_var.g_map-pow(time_var.g_map,5));
        h_or_q_e(2) = des_pose.hand_or_q_e_init_vec(2) + 0.25*error_var.error_f_tot(5)*(5*time_var.g_map-pow(time_var.g_map,5));
        h_or_q_w = des_pose.hand_or_q_w_init_vec + 0.25*error_var.error_f_tot(6)*(5*time_var.g_map-pow(time_var.g_map,5));
        des_pose.h_hand_pose(3) = h_or_q_e(0);
        des_pose.h_hand_pose(4) = h_or_q_e(1);
        des_pose.h_hand_pose(5) = h_or_q_e(2);
        des_pose.h_hand_pose(6) = h_or_q_w;

        // human-like desired finger position
        des_pose.h_fing_pos(0) = JointState.jointsInitPosition_hand(0) + 0.25*error_var.error_f_fing_tot(0)*(5*time_var.g_map-pow(time_var.g_map,5));

        // human-like desired hand velocity
        des_pose.h_hand_vel(0) = (5/(4*JointState.period_T))*error_var.error_f_tot(0)*(1-pow(time_var.g_map,4));
        des_pose.h_hand_vel(1) = (5/(4*JointState.period_T))*error_var.error_f_tot(1)*(1-pow(time_var.g_map,4));
        des_pose.h_hand_vel(2) = (5/(4*JointState.period_T))*error_var.error_f_tot(2)*(1-pow(time_var.g_map,4));
        Vector3d h_vel_q_e; double h_vel_q_w;
        h_vel_q_e(0) = (5/(4*JointState.period_T))*error_var.error_f_tot(3)*(1-pow(time_var.g_map,4));
        h_vel_q_e(1) = (5/(4*JointState.period_T))*error_var.error_f_tot(4)*(1-pow(time_var.g_map,4));
        h_vel_q_e(2) = (5/(4*JointState.period_T))*error_var.error_f_tot(5)*(1-pow(time_var.g_map,4));
        h_vel_q_w = (5/(4*JointState.period_T))*error_var.error_f_tot(6)*(1-pow(time_var.g_map,4));
        Vector3d h_ang_vel;
        h_ang_vel = 2*(h_or_q_e.cross(h_vel_q_e)+h_or_q_w*h_vel_q_e-h_vel_q_w*h_or_q_e);
        des_pose.h_hand_vel(3) = h_ang_vel(0);
        des_pose.h_hand_vel(4) = h_ang_vel(1);
        des_pose.h_hand_vel(5) = h_ang_vel(2);

        // human-like desired joints velocity
        des_pose.h_fing_vel(0) = des_pose.h_fing_vel(0) = (5/(4*JointState.period_T))*error_var.error_f_fing_tot(0)*(1-pow(time_var.g_map,4));


        // human-like desired hand acceleration
        des_pose.h_hand_acc(0) = -(5/pow(JointState.period_T,2))*error_var.error_f_tot(0)*pow(time_var.g_map,3);
        des_pose.h_hand_acc(1) = -(5/pow(JointState.period_T,2))*error_var.error_f_tot(1)*pow(time_var.g_map,3);
        des_pose.h_hand_acc(2) = -(5/pow(JointState.period_T,2))*error_var.error_f_tot(2)*pow(time_var.g_map,3);
        Vector3d h_acc_q_e; double h_acc_q_w;
        h_acc_q_e(0) = -(5/pow(JointState.period_T,2))*error_var.error_f_tot(3)*pow(time_var.g_map,3);
        h_acc_q_e(1) = -(5/pow(JointState.period_T,2))*error_var.error_f_tot(4)*pow(time_var.g_map,3);
        h_acc_q_e(2) = -(5/pow(JointState.period_T,2))*error_var.error_f_tot(5)*pow(time_var.g_map,3);
        h_acc_q_w = -(5/pow(JointState.period_T,2))*error_var.error_f_tot(6)*pow(time_var.g_map,3);
        Vector3d h_ang_acc;
        h_ang_acc = 2*(h_or_q_e.cross(h_acc_q_e)+h_or_q_w*h_acc_q_e-h_acc_q_w*h_or_q_e);
        des_pose.h_hand_acc(3) = h_ang_acc(0);
        des_pose.h_hand_acc(4) = h_ang_acc(1);
        des_pose.h_hand_acc(5) = h_ang_acc(2);

        // human-like desired finger acceleration
        des_pose.h_fing_acc(0) = -(5/pow(JointState.period_T,2))*error_var.error_f_fing_tot(0)*pow(time_var.g_map,3);


        if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
          // desired swivel angle references
          error_var.des_alpha_pos_left = JointState.alpha_positions_left.at(time_var.index);
          error_var.des_alpha_vel_left = JointState.alpha_velocities_left.at(time_var.index);
          error_var.des_alpha_acc_left = JointState.alpha_accelerations_left.at(time_var.index);


          // human-like desired hand pose
          des_pose.h_hand_pose_left(0) = des_pose.hand_pos_init_vec_left(0) + 0.25*error_var.error_f_tot_left(0)*(5*time_var.g_map-pow(time_var.g_map,5));
          des_pose.h_hand_pose_left(1) = des_pose.hand_pos_init_vec_left(1) + 0.25*error_var.error_f_tot_left(1)*(5*time_var.g_map-pow(time_var.g_map,5));
          des_pose.h_hand_pose_left(2) = des_pose.hand_pos_init_vec_left(2) + 0.25*error_var.error_f_tot_left(2)*(5*time_var.g_map-pow(time_var.g_map,5));
          Vector3d h_or_q_e_left; double h_or_q_w_left;
          h_or_q_e_left(0) = des_pose.hand_or_q_e_init_vec_left(0) + 0.25*error_var.error_f_tot_left(3)*(5*time_var.g_map-pow(time_var.g_map,5));
          h_or_q_e_left(1) = des_pose.hand_or_q_e_init_vec_left(1) + 0.25*error_var.error_f_tot_left(4)*(5*time_var.g_map-pow(time_var.g_map,5));
          h_or_q_e_left(2) = des_pose.hand_or_q_e_init_vec_left(2) + 0.25*error_var.error_f_tot_left(5)*(5*time_var.g_map-pow(time_var.g_map,5));
          h_or_q_w_left = des_pose.hand_or_q_w_init_vec_left + 0.25*error_var.error_f_tot_left(6)*(5*time_var.g_map-pow(time_var.g_map,5));
          des_pose.h_hand_pose_left(3) = h_or_q_e_left(0);
          des_pose.h_hand_pose_left(4) = h_or_q_e_left(1);
          des_pose.h_hand_pose_left(5) = h_or_q_e_left(2);
          des_pose.h_hand_pose_left(6) = h_or_q_w_left;

          // human-like desired finger position
          des_pose.h_fing_pos_left(0) = JointState.jointsInitPosition_hand_left(0) + 0.25*error_var.error_f_fing_tot_left(0)*(5*time_var.g_map-pow(time_var.g_map,5));

          // human-like desired hand velocity
          des_pose.h_hand_vel_left(0) = (5/(4*JointState.period_T))*error_var.error_f_tot_left(0)*(1-pow(time_var.g_map,4));
          des_pose.h_hand_vel_left(1) = (5/(4*JointState.period_T))*error_var.error_f_tot_left(1)*(1-pow(time_var.g_map,4));
          des_pose.h_hand_vel_left(2) = (5/(4*JointState.period_T))*error_var.error_f_tot_left(2)*(1-pow(time_var.g_map,4));
          Vector3d h_vel_q_e_left; double h_vel_q_w_left;
          h_vel_q_e_left(0) = (5/(4*JointState.period_T))*error_var.error_f_tot_left(3)*(1-pow(time_var.g_map,4));
          h_vel_q_e_left(1) = (5/(4*JointState.period_T))*error_var.error_f_tot_left(4)*(1-pow(time_var.g_map,4));
          h_vel_q_e_left(2) = (5/(4*JointState.period_T))*error_var.error_f_tot_left(5)*(1-pow(time_var.g_map,4));
          h_vel_q_w_left = (5/(4*JointState.period_T))*error_var.error_f_tot_left(6)*(1-pow(time_var.g_map,4));
          Vector3d h_ang_vel_left;
          h_ang_vel_left = 2*(h_or_q_e_left.cross(h_vel_q_e_left)+h_or_q_w_left*h_vel_q_e_left-h_vel_q_w_left*h_or_q_e_left);
          des_pose.h_hand_vel_left(3) = h_ang_vel_left(0);
          des_pose.h_hand_vel_left(4) = h_ang_vel_left(1);
          des_pose.h_hand_vel_left(5) = h_ang_vel_left(2);

          // human-like desired joints velocity
          des_pose.h_fing_vel_left(0) = des_pose.h_fing_vel_left(0) = (5/(4*JointState.period_T))*error_var.error_f_fing_tot_left(0)*(1-pow(time_var.g_map,4));


          // human-like desired hand acceleration
          des_pose.h_hand_acc_left(0) = -(5/pow(JointState.period_T,2))*error_var.error_f_tot_left(0)*pow(time_var.g_map,3);
          des_pose.h_hand_acc_left(1) = -(5/pow(JointState.period_T,2))*error_var.error_f_tot_left(1)*pow(time_var.g_map,3);
          des_pose.h_hand_acc_left(2) = -(5/pow(JointState.period_T,2))*error_var.error_f_tot_left(2)*pow(time_var.g_map,3);
          Vector3d h_acc_q_e_left; double h_acc_q_w_left;
          h_acc_q_e_left(0) = -(5/pow(JointState.period_T,2))*error_var.error_f_tot_left(3)*pow(time_var.g_map,3);
          h_acc_q_e_left(1) = -(5/pow(JointState.period_T,2))*error_var.error_f_tot_left(4)*pow(time_var.g_map,3);
          h_acc_q_e_left(2) = -(5/pow(JointState.period_T,2))*error_var.error_f_tot_left(5)*pow(time_var.g_map,3);
          h_acc_q_w_left = -(5/pow(JointState.period_T,2))*error_var.error_f_tot_left(6)*pow(time_var.g_map,3);
          Vector3d h_ang_acc_left;
          h_ang_acc_left = 2*(h_or_q_e_left.cross(h_acc_q_e_left)+h_or_q_w_left*h_acc_q_e_left-h_acc_q_w_left*h_or_q_e_left);
          des_pose.h_hand_acc_left(3) = h_ang_acc_left(0);
          des_pose.h_hand_acc_left(4) = h_ang_acc_left(1);
          des_pose.h_hand_acc_left(5) = h_ang_acc_left(2);

          // human-like desired finger acceleration
          des_pose.h_fing_acc_left(0) = -(5/pow(JointState.period_T,2))*error_var.error_f_fing_tot_left(0)*pow(time_var.g_map,3);
        }

    }else if(stage_descr.compare("retreat")==0){

    ctx.mTimeMapdlg->getRetreatTimeMapping(tau,dec_rate,diff_w);
    // normalized mapped time
    if(usr_interface_input.sim_robot){
        time_var.g_map = 1 - exp((-dec_rate*ctx.curr_time)/(tau*(1+diff_w* error_var.error_tot.squaredNorm())));
    }else{
        double d_curr_time = (boost::chrono::duration_cast<msec>(ctx.curr_time_ctrl-ctx.start_time_point)).count();
        time_var.g_map = 1 - exp((-dec_rate*(d_curr_time/1000))/(tau*(1+diff_w* error_var.error_tot.squaredNorm())));
    }
    time_var.index = static_cast<int>(0.5+(JointState.n_steps-1)*time_var.g_map);
    // desired swivel angle references
    error_var.des_alpha_pos = JointState.alpha_positions.at(time_var.index);
    error_var.des_alpha_vel = JointState.alpha_velocities.at(time_var.index);
    error_var.des_alpha_acc = JointState.alpha_accelerations.at(time_var.index);

    // position
    human_likeness_coef.hl_p_x_pos_coeff = human_likeness_coef.hl_p_x_pos_coeff_ret;
    human_likeness_coef.hl_p_y_pos_coeff = human_likeness_coef.hl_p_y_pos_coeff_ret;
    human_likeness_coef.hl_p_z_pos_coeff = human_likeness_coef.hl_p_z_pos_coeff_ret;
    human_likeness_coef.hl_p_x_or_coeff = human_likeness_coef.hl_p_x_or_coeff_ret;
    human_likeness_coef.hl_p_y_or_coeff = human_likeness_coef.hl_p_y_or_coeff_ret;
    human_likeness_coef.hl_p_z_or_coeff = human_likeness_coef.hl_p_z_or_coeff_ret;
    // velocity
    human_likeness_coef.hl_d_x_pos_coeff = human_likeness_coef.hl_d_x_pos_coeff_ret;
    human_likeness_coef.hl_d_y_pos_coeff = human_likeness_coef.hl_d_y_pos_coeff_ret;
    human_likeness_coef.hl_d_z_pos_coeff = human_likeness_coef.hl_d_z_pos_coeff_ret;
    human_likeness_coef.hl_d_x_or_coeff = human_likeness_coef.hl_d_x_or_coeff_ret;
    human_likeness_coef.hl_d_y_or_coeff = human_likeness_coef.hl_d_y_or_coeff_ret;
    human_likeness_coef.hl_d_z_or_coeff = human_likeness_coef.hl_d_z_or_coeff_ret;

    // human-like desired hand pose
    des_pose.h_hand_pose(0) = des_pose.hand_pos_init_vec(0) + 0.33*error_var.error_f_tot(0)*(5*pow(time_var.g_map,4)-2*pow(time_var.g_map,5));
    des_pose.h_hand_pose(1) = des_pose.hand_pos_init_vec(1) + 0.33*error_var.error_f_tot(1)*(5*pow(time_var.g_map,4)-2*pow(time_var.g_map,5));
    des_pose.h_hand_pose(2) = des_pose.hand_pos_init_vec(2) + 0.33*error_var.error_f_tot(2)*(5*pow(time_var.g_map,4)-2*pow(time_var.g_map,5));
    Vector3d h_or_q_e; double h_or_q_w;
    h_or_q_e(0) = des_pose.hand_or_q_e_init_vec(0) + 0.33*error_var.error_f_tot(3)*(5*pow(time_var.g_map,4)-2*pow(time_var.g_map,5));
    h_or_q_e(1) = des_pose.hand_or_q_e_init_vec(1) + 0.33*error_var.error_f_tot(4)*(5*pow(time_var.g_map,4)-2*pow(time_var.g_map,5));
    h_or_q_e(2) = des_pose.hand_or_q_e_init_vec(2) + 0.33*error_var.error_f_tot(5)*(5*pow(time_var.g_map,4)-2*pow(time_var.g_map,5));
    h_or_q_w = des_pose.hand_or_q_w_init_vec + 0.33*error_var.error_f_tot(6)*(5*pow(time_var.g_map,4)-2*pow(time_var.g_map,5));
    des_pose.h_hand_pose(3) = h_or_q_e(0);
    des_pose.h_hand_pose(4) = h_or_q_e(1);
    des_pose.h_hand_pose(5) = h_or_q_e(2);
    des_pose.h_hand_pose(6) = h_or_q_w;

    // human-like desired finger position
    des_pose.h_fing_pos(0) = JointState.jointsInitPosition_hand(0) + 0.33*error_var.error_f_fing_tot(0)*(5*pow(time_var.g_map,4)-2*pow(time_var.g_map,5));


    // human-like desired hand velocity
    des_pose.h_hand_vel(0) = (10/(3*JointState.period_T))*error_var.error_f_tot(0)*(2*pow(time_var.g_map,3)-pow(time_var.g_map,4));
    des_pose.h_hand_vel(1) = (10/(3*JointState.period_T))*error_var.error_f_tot(1)*(2*pow(time_var.g_map,3)-pow(time_var.g_map,4));
    des_pose.h_hand_vel(2) = (10/(3*JointState.period_T))*error_var.error_f_tot(2)*(2*pow(time_var.g_map,3)-pow(time_var.g_map,4));
    Vector3d h_vel_q_e; double h_vel_q_w;
    h_vel_q_e(0) = (10/(3*JointState.period_T))*error_var.error_f_tot(3)*(2*pow(time_var.g_map,3)-pow(time_var.g_map,4));
    h_vel_q_e(1) = (10/(3*JointState.period_T))*error_var.error_f_tot(4)*(2*pow(time_var.g_map,3)-pow(time_var.g_map,4));
    h_vel_q_e(2) = (10/(3*JointState.period_T))*error_var.error_f_tot(5)*(2*pow(time_var.g_map,3)-pow(time_var.g_map,4));
    h_vel_q_w = (10/(3*JointState.period_T))*error_var.error_f_tot(6)*(2*pow(time_var.g_map,3)-pow(time_var.g_map,4));
    Vector3d h_ang_vel;
    h_ang_vel = 2*(h_or_q_e.cross(h_vel_q_e)+h_or_q_w*h_vel_q_e-h_vel_q_w*h_or_q_e);
    des_pose.h_hand_vel(3) = h_ang_vel(0);
    des_pose.h_hand_vel(4) = h_ang_vel(1);
    des_pose.h_hand_vel(5) = h_ang_vel(2);

    // human-like desired finger velocity
    des_pose.h_fing_vel(0) = (10/(3*JointState.period_T))*error_var.error_f_fing_tot(0)*(2*pow(time_var.g_map,3)-pow(time_var.g_map,4));


    // human-like desired hand acceleration
    des_pose.h_hand_acc(0) = (20/(3*pow(JointState.period_T,2)))*error_var.error_f_tot(0)*(3*pow(time_var.g_map,2)-2*pow(time_var.g_map,3));
    des_pose.h_hand_acc(1) = (20/(3*pow(JointState.period_T,2)))*error_var.error_f_tot(1)*(3*pow(time_var.g_map,2)-2*pow(time_var.g_map,3));
    des_pose.h_hand_acc(2) = (20/(3*pow(JointState.period_T,2)))*error_var.error_f_tot(2)*(3*pow(time_var.g_map,2)-2*pow(time_var.g_map,3));
    Vector3d h_acc_q_e; double h_acc_q_w;
    h_acc_q_e(0) = (20/(3*pow(JointState.period_T,2)))*error_var.error_f_tot(3)*(3*pow(time_var.g_map,2)-2*pow(time_var.g_map,3));
    h_acc_q_e(1) = (20/(3*pow(JointState.period_T,2)))*error_var.error_f_tot(4)*(3*pow(time_var.g_map,2)-2*pow(time_var.g_map,3));
    h_acc_q_e(2) = (20/(3*pow(JointState.period_T,2)))*error_var.error_f_tot(5)*(3*pow(time_var.g_map,2)-2*pow(time_var.g_map,3));
    h_acc_q_w = (20/(3*pow(JointState.period_T,2)))*error_var.error_f_tot(6)*(3*pow(time_var.g_map,2)-2*pow(time_var.g_map,3));
    Vector3d h_ang_acc;
    h_ang_acc = 2*(h_or_q_e.cross(h_acc_q_e)+h_or_q_w*h_acc_q_e-h_acc_q_w*h_or_q_e);
    des_pose.h_hand_acc(3) = h_ang_acc(0);
    des_pose.h_hand_acc(4) = h_ang_acc(1);
    des_pose.h_hand_acc(5) = h_ang_acc(2);

    // human-like desired finger acceleration
    des_pose.h_fing_acc(0) = (20/(3*pow(JointState.period_T,2)))*error_var.error_f_fing_tot(0)*(3*pow(time_var.g_map,2)-2*pow(time_var.g_map,3));


    if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
      error_var.des_alpha_pos_left = JointState.alpha_positions_left.at(time_var.index);
      error_var.des_alpha_vel_left = JointState.alpha_velocities_left.at(time_var.index);
      error_var.des_alpha_acc_left = JointState.alpha_accelerations_left.at(time_var.index);

      // human-like desired hand pose
      des_pose.h_hand_pose_left(0) = des_pose.hand_pos_init_vec_left(0) + 0.33*error_var.error_f_tot_left(0)*(5*pow(time_var.g_map,4)-2*pow(time_var.g_map,5));
      des_pose.h_hand_pose_left(1) = des_pose.hand_pos_init_vec_left(1) + 0.33*error_var.error_f_tot_left(1)*(5*pow(time_var.g_map,4)-2*pow(time_var.g_map,5));
      des_pose.h_hand_pose_left(2) = des_pose.hand_pos_init_vec_left(2) + 0.33*error_var.error_f_tot_left(2)*(5*pow(time_var.g_map,4)-2*pow(time_var.g_map,5));
      Vector3d h_or_q_e_left; double h_or_q_w_left;
      h_or_q_e_left(0) = des_pose.hand_or_q_e_init_vec_left(0) + 0.33*error_var.error_f_tot_left(3)*(5*pow(time_var.g_map,4)-2*pow(time_var.g_map,5));
      h_or_q_e_left(1) = des_pose.hand_or_q_e_init_vec_left(1) + 0.33*error_var.error_f_tot_left(4)*(5*pow(time_var.g_map,4)-2*pow(time_var.g_map,5));
      h_or_q_e_left(2) = des_pose.hand_or_q_e_init_vec_left(2) + 0.33*error_var.error_f_tot_left(5)*(5*pow(time_var.g_map,4)-2*pow(time_var.g_map,5));
      h_or_q_w_left = des_pose.hand_or_q_w_init_vec_left + 0.33*error_var.error_f_tot_left(6)*(5*pow(time_var.g_map,4)-2*pow(time_var.g_map,5));
      des_pose.h_hand_pose_left(3) = h_or_q_e_left(0);
      des_pose.h_hand_pose_left(4) = h_or_q_e_left(1);
      des_pose.h_hand_pose_left(5) = h_or_q_e_left(2);
      des_pose.h_hand_pose_left(6) = h_or_q_w_left;

      // human-like desired finger position
      des_pose.h_fing_pos_left(0) = JointState.jointsInitPosition_hand_left(0) + 0.33*error_var.error_f_fing_tot_left(0)*(5*pow(time_var.g_map,4)-2*pow(time_var.g_map,5));


      // human-like desired hand velocity
      des_pose.h_hand_vel_left(0) = (10/(3*JointState.period_T))*error_var.error_f_tot_left(0)*(2*pow(time_var.g_map,3)-pow(time_var.g_map,4));
      des_pose.h_hand_vel_left(1) = (10/(3*JointState.period_T))*error_var.error_f_tot_left(1)*(2*pow(time_var.g_map,3)-pow(time_var.g_map,4));
      des_pose.h_hand_vel_left(2) = (10/(3*JointState.period_T))*error_var.error_f_tot_left(2)*(2*pow(time_var.g_map,3)-pow(time_var.g_map,4));
      Vector3d h_vel_q_e_left; double h_vel_q_w_left;
      h_vel_q_e_left(0) = (10/(3*JointState.period_T))*error_var.error_f_tot_left(3)*(2*pow(time_var.g_map,3)-pow(time_var.g_map,4));
      h_vel_q_e_left(1) = (10/(3*JointState.period_T))*error_var.error_f_tot_left(4)*(2*pow(time_var.g_map,3)-pow(time_var.g_map,4));
      h_vel_q_e_left(2) = (10/(3*JointState.period_T))*error_var.error_f_tot_left(5)*(2*pow(time_var.g_map,3)-pow(time_var.g_map,4));
      h_vel_q_w_left = (10/(3*JointState.period_T))*error_var.error_f_tot_left(6)*(2*pow(time_var.g_map,3)-pow(time_var.g_map,4));
      Vector3d h_ang_vel_left;
      h_ang_vel_left = 2*(h_or_q_e_left.cross(h_vel_q_e_left)+h_or_q_w_left*h_vel_q_e_left-h_vel_q_w_left*h_or_q_e_left);
      des_pose.h_hand_vel_left(3) = h_ang_vel_left(0);
      des_pose.h_hand_vel_left(4) = h_ang_vel_left(1);
      des_pose.h_hand_vel_left(5) = h_ang_vel_left(2);

      // human-like desired finger velocity
      des_pose.h_fing_vel_left(0) = (10/(3*JointState.period_T))*error_var.error_f_fing_tot_left(0)*(2*pow(time_var.g_map,3)-pow(time_var.g_map,4));


      // human-like desired hand acceleration
      des_pose.h_hand_acc_left(0) = (20/(3*pow(JointState.period_T,2)))*error_var.error_f_tot_left(0)*(3*pow(time_var.g_map,2)-2*pow(time_var.g_map,3));
      des_pose.h_hand_acc_left(1) = (20/(3*pow(JointState.period_T,2)))*error_var.error_f_tot_left(1)*(3*pow(time_var.g_map,2)-2*pow(time_var.g_map,3));
      des_pose.h_hand_acc_left(2) = (20/(3*pow(JointState.period_T,2)))*error_var.error_f_tot_left(2)*(3*pow(time_var.g_map,2)-2*pow(time_var.g_map,3));
      Vector3d h_acc_q_e_left; double h_acc_q_w_left;
      h_acc_q_e_left(0) = (20/(3*pow(JointState.period_T,2)))*error_var.error_f_tot_left(3)*(3*pow(time_var.g_map,2)-2*pow(time_var.g_map,3));
      h_acc_q_e_left(1) = (20/(3*pow(JointState.period_T,2)))*error_var.error_f_tot_left(4)*(3*pow(time_var.g_map,2)-2*pow(time_var.g_map,3));
      h_acc_q_e_left(2) = (20/(3*pow(JointState.period_T,2)))*error_var.error_f_tot_left(5)*(3*pow(time_var.g_map,2)-2*pow(time_var.g_map,3));
      h_acc_q_w_left = (20/(3*pow(JointState.period_T,2)))*error_var.error_f_tot_left(6)*(3*pow(time_var.g_map,2)-2*pow(time_var.g_map,3));
      Vector3d h_ang_acc_left;
      h_ang_acc_left = 2*(h_or_q_e_left.cross(h_acc_q_e_left)+h_or_q_w_left*h_acc_q_e_left-h_acc_q_w_left*h_or_q_e_left);
      des_pose.h_hand_acc_left(3) = h_ang_acc_left(0);
      des_pose.h_hand_acc_left(4) = h_ang_acc_left(1);
      des_pose.h_hand_acc_left(5) = h_ang_acc_left(2);

      // human-like desired finger acceleration
      des_pose.h_fing_acc_left(0) = (20/(3*pow(JointState.period_T,2)))*error_var.error_f_fing_tot_left(0)*(3*pow(time_var.g_map,2)-2*pow(time_var.g_map,3));

    }
  }

    // errors of the swivel angle data
    error_var.error_alpha_pos = mes_pose_str.alpha_pos_read.at(0) - error_var.des_alpha_pos;
    error_var.error_alpha_vel = mes_pose_str.alpha_vel_read.at(0) - error_var.des_alpha_vel;
    error_var.error_alpha_acc = mes_pose_str.alpha_acc_read.at(0) - error_var.des_alpha_acc;

    // hand
    // error in position
    Vector3d error_h_pos;
    error_h_pos(0) = des_pose.h_hand_pose(0) - mes_pose_str.r_hand_pos_vec(0);
    error_h_pos(1) = des_pose.h_hand_pose(1) - mes_pose_str.r_hand_pos_vec(1);
    error_h_pos(2) = des_pose.h_hand_pose(2) - mes_pose_str.r_hand_pos_vec(2);
    // error in orientation
    Quaterniond h_hand_or_q;
    h_hand_or_q.x() = des_pose.h_hand_pose(3);
    h_hand_or_q.y() = des_pose.h_hand_pose(4);
    h_hand_or_q.z() = des_pose.h_hand_pose(5);
    h_hand_or_q.w() = des_pose.h_hand_pose(6);
    Vector3d h_hand_or_q_e;
    h_hand_or_q_e << des_pose.h_hand_pose(3),des_pose.h_hand_pose(4),des_pose.h_hand_pose(5);
    double h_hand_or_q_w = des_pose.h_hand_pose(6);

    Vector3d error_h_or = mes_pose_str.r_hand_q_w * h_hand_or_q_e - h_hand_or_q_w * mes_pose_str.r_hand_or_q_e - h_hand_or_q_e.cross(mes_pose_str.r_hand_or_q_e);
    // total error in position + orientation
    error_var.error_h_tot << error_h_pos(0),error_h_pos(1),error_h_pos(2), error_h_or(0),error_h_or(1),error_h_or(2);

    // fingers
    error_var.error_h_fing_tot(0) = des_pose.h_fing_pos(0) - mes_pose_str.r_hand_posture.at(0);

    // hand
    // error in linear velocity
    Vector3d error_h_lin_vel;
    error_h_lin_vel(0) = des_pose.h_hand_vel(0) - mes_pose_str.r_hand_lin_vel_vec(0);
    error_h_lin_vel(1) = des_pose.h_hand_vel(1) - mes_pose_str.r_hand_lin_vel_vec(1);
    error_h_lin_vel(2) = des_pose.h_hand_vel(2) - mes_pose_str.r_hand_lin_vel_vec(2);

    // error in angular velocity (omega)
    Vector3d error_h_der_or;
    error_h_der_or(0) = des_pose.h_hand_vel(3) - mes_pose_str.r_hand_ang_vel_vec(0);
    error_h_der_or(1) = des_pose.h_hand_vel(4) - mes_pose_str.r_hand_ang_vel_vec(1);
    error_h_der_or(2) = des_pose.h_hand_vel(5) - mes_pose_str.r_hand_ang_vel_vec(2);

    // total error in velocity
    error_var.der_error_h_tot << error_h_lin_vel(0),error_h_lin_vel(1),error_h_lin_vel(2),error_h_der_or(0),error_h_der_or(1),error_h_der_or(2);
    // fingers
    error_var.der_error_h_fing_tot(0) = des_pose.h_fing_vel(0) - mes_pose_str.r_hand_velocities_read.at(0);

    // human-like reference hand acceleration
    error_var.h_hand_ref_acc(0) = des_pose.h_hand_acc(0);
    error_var.h_hand_ref_acc(1) = des_pose.h_hand_acc(1);
    error_var.h_hand_ref_acc(2) = des_pose.h_hand_acc(2);
    error_var.h_hand_ref_acc(3) = des_pose.h_hand_acc(3);
    error_var.h_hand_ref_acc(4) = des_pose.h_hand_acc(4);
    error_var.h_hand_ref_acc(5) = des_pose.h_hand_acc(5);


    // human-like reference fingers acceleration
    error_var.h_fing_ref_acc(0) = des_pose.h_fing_acc(0);


    // finger control !!!
    MatrixXd Kp_fing = MatrixXd::Identity(JOINTS_HAND,JOINTS_HAND);
    MatrixXd Kd_fing = MatrixXd::Identity(JOINTS_HAND,JOINTS_HAND);
    Kp_fing(0,0) = human_likeness_coef.fing_p_coeff;
    Kd_fing(0,0) = human_likeness_coef.fing_d_coeff;

    VectorXd r_fing_acc_read_vec = error_var.h_fing_ref_acc + Kd_fing*error_var.der_error_h_fing_tot + Kp_fing*error_var.error_h_fing_tot;
    VectorXd r_hand_velocities_vec = r_fing_acc_read_vec * time_var.time_step;
    if(mov_type==0 && stage_descr.compare("retreat")==0){
      // retreat stage of a pick movement
      r_hand_velocities_vec = VectorXd::Zero(JOINTS_HAND);
    }
    VectorXd::Map(&mes_pose_str.r_hand_velocities[0], r_hand_velocities_vec.size()) = r_hand_velocities_vec;

    //------ Dual Control ----------
    if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
      // errors of the swivel angle data
      error_var.error_alpha_pos_left = mes_pose_str.alpha_pos_read_left.at(0) - error_var.des_alpha_pos_left;
      error_var.error_alpha_vel_left = mes_pose_str.alpha_vel_read_left.at(0) - error_var.des_alpha_vel_left;
      error_var.error_alpha_acc_left = mes_pose_str.alpha_acc_read_left.at(0) - error_var.des_alpha_acc_left;

      // hand
      // error in position
      Vector3d error_h_pos_left;
      error_h_pos_left(0) = des_pose.h_hand_pose_left(0) - mes_pose_str.l_hand_pos_vec(0);
      error_h_pos_left(1) = des_pose.h_hand_pose_left(1) - mes_pose_str.l_hand_pos_vec(1);
      error_h_pos_left(2) = des_pose.h_hand_pose_left(2) - mes_pose_str.l_hand_pos_vec(2);
      // error in orientation
      Quaterniond h_hand_or_q_left;
      h_hand_or_q_left.x() = des_pose.h_hand_pose_left(3);
      h_hand_or_q_left.y() = des_pose.h_hand_pose_left(4);
      h_hand_or_q_left.z() = des_pose.h_hand_pose_left(5);
      h_hand_or_q_left.w() = des_pose.h_hand_pose_left(6);
      Vector3d h_hand_or_q_e_left;
      h_hand_or_q_e_left << des_pose.h_hand_pose_left(3),des_pose.h_hand_pose_left(4),des_pose.h_hand_pose_left(5);
      double h_hand_or_q_w_left = des_pose.h_hand_pose_left(6);

      Vector3d error_h_or_left = mes_pose_str.l_hand_q_w* h_hand_or_q_e_left - h_hand_or_q_w_left * mes_pose_str.l_hand_or_q_e - h_hand_or_q_e_left.cross(mes_pose_str.l_hand_or_q_e);
      // total error in position + orientation
      error_var.error_h_tot_left << error_h_pos_left(0),error_h_pos_left(1),error_h_pos_left(2), error_h_or_left(0),error_h_or_left(1),error_h_or_left(2);

      // fingers
      error_var.error_h_fing_tot_left(0) = des_pose.h_fing_pos_left(0) - mes_pose_str.l_hand_posture.at(0);

      // hand
      // error in linear velocity
      Vector3d error_h_lin_vel_left;
      error_h_lin_vel_left(0) = des_pose.h_hand_vel_left(0) - mes_pose_str.l_hand_lin_vel_vec(0);
      error_h_lin_vel_left(1) = des_pose.h_hand_vel_left(1) - mes_pose_str.l_hand_lin_vel_vec(1);
      error_h_lin_vel_left(2) = des_pose.h_hand_vel_left(2) - mes_pose_str.l_hand_lin_vel_vec(2);

      // error in angular velocity (omega)
      Vector3d error_h_der_or_left;
      error_h_der_or_left(0) = des_pose.h_hand_vel_left(3) - mes_pose_str.l_hand_ang_vel_vec(0);
      error_h_der_or_left(1) = des_pose.h_hand_vel_left(4) - mes_pose_str.l_hand_ang_vel_vec(1);
      error_h_der_or_left(2) = des_pose.h_hand_vel_left(5) - mes_pose_str.l_hand_ang_vel_vec(2);

      // total error in velocity
      error_var.der_error_h_tot_left << error_h_lin_vel_left(0),error_h_lin_vel_left(1),error_h_lin_vel_left(2),error_h_der_or_left(0),error_h_der_or_left(1),error_h_der_or_left(2);
      // fingers
      error_var.der_error_h_fing_tot_left(0) = des_pose.h_fing_vel_left(0) - mes_pose_str.l_hand_velocities_read.at(0);

      // human-like reference hand acceleration
      error_var.h_hand_ref_acc_left(0) = des_pose.h_hand_acc_left(0);
      error_var.h_hand_ref_acc_left(1) = des_pose.h_hand_acc_left(1);
      error_var.h_hand_ref_acc_left(2) = des_pose.h_hand_acc_left(2);
      error_var.h_hand_ref_acc_left(3) = des_pose.h_hand_acc_left(3);
      error_var.h_hand_ref_acc_left(4) = des_pose.h_hand_acc_left(4);
      error_var.h_hand_ref_acc_left(5) = des_pose.h_hand_acc_left(5);


      // human-like reference fingers acceleration
      error_var.h_fing_ref_acc_left(0) = des_pose.h_fing_acc_left(0);


      // finger control !!!
      MatrixXd Kp_fing = MatrixXd::Identity(JOINTS_HAND,JOINTS_HAND);
      MatrixXd Kd_fing = MatrixXd::Identity(JOINTS_HAND,JOINTS_HAND);
      Kp_fing(0,0) = human_likeness_coef.fing_p_coeff;
      Kd_fing(0,0) = human_likeness_coef.fing_d_coeff;

      VectorXd l_fing_acc_read_vec = error_var.h_fing_ref_acc_left + Kd_fing*error_var.der_error_h_fing_tot_left + Kp_fing*error_var.error_h_fing_tot_left;
      VectorXd l_hand_velocities_vec = l_fing_acc_read_vec * time_var.time_step;
      if(mov_type==0 && stage_descr.compare("retreat")==0){
        // retreat stage of a pick movement
        l_hand_velocities_vec = VectorXd::Zero(JOINTS_HAND);
      }
      VectorXd::Map(&mes_pose_str.l_hand_velocities[0], l_hand_velocities_vec.size()) = l_hand_velocities_vec;
    }

}

/**********************************************
*	Trapezoidal velocity profile                *
* Substitutes the human_like velocity profile	*
* for a trapezoidal velocity profile, this  	*
* represents a lower computational burden for *
* the controller                              *
*********************************************/
void LocalPlanner::exec_trapezoidal_vel_profile() {
  time_var.g_map_th_pa = ui.lineEdit_g_th_plan_app->text().toDouble();
  if(usr_interface_input.follow_tar){
      time_var.g_map_th_rp = 2; // never go to retreat or to the subsequent movement
  }else{
      time_var.g_map_th_rp = ui.lineEdit_g_th_ret_plan->text().toDouble();
  }
  double thr_error = 0.0;
  double curr_t = 0.0;
  int flag = 0;
  // desired position
  // Vector3d des_hand_or; VectorXd des_hand_or_q(4); Vector3d des_hand_or_q_e;
  if(thr_error <= 3 && curr_t > 0){
    des_pose.des_hand_pos << des_pose.des_hand_pos_x,des_pose.des_hand_pos_y,des_pose.des_hand_pos_z+20;
    flag = 1;
  }
  else if(flag == 0){
    des_pose.des_hand_pos << des_pose.des_hand_pos_x,des_pose.des_hand_pos_y,des_pose.des_hand_pos_z;
  }
  mes_pose_str.des_hand_or << des_pose.des_hand_or_x,des_pose.des_hand_or_y,des_pose.des_hand_or_z;
  mes_pose_str.des_hand_or_q_e << des_pose.des_hand_or_q_x,des_pose.des_hand_or_q_y,des_pose.des_hand_or_q_z;
  mes_pose_str.des_hand_or_q << des_pose.des_hand_or_q_x,des_pose.des_hand_or_q_y,des_pose.des_hand_or_q_z,des_pose.des_hand_or_q_w;

  // trapezoidal velocity profile
  VectorXd vel_trap(JOINTS_ARM);
  VectorXd acc_trap(JOINTS_ARM);
  VectorXd vel_trap_left(JOINTS_ARM);
  VectorXd acc_trap_left(JOINTS_ARM);

  vel_trap(0) = 2*(des_pose.des_hand_pos(0)-des_pose.hand_pos_init_vec(0))/usr_interface_input.t_f_trap;
  vel_trap(1) = 2*(des_pose.des_hand_pos(1)-des_pose.hand_pos_init_vec(1))/usr_interface_input.t_f_trap;
  vel_trap(2) = 2*(des_pose.des_hand_pos(2)-des_pose.hand_pos_init_vec(2))/usr_interface_input.t_f_trap;
  vel_trap(3) = 2*(mes_pose_str.des_hand_or_q_e(0)-des_pose.hand_or_q_e_init_vec(0))/usr_interface_input.t_f_trap;
  vel_trap(4) = 2*(mes_pose_str.des_hand_or_q_e(1)-des_pose.hand_or_q_e_init_vec(1))/usr_interface_input.t_f_trap;
  vel_trap(5) = 2*(mes_pose_str.des_hand_or_q_e(2)-des_pose.hand_or_q_e_init_vec(2))/usr_interface_input.t_f_trap;
  vel_trap(6) = 2*(des_pose.des_hand_or_q_w - des_pose.hand_or_q_w_init_vec)/usr_interface_input.t_f_trap;
  acc_trap(0) = pow(vel_trap(0),2)/(des_pose.hand_pos_init_vec(0)-des_pose.des_hand_pos(0)+vel_trap(0)*usr_interface_input.t_f_trap);
  acc_trap(1) = pow(vel_trap(1),2)/(des_pose.hand_pos_init_vec(1)-des_pose.des_hand_pos(1)+vel_trap(1)*usr_interface_input.t_f_trap);
  acc_trap(2) = pow(vel_trap(2),2)/(des_pose.hand_pos_init_vec(2)-des_pose.des_hand_pos(2)+vel_trap(2)*usr_interface_input.t_f_trap);
  acc_trap(3) = pow(vel_trap(3),2)/(des_pose.hand_or_q_e_init_vec(0)-mes_pose_str.des_hand_or_q_e(0)+vel_trap(3)*usr_interface_input.t_f_trap);
  acc_trap(4) = pow(vel_trap(4),2)/(des_pose.hand_or_q_e_init_vec(1)-mes_pose_str.des_hand_or_q_e(1)+vel_trap(4)*usr_interface_input.t_f_trap);
  acc_trap(5) = pow(vel_trap(5),2)/(des_pose.hand_or_q_e_init_vec(2)-mes_pose_str.des_hand_or_q_e(2)+vel_trap(5)*usr_interface_input.t_f_trap);
  acc_trap(6) = pow(vel_trap(6),2)/(des_pose.hand_or_q_w_init_vec-des_pose.des_hand_or_q_w+vel_trap(6)*usr_interface_input.t_f_trap);

  if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
    des_pose.des_hand_pos_left << des_pose.des_hand_pos_x_left,des_pose.des_hand_pos_y_left,des_pose.des_hand_pos_z_left;
    mes_pose_str.des_hand_or_left << des_pose.des_hand_or_x_left,des_pose.des_hand_or_y_left,des_pose.des_hand_or_z_left;
    mes_pose_str.des_hand_or_q_e_left << des_pose.des_hand_or_q_x_left,des_pose.des_hand_or_q_y_left,des_pose.des_hand_or_q_z_left;
    mes_pose_str.des_hand_or_q_left << des_pose.des_hand_or_q_x_left,des_pose.des_hand_or_q_y_left,des_pose.des_hand_or_q_z_left,des_pose.des_hand_or_q_w_left;

    vel_trap_left(0) = 2*(des_pose.des_hand_pos_left(0)-des_pose.hand_pos_init_vec_left(0))/usr_interface_input.t_f_trap;
    vel_trap_left(1) = 2*(des_pose.des_hand_pos_left(1)-des_pose.hand_pos_init_vec_left(1))/usr_interface_input.t_f_trap;
    vel_trap_left(2) = 2*(des_pose.des_hand_pos_left(2)-des_pose.hand_pos_init_vec_left(2))/usr_interface_input.t_f_trap;
    vel_trap_left(3) = 2*(mes_pose_str.des_hand_or_q_e_left(0)-des_pose.hand_or_q_e_init_vec_left(0))/usr_interface_input.t_f_trap;
    vel_trap_left(4) = 2*(mes_pose_str.des_hand_or_q_e_left(1)-des_pose.hand_or_q_e_init_vec_left(1))/usr_interface_input.t_f_trap;
    vel_trap_left(5) = 2*(mes_pose_str.des_hand_or_q_e_left(2)-des_pose.hand_or_q_e_init_vec_left(2))/usr_interface_input.t_f_trap;
    vel_trap_left(6) = 2*(des_pose.des_hand_or_q_w_left - des_pose.hand_or_q_w_init_vec_left)/usr_interface_input.t_f_trap;
    acc_trap_left(0) = pow(vel_trap_left(0),2)/(des_pose.hand_pos_init_vec_left(0)-des_pose.des_hand_pos_left(0)+vel_trap_left(0)*usr_interface_input.t_f_trap);
    acc_trap_left(1) = pow(vel_trap_left(1),2)/(des_pose.hand_pos_init_vec_left(1)-des_pose.des_hand_pos_left(1)+vel_trap_left(1)*usr_interface_input.t_f_trap);
    acc_trap_left(2) = pow(vel_trap_left(2),2)/(des_pose.hand_pos_init_vec_left(2)-des_pose.des_hand_pos_left(2)+vel_trap_left(2)*usr_interface_input.t_f_trap);
    acc_trap_left(3) = pow(vel_trap_left(3),2)/(des_pose.hand_or_q_e_init_vec_left(0)-mes_pose_str.des_hand_or_q_e_left(0)+vel_trap_left(3)*usr_interface_input.t_f_trap);
    acc_trap_left(4) = pow(vel_trap_left(4),2)/(des_pose.hand_or_q_e_init_vec_left(1)-mes_pose_str.des_hand_or_q_e_left(1)+vel_trap_left(4)*usr_interface_input.t_f_trap);
    acc_trap_left(5) = pow(vel_trap_left(5),2)/(des_pose.hand_or_q_e_init_vec_left(2)-mes_pose_str.des_hand_or_q_e_left(2)+vel_trap_left(5)*usr_interface_input.t_f_trap);
    acc_trap_left(6) = pow(vel_trap_left(6),2)/(des_pose.hand_or_q_w_init_vec_left-des_pose.des_hand_or_q_w_left+vel_trap_left(6)*usr_interface_input.t_f_trap);

  }

//movimento sincrono entre maos
  double t_c_trap = usr_interface_input.t_f_trap/2;
  //double curr_t;
  if(usr_interface_input.sim_robot){
      curr_t = ctx.curr_time;
  }else{
      double d_curr_t = (boost::chrono::duration_cast<msec>(ctx.curr_time_ctrl - ctx.start_time_point)).count();
      curr_t = d_curr_t/1000;
  }

  if(curr_t <= t_c_trap)
  {
      // hand pose
      des_pose.trap_hand_pose(0) = des_pose.hand_pos_init_vec(0) + 0.5*acc_trap(0)*pow(curr_t,2);
      des_pose.trap_hand_pose(1) = des_pose.hand_pos_init_vec(1) + 0.5*acc_trap(1)*pow(curr_t,2);
      des_pose.trap_hand_pose(2) = des_pose.hand_pos_init_vec(2) + 0.5*acc_trap(2)*pow(curr_t,2);
      des_pose.trap_hand_pose(3) = des_pose.hand_or_q_e_init_vec(0) + 0.5*acc_trap(3)*pow(curr_t,2);
      des_pose.trap_hand_pose(4) = des_pose.hand_or_q_e_init_vec(1) + 0.5*acc_trap(4)*pow(curr_t,2);
      des_pose.trap_hand_pose(5) = des_pose.hand_or_q_e_init_vec(2) + 0.5*acc_trap(5)*pow(curr_t,2);
      des_pose.trap_hand_pose(6) = des_pose.hand_or_q_w_init_vec + 0.5*acc_trap(6)*pow(curr_t,2);

      // hand velocity
      des_pose.trap_hand_vel(0) = acc_trap(0)*curr_t;
      des_pose.trap_hand_vel(1) = acc_trap(1)*curr_t;
      des_pose.trap_hand_vel(2) = acc_trap(2)*curr_t;
      des_pose.trap_hand_vel(3) = acc_trap(3)*curr_t;
      des_pose.trap_hand_vel(4) = acc_trap(4)*curr_t;
      des_pose.trap_hand_vel(5) = acc_trap(5)*curr_t;
      des_pose.trap_hand_vel(6) = acc_trap(6)*curr_t;

      // hand acceleration
      des_pose.trap_hand_acc(0) = acc_trap(0);
      des_pose.trap_hand_acc(1) = acc_trap(1);
      des_pose.trap_hand_acc(2) = acc_trap(2);
      des_pose.trap_hand_acc(3) = acc_trap(3);
      des_pose.trap_hand_acc(4) = acc_trap(4);
      des_pose.trap_hand_acc(5) = acc_trap(5);
      des_pose.trap_hand_acc(6) = acc_trap(6);
      if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
        // hand pose
        des_pose.trap_hand_pose_left(0) = des_pose.hand_pos_init_vec_left(0) + 0.5*acc_trap_left(0)*pow(curr_t,2);
        des_pose.trap_hand_pose_left(1) = des_pose.hand_pos_init_vec_left(1) + 0.5*acc_trap_left(1)*pow(curr_t,2);
        des_pose.trap_hand_pose_left(2) = des_pose.hand_pos_init_vec_left(2) + 0.5*acc_trap_left(2)*pow(curr_t,2);
        des_pose.trap_hand_pose_left(3) = des_pose.hand_or_q_e_init_vec_left(0) + 0.5*acc_trap_left(3)*pow(curr_t,2);
        des_pose.trap_hand_pose_left(4) = des_pose.hand_or_q_e_init_vec_left(1) + 0.5*acc_trap_left(4)*pow(curr_t,2);
        des_pose.trap_hand_pose_left(5) = des_pose.hand_or_q_e_init_vec_left(2) + 0.5*acc_trap_left(5)*pow(curr_t,2);
        des_pose.trap_hand_pose_left(6) = des_pose.hand_or_q_w_init_vec_left + 0.5*acc_trap_left(6)*pow(curr_t,2);

        // hand velocity
        des_pose.trap_hand_vel_left(0) = acc_trap_left(0)*curr_t;
        des_pose.trap_hand_vel_left(1) = acc_trap_left(1)*curr_t;
        des_pose.trap_hand_vel_left(2) = acc_trap_left(2)*curr_t;
        des_pose.trap_hand_vel_left(3) = acc_trap_left(3)*curr_t;
        des_pose.trap_hand_vel_left(4) = acc_trap_left(4)*curr_t;
        des_pose.trap_hand_vel_left(5) = acc_trap_left(5)*curr_t;
        des_pose.trap_hand_vel_left(6) = acc_trap_left(6)*curr_t;

        // hand acceleration
        des_pose.trap_hand_acc_left(0) = acc_trap_left(0);
        des_pose.trap_hand_acc_left(1) = acc_trap_left(1);
        des_pose.trap_hand_acc_left(2) = acc_trap_left(2);
        des_pose.trap_hand_acc_left(3) = acc_trap_left(3);
        des_pose.trap_hand_acc_left(4) = acc_trap_left(4);
        des_pose.trap_hand_acc_left(5) = acc_trap_left(5);
        des_pose.trap_hand_acc_left(6) = acc_trap_left(6);
      }

  }else if((curr_t > t_c_trap) && (curr_t <= usr_interface_input.t_f_trap)){

      //hand pose
      des_pose.trap_hand_pose(0) = des_pose.des_hand_pos(0) - 0.5*acc_trap(0)*pow((usr_interface_input.t_f_trap-curr_t),2);
      des_pose.trap_hand_pose(1) = des_pose.des_hand_pos(1) - 0.5*acc_trap(1)*pow((usr_interface_input.t_f_trap-curr_t),2);
      des_pose.trap_hand_pose(2) = des_pose.des_hand_pos(2) - 0.5*acc_trap(2)*pow((usr_interface_input.t_f_trap-curr_t),2);
      des_pose.trap_hand_pose(3) = mes_pose_str.des_hand_or_q_e(0) - 0.5*acc_trap(3)*pow((usr_interface_input.t_f_trap-curr_t),2);
      des_pose.trap_hand_pose(4) = mes_pose_str.des_hand_or_q_e(1) - 0.5*acc_trap(4)*pow((usr_interface_input.t_f_trap-curr_t),2);
      des_pose.trap_hand_pose(5) = mes_pose_str.des_hand_or_q_e(2) - 0.5*acc_trap(5)*pow((usr_interface_input.t_f_trap-curr_t),2);
      des_pose.trap_hand_pose(6) = des_pose.des_hand_or_q_w - 0.5*acc_trap(6)*pow((usr_interface_input.t_f_trap-curr_t),2);

      // hand velocity
      des_pose.trap_hand_vel(0) = -acc_trap(0)*(usr_interface_input.t_f_trap-curr_t);
      des_pose.trap_hand_vel(1) = -acc_trap(1)*(usr_interface_input.t_f_trap-curr_t);
      des_pose.trap_hand_vel(2) = -acc_trap(2)*(usr_interface_input.t_f_trap-curr_t);
      des_pose.trap_hand_vel(3) = -acc_trap(3)*(usr_interface_input.t_f_trap-curr_t);
      des_pose.trap_hand_vel(4) = -acc_trap(4)*(usr_interface_input.t_f_trap-curr_t);
      des_pose.trap_hand_vel(5) = -acc_trap(5)*(usr_interface_input.t_f_trap-curr_t);
      des_pose.trap_hand_vel(6) = -acc_trap(6)*(usr_interface_input.t_f_trap-curr_t);

      // hand acceleration
      des_pose.trap_hand_acc(0) = -acc_trap(0);
      des_pose.trap_hand_acc(1) = -acc_trap(1);
      des_pose.trap_hand_acc(2) = -acc_trap(2);
      des_pose.trap_hand_acc(3) = -acc_trap(3);
      des_pose.trap_hand_acc(4) = -acc_trap(4);
      des_pose.trap_hand_acc(5) = -acc_trap(5);
      des_pose.trap_hand_acc(6) = -acc_trap(6);

      if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
        //hand pose
        des_pose.trap_hand_pose_left(0) = des_pose.des_hand_pos_left(0) - 0.5*acc_trap_left(0)*pow((usr_interface_input.t_f_trap-curr_t),2);
        des_pose.trap_hand_pose_left(1) = des_pose.des_hand_pos_left(1) - 0.5*acc_trap_left(1)*pow((usr_interface_input.t_f_trap-curr_t),2);
        des_pose.trap_hand_pose_left(2) = des_pose.des_hand_pos_left(2) - 0.5*acc_trap_left(2)*pow((usr_interface_input.t_f_trap-curr_t),2);
        des_pose.trap_hand_pose_left(3) = mes_pose_str.des_hand_or_q_e_left(0) - 0.5*acc_trap_left(3)*pow((usr_interface_input.t_f_trap-curr_t),2);
        des_pose.trap_hand_pose_left(4) = mes_pose_str.des_hand_or_q_e_left(1) - 0.5*acc_trap_left(4)*pow((usr_interface_input.t_f_trap-curr_t),2);
        des_pose.trap_hand_pose_left(5) = mes_pose_str.des_hand_or_q_e_left(2) - 0.5*acc_trap_left(5)*pow((usr_interface_input.t_f_trap-curr_t),2);
        des_pose.trap_hand_pose_left(6) = des_pose.des_hand_or_q_w_left - 0.5*acc_trap_left(6)*pow((usr_interface_input.t_f_trap-curr_t),2);

        // hand velocity
        des_pose.trap_hand_vel_left(0) = -acc_trap_left(0)*(usr_interface_input.t_f_trap-curr_t);
        des_pose.trap_hand_vel_left(1) = -acc_trap_left(1)*(usr_interface_input.t_f_trap-curr_t);
        des_pose.trap_hand_vel_left(2) = -acc_trap_left(2)*(usr_interface_input.t_f_trap-curr_t);
        des_pose.trap_hand_vel_left(3) = -acc_trap_left(3)*(usr_interface_input.t_f_trap-curr_t);
        des_pose.trap_hand_vel_left(4) = -acc_trap_left(4)*(usr_interface_input.t_f_trap-curr_t);
        des_pose.trap_hand_vel_left(5) = -acc_trap_left(5)*(usr_interface_input.t_f_trap-curr_t);
        des_pose.trap_hand_vel_left(6) = -acc_trap_left(6)*(usr_interface_input.t_f_trap-curr_t);

        // hand acceleration
        des_pose.trap_hand_acc_left(0) = -acc_trap_left(0);
        des_pose.trap_hand_acc_left(1) = -acc_trap_left(1);
        des_pose.trap_hand_acc_left(2) = -acc_trap_left(2);
        des_pose.trap_hand_acc_left(3) = -acc_trap_left(3);
        des_pose.trap_hand_acc_left(4) = -acc_trap_left(4);
        des_pose.trap_hand_acc_left(5) = -acc_trap_left(5);
        des_pose.trap_hand_acc_left(6) = -acc_trap_left(6);
      }

  }else if(curr_t > usr_interface_input.t_f_trap){
      //hand pose
      des_pose.trap_hand_pose(0) = des_pose.des_hand_pos(0);
      des_pose.trap_hand_pose(1) = des_pose.des_hand_pos(1);
      des_pose.trap_hand_pose(2) = des_pose.des_hand_pos(2);
      des_pose.trap_hand_pose(3) = mes_pose_str.des_hand_or_q_e(0);
      des_pose.trap_hand_pose(4) = mes_pose_str.des_hand_or_q_e(1);
      des_pose.trap_hand_pose(5) = mes_pose_str.des_hand_or_q_e(2);
      des_pose.trap_hand_pose(6) = des_pose.des_hand_or_q_w;

      // hand velocity
      des_pose.trap_hand_vel(0) = 0.0;
      des_pose.trap_hand_vel(1) = 0.0;
      des_pose.trap_hand_vel(2) = 0.0;
      des_pose.trap_hand_vel(3) = 0.0;
      des_pose.trap_hand_vel(4) = 0.0;
      des_pose.trap_hand_vel(5) = 0.0;
      des_pose.trap_hand_vel(6) = 0.0;

      // hand acceleration
      des_pose.trap_hand_acc(0) = 0.0;
      des_pose.trap_hand_acc(1) = 0.0;
      des_pose.trap_hand_acc(2) = 0.0;
      des_pose.trap_hand_acc(3) = 0.0;
      des_pose.trap_hand_acc(4) = 0.0;
      des_pose.trap_hand_acc(5) = 0.0;
      des_pose.trap_hand_acc(6) = 0.0;

      if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
        //hand pose
        des_pose.trap_hand_pose_left(0) = des_pose.des_hand_pos_left(0);
        des_pose.trap_hand_pose_left(1) = des_pose.des_hand_pos_left(1);
        des_pose.trap_hand_pose_left(2) = des_pose.des_hand_pos_left(2);
        des_pose.trap_hand_pose_left(3) = mes_pose_str.des_hand_or_q_e_left(0);
        des_pose.trap_hand_pose_left(4) = mes_pose_str.des_hand_or_q_e_left(1);
        des_pose.trap_hand_pose_left(5) = mes_pose_str.des_hand_or_q_e_left(2);
        des_pose.trap_hand_pose_left(6) = des_pose.des_hand_or_q_w_left;

        // hand velocity
        des_pose.trap_hand_vel_left(0) = 0.0;
        des_pose.trap_hand_vel_left(1) = 0.0;
        des_pose.trap_hand_vel_left(2) = 0.0;
        des_pose.trap_hand_vel_left(3) = 0.0;
        des_pose.trap_hand_vel_left(4) = 0.0;
        des_pose.trap_hand_vel_left(5) = 0.0;
        des_pose.trap_hand_vel_left(6) = 0.0;

        // hand acceleration
        des_pose.trap_hand_acc_left(0) = 0.0;
        des_pose.trap_hand_acc_left(1) = 0.0;
        des_pose.trap_hand_acc_left(2) = 0.0;
        des_pose.trap_hand_acc_left(3) = 0.0;
        des_pose.trap_hand_acc_left(4) = 0.0;
        des_pose.trap_hand_acc_left(5) = 0.0;
        des_pose.trap_hand_acc_left(6) = 0.0;
      }
  }


  // error in position
  Vector3d error_trap_pos;
  error_trap_pos(0) = des_pose.trap_hand_pose(0) - mes_pose_str.r_hand_pos_vec(0);
  error_trap_pos(1) = des_pose.trap_hand_pose(1) - mes_pose_str.r_hand_pos_vec(1);
  error_trap_pos(2) = des_pose.trap_hand_pose(2) - mes_pose_str.r_hand_pos_vec(2);

  // error in orientation
  Quaterniond trap_hand_or_q;
  trap_hand_or_q.x() = des_pose.trap_hand_pose(3);
  trap_hand_or_q.y() = des_pose.trap_hand_pose(4);
  trap_hand_or_q.z() = des_pose.trap_hand_pose(5);
  trap_hand_or_q.w() = des_pose.trap_hand_pose(6);

  Vector3d trap_hand_or_q_e;
  trap_hand_or_q_e << des_pose.trap_hand_pose(3),des_pose.trap_hand_pose(4),des_pose.trap_hand_pose(5);
  double trap_hand_or_q_w = des_pose.trap_hand_pose(6);
  Vector3d error_trap_or = mes_pose_str.r_hand_q_w * trap_hand_or_q_e - trap_hand_or_q_w * mes_pose_str.r_hand_or_q_e - trap_hand_or_q_e.cross(mes_pose_str.r_hand_or_q_e);

  // total error in position + orientation
  error_var.error_trap_tot << error_trap_pos(0),error_trap_pos(1),error_trap_pos(2), error_trap_or(0),error_trap_or(1),error_trap_or(2);


  thr_error = sqrt(pow(error_var.error_trap_tot(0),2)+pow(error_var.error_trap_tot(1),2)+pow(error_var.error_trap_tot(2),2));


  // error in linear velocity
  Vector3d error_trap_lin_vel;
  error_trap_lin_vel(0) = des_pose.trap_hand_vel(0) - mes_pose_str.r_hand_lin_vel_vec(0);
  error_trap_lin_vel(1) = des_pose.trap_hand_vel(1) - mes_pose_str.r_hand_lin_vel_vec(1);
  error_trap_lin_vel(2) = des_pose.trap_hand_vel(2) - mes_pose_str.r_hand_lin_vel_vec(2);

  // error in orientation velocity (quaternion)
  Vector3d trap_hand_ang_vel_q_e;
  trap_hand_ang_vel_q_e << des_pose.trap_hand_vel(3),des_pose.trap_hand_vel(4),des_pose.trap_hand_vel(5);
  double trap_hand_ang_vel_q_w = des_pose.trap_hand_vel(6);

  Vector3d error_trap_der_or = mes_pose_str.r_hand_ang_vel_q_w * trap_hand_or_q_e + mes_pose_str.r_hand_q_w * trap_hand_ang_vel_q_e - trap_hand_ang_vel_q_w * mes_pose_str.r_hand_or_q_e
                    - trap_hand_or_q_w * mes_pose_str.r_hand_ang_vel_q_e - trap_hand_ang_vel_q_e.cross( mes_pose_str.r_hand_or_q_e) - trap_hand_or_q_e.cross(mes_pose_str.r_hand_ang_vel_q_e);
  // total error in velocity
  error_var.der_error_trap_tot << error_trap_lin_vel(0),error_trap_lin_vel(1),error_trap_lin_vel(2),error_trap_der_or(0),error_trap_der_or(1),error_trap_der_or(2);

  // triangular reference velocity
  error_var.trap_hand_ref_vel(0) = des_pose.trap_hand_vel(0);
  error_var.trap_hand_ref_vel(1) = des_pose.trap_hand_vel(1);
  error_var.trap_hand_ref_vel(2) = des_pose.trap_hand_vel(2);
  Vector3d trap_hand_vel_q_e;
  trap_hand_vel_q_e << des_pose.trap_hand_vel(3), des_pose.trap_hand_vel(4), des_pose.trap_hand_vel(5);
  double trap_hand_vel_q_w = des_pose.trap_hand_vel(6);

  Vector3d des_omega = 2 * trap_hand_or_q_e.cross(trap_hand_vel_q_e) + 2 * trap_hand_or_q_w*trap_hand_vel_q_e - 2 * trap_hand_vel_q_w * trap_hand_or_q_e;
  error_var.trap_hand_ref_vel(3) = des_omega(0);
  error_var.trap_hand_ref_vel(4) = des_omega(1);
  error_var.trap_hand_ref_vel(5) = des_omega(2);

  // triangular reference acceleration
  error_var.trap_hand_ref_acc(0) = des_pose.trap_hand_acc(0);
  error_var.trap_hand_ref_acc(1) = des_pose.trap_hand_acc(1);
  error_var.trap_hand_ref_acc(2) = des_pose.trap_hand_acc(2);
  Vector3d trap_hand_acc_q_e;
  trap_hand_acc_q_e << des_pose.trap_hand_acc(3), des_pose.trap_hand_acc(4), des_pose.trap_hand_acc(5);
  double trap_hand_acc_q_w = des_pose.trap_hand_acc(6);

  Vector3d des_alpha = 2 * trap_hand_or_q_e.cross(trap_hand_acc_q_e) + 2 * trap_hand_or_q_w * trap_hand_acc_q_e - 2 * trap_hand_acc_q_w * trap_hand_or_q_e;
  error_var.trap_hand_ref_acc(3) = des_alpha(0);
  error_var.trap_hand_ref_acc(4) = des_alpha(1);
  error_var.trap_hand_ref_acc(5) = des_alpha(2);

  if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
    // error in position
    Vector3d error_trap_pos_left;
    error_trap_pos_left(0) = des_pose.trap_hand_pose_left(0) - mes_pose_str.l_hand_pos_vec(0);
    error_trap_pos_left(1) = des_pose.trap_hand_pose_left(1) - mes_pose_str.l_hand_pos_vec(1);
    error_trap_pos_left(2) = des_pose.trap_hand_pose_left(2) - mes_pose_str.l_hand_pos_vec(2);

    // error in orientation
    Quaterniond trap_hand_or_q_left;
    trap_hand_or_q_left.x() = des_pose.trap_hand_pose_left(3);
    trap_hand_or_q_left.y() = des_pose.trap_hand_pose_left(4);
    trap_hand_or_q_left.z() = des_pose.trap_hand_pose_left(5);
    trap_hand_or_q_left.w() = des_pose.trap_hand_pose_left(6);

    Vector3d trap_hand_or_q_e_left;
    trap_hand_or_q_e_left << des_pose.trap_hand_pose_left(3),des_pose.trap_hand_pose_left(4),des_pose.trap_hand_pose_left(5);
    double trap_hand_or_q_w_left = des_pose.trap_hand_pose_left(6);
    Vector3d error_trap_or_left = mes_pose_str.l_hand_q_w* trap_hand_or_q_e_left - trap_hand_or_q_w_left * mes_pose_str.l_hand_or_q_e- trap_hand_or_q_e_left.cross(mes_pose_str.l_hand_or_q_e);

    // total error in position + orientation
    error_var.error_trap_tot_left << error_trap_pos_left(0),error_trap_pos_left(1),error_trap_pos_left(2), error_trap_or_left(0),error_trap_or_left(1),error_trap_or_left(2);

    // error in linear velocity
    Vector3d error_trap_lin_vel_left;
    error_trap_lin_vel_left(0) = des_pose.trap_hand_vel_left(0) - mes_pose_str.l_hand_lin_vel_vec(0);
    error_trap_lin_vel_left(1) = des_pose.trap_hand_vel_left(1) - mes_pose_str.l_hand_lin_vel_vec(1);
    error_trap_lin_vel_left(2) = des_pose.trap_hand_vel_left(2) - mes_pose_str.l_hand_lin_vel_vec(2);

    // error in orientation velocity (quaternion)
    Vector3d trap_hand_ang_vel_q_e_left;
    trap_hand_ang_vel_q_e_left << des_pose.trap_hand_vel_left(3),des_pose.trap_hand_vel_left(4),des_pose.trap_hand_vel_left(5);
    double trap_hand_ang_vel_q_w_left = des_pose.trap_hand_vel_left(6);

    Vector3d error_trap_der_or_left = mes_pose_str.l_hand_ang_vel_q_w* trap_hand_or_q_e_left + mes_pose_str.l_hand_q_w * trap_hand_ang_vel_q_e_left - trap_hand_ang_vel_q_w_left * mes_pose_str.l_hand_or_q_e
                      - trap_hand_or_q_w_left * mes_pose_str.l_hand_ang_vel_q_e - trap_hand_ang_vel_q_e_left.cross( mes_pose_str.l_hand_or_q_e) - trap_hand_or_q_e_left.cross(mes_pose_str.l_hand_ang_vel_q_e);
    // total error in velocity
    error_var.der_error_trap_tot_left << error_trap_lin_vel_left(0),error_trap_lin_vel_left(1),error_trap_lin_vel_left(2),error_trap_der_or_left(0),error_trap_der_or_left(1),error_trap_der_or_left(2);

    // triangular reference velocity
    error_var.trap_hand_ref_vel_left(0) = des_pose.trap_hand_vel_left(0);
    error_var.trap_hand_ref_vel_left(1) = des_pose.trap_hand_vel_left(1);
    error_var.trap_hand_ref_vel_left(2) = des_pose.trap_hand_vel_left(2);
    Vector3d trap_hand_vel_q_e_left;
    trap_hand_vel_q_e_left << des_pose.trap_hand_vel_left(3), des_pose.trap_hand_vel_left(4), des_pose.trap_hand_vel_left(5);
    double trap_hand_vel_q_w_left = des_pose.trap_hand_vel_left(6);

    Vector3d des_omega_left = 2 * trap_hand_or_q_e_left.cross(trap_hand_vel_q_e_left) + 2 * trap_hand_or_q_w_left*trap_hand_vel_q_e_left - 2 * trap_hand_vel_q_w_left * trap_hand_or_q_e_left;
    error_var.trap_hand_ref_vel_left(3) = des_omega_left(0);
    error_var.trap_hand_ref_vel_left(4) = des_omega_left(1);
    error_var.trap_hand_ref_vel_left(5) = des_omega_left(2);

    // triangular reference acceleration
    error_var.trap_hand_ref_acc_left(0) = des_pose.trap_hand_acc_left(0);
    error_var.trap_hand_ref_acc_left(1) = des_pose.trap_hand_acc_left(1);
    error_var.trap_hand_ref_acc_left(2) = des_pose.trap_hand_acc_left(2);
    Vector3d trap_hand_acc_q_e_left;
    trap_hand_acc_q_e_left << des_pose.trap_hand_acc_left(3), des_pose.trap_hand_acc_left(4), des_pose.trap_hand_acc_left(5);
    double trap_hand_acc_q_w_left = des_pose.trap_hand_acc_left(6);

    Vector3d des_alpha_left = 2 * trap_hand_or_q_e_left.cross(trap_hand_acc_q_e_left) + 2 * trap_hand_or_q_w_left * trap_hand_acc_q_e_left - 2 * trap_hand_acc_q_w_left * trap_hand_or_q_e_left;
    error_var.trap_hand_ref_acc_left(3) = des_alpha_left(0);
    error_var.trap_hand_ref_acc_left(4) = des_alpha_left(1);
    error_var.trap_hand_ref_acc_left(5) = des_alpha_left(2);
  }
}

/******************************************************
*		AUXILIARY FUNCTIONS                               *
*-----------------------------------------------------*
* These functions avoid repetition of code and improve*
* code redability                                     *
*-----------------------------------------------------*
* applyNoiseToTarget - this function simulates the    *
* the noise that  would be encouterd in the objects 	*
* due to noise in the sensors, i.e. camera            *
*-----------------------------------------------------*
* read_usr_interface - used to read user input        *
* Note - supports dual arm ✅                          *
* To support task needs - traj_descr_mov              *
*-----------------------------------------------------*
* gets - reads real-time data from the robot such as	*
* joint position, velocity and acceleration and wrist *
* , elbow, etc... position, velocity and as well.     *
*-----------------------------------------------------*
******************************************************/


void LocalPlanner::read_usr_interface() {
  // reading the user interface
  usr_interface_input.follow_tar = ui.checkBox_follow_target->isChecked();

  //este codigo parece ser repetido no use_plan_hand
  if(!(ui.radioButton_ctrl_task->isChecked()) && this->h_results != nullptr){
    stage_descr = h_results->trajectory_descriptions.at(ctx.i_ctrl);
  }
  else if(ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 1)){
    stage_descr = this->trajectory_descriptions.at(ctx.i_ctrl);
  }
  else if(!(ui.radioButton_ctrl_task->isChecked()) && this->h_dual_results != nullptr){
    stage_descr = h_dual_results->trajectory_descriptions.at(ctx.i_ctrl);
  }
  else if(ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2)){
    stage_descr = this->trajectory_descriptions.at(ctx.i_ctrl);
  }
  usr_interface_input.sim_robot = ui.radioButton_sim->isChecked();
  usr_interface_input.plan_hand_pos = ui.checkBox_use_plan_hand_pos->isChecked();

  usr_interface_input.vel_max = ui.lineEdit_vel_max->text().toDouble()*M_PI/180;
  usr_interface_input.joints_arm_vel_ctrl = ui.checkBox_joints_velocity_ctrl->isChecked();
  usr_interface_input.swivel_angle_th = ui.lineEdit_swivel_angle_th->text().toDouble();
  usr_interface_input.g_map_th_max_replan = ui.lineEdit_g_th_max_replan->text().toDouble();
  usr_interface_input.g_map_th_min_replan = ui.lineEdit_g_th_min_replan->text().toDouble();
  usr_interface_input.t_f_trap = ui.lineEdit_t_f_trap->text().toDouble();

  usr_interface_input.jlim_en = ui.checkBox_joints_limits_av->isChecked();
  if(usr_interface_input.jlim_en){
  avoidance_var.jlim_th = ui.lineEdit_jlim_th->text().toDouble();
  avoidance_var.jlim_rate = ui.lineEdit_jlim_rate->text().toDouble();
  avoidance_var.jlim_coeff = ui.lineEdit_jlim_coeff->text().toDouble();
  avoidance_var.jlim_damping = ui.lineEdit_jlim_damping->text().toDouble();
  }

  usr_interface_input.sing_en = ui.checkBox_sing_av->isChecked();

  if(usr_interface_input.sing_en){
  avoidance_var.sing_damping = ui.lineEdit_sing_damping->text().toDouble();
  avoidance_var.sing_coeff = ui.lineEdit_sing_coeff->text().toDouble();
  }

  usr_interface_input.obsts_en = ui.checkBox_obsts_av->isChecked();
  usr_interface_input.hl_en = ui.checkBox_hl_add->isChecked();
  usr_interface_input.hl_alpha_en = ui.checkBox_hl_add_alpha->isChecked();

  /*
  // position Koeff
  Vector3d error_abs_pos; Vector3d error_abs_or; //pode mudar de sitio este codigo!!!!!!!!!!
  error_abs_pos << abs(mes_pose_str.error_pos(0)),abs(mes_pose_str.error_pos(1)),abs(mes_pose_str.error_pos(2));
  error_abs_or << abs(mes_pose_str.error_or(0)),abs(mes_pose_str.error_or(1)),abs(mes_pose_str.error_or(2));
  mes_pose_str.e_n_pos = mes_pose_str.error_pos.norm(); mes_pose_str.e_n_or = mes_pose_str.error_or.norm();
*/

  human_likeness_coef.error_pos_th = ui.lineEdit_err_p_pos->text().toDouble();
  human_likeness_coef.error_or_th = ui.lineEdit_err_p_or->text().toDouble();
  human_likeness_coef.coeff_p_pos = ui.lineEdit_coeff_p_pos->text().toDouble();
  human_likeness_coef.coeff_p_or = ui.lineEdit_coeff_p_or->text().toDouble();
  human_likeness_coef.coeff_d_pos = ui.lineEdit_coeff_d_pos->text().toDouble();
  human_likeness_coef.coeff_d_or = ui.lineEdit_coeff_d_or->text().toDouble();


  if((ui.checkBox_des_right_hand_pos_x->isChecked())){
      if(usr_interface_input.hl_en){
    human_likeness_coef.Koeff_p(0,0) = human_likeness_coef.hl_p_x_pos_coeff;
      }else{
    human_likeness_coef.Koeff_p(0,0) = human_likeness_coef.coeff_p_pos;
      }
  }else{ human_likeness_coef.Koeff_p(0,0) = 0.0;}
  if((ui.checkBox_des_right_hand_pos_y->isChecked())){
      if(usr_interface_input.hl_en){
    human_likeness_coef.Koeff_p(1,1) = human_likeness_coef.hl_p_y_pos_coeff;
      }else{
    human_likeness_coef.Koeff_p(1,1) = human_likeness_coef.coeff_p_pos;
      }
  }else{ human_likeness_coef.Koeff_p(1,1) = 0.0;}
  if((ui.checkBox_des_right_hand_pos_z->isChecked())){
      if(usr_interface_input.hl_en){
    human_likeness_coef.Koeff_p(2,2) = human_likeness_coef.hl_p_z_pos_coeff;
      }else{
    human_likeness_coef.Koeff_p(2,2) = human_likeness_coef.coeff_p_pos;
      }
  }else{ human_likeness_coef.Koeff_p(2,2) = 0.0;}
  if((ui.checkBox_des_right_hand_q_x->isChecked())){
      if(usr_interface_input.hl_en){
    human_likeness_coef.Koeff_p(3,3) = human_likeness_coef.hl_p_x_or_coeff;
      }else{
    human_likeness_coef.Koeff_p(3,3) = human_likeness_coef.coeff_p_or;
      }
  }else{ human_likeness_coef.Koeff_p(3,3) = 0.0;}
  if((ui.checkBox_des_right_hand_q_y->isChecked())){
      if(usr_interface_input.hl_en){
    human_likeness_coef.Koeff_p(4,4) = human_likeness_coef.hl_p_y_or_coeff;
      }else{
    human_likeness_coef.Koeff_p(4,4) = human_likeness_coef.coeff_p_or;
      }
  }else{ human_likeness_coef.Koeff_p(4,4) = 0.0;}
  if((ui.checkBox_des_right_hand_q_z->isChecked())){
      if(usr_interface_input.hl_en){
    human_likeness_coef.Koeff_p(5,5) = human_likeness_coef.hl_p_z_or_coeff;
      }else{
    human_likeness_coef.Koeff_p(5,5) = human_likeness_coef.coeff_p_or;
      }
  }else{ human_likeness_coef.Koeff_p(5,5) = 0.0;}

  //MatrixXd Koeff_d = MatrixXd::Identity(6,6); //colocado no .hpp
  if((ui.checkBox_des_right_hand_pos_x->isChecked())){
      if(usr_interface_input.hl_en){
    human_likeness_coef.Koeff_d(0,0) = human_likeness_coef.hl_d_x_pos_coeff;
      }else{
    human_likeness_coef.Koeff_d(0,0) = human_likeness_coef.coeff_d_pos;
      }
  }else{ human_likeness_coef.Koeff_d(0,0) = 0.0;}
  if((ui.checkBox_des_right_hand_pos_y->isChecked())){
      if(usr_interface_input.hl_en){
    human_likeness_coef.Koeff_d(1,1) = human_likeness_coef.hl_d_y_pos_coeff;
      }else{
    human_likeness_coef.Koeff_d(1,1) = human_likeness_coef.coeff_d_pos;
      }
  }else{ human_likeness_coef.Koeff_d(1,1) = 0.0;}
  if((ui.checkBox_des_right_hand_pos_z->isChecked())){
      if(usr_interface_input.hl_en){
    human_likeness_coef.Koeff_d(2,2) = human_likeness_coef.hl_d_z_pos_coeff;
      }else{
    human_likeness_coef.Koeff_d(2,2) = human_likeness_coef.coeff_d_pos;
      }
  }else{ human_likeness_coef.Koeff_d(2,2) = 0.0;}
  if((ui.checkBox_des_right_hand_q_x->isChecked())){
      if(usr_interface_input.hl_en){
    human_likeness_coef.Koeff_d(3,3) = human_likeness_coef.hl_d_x_or_coeff;
      }else{
    human_likeness_coef.Koeff_d(3,3) = human_likeness_coef.coeff_d_or;
      }
  }else{ human_likeness_coef.Koeff_d(3,3) = 0.0;}
  if((ui.checkBox_des_right_hand_q_y->isChecked())){
      if(usr_interface_input.hl_en){
    human_likeness_coef.Koeff_d(4,4) = human_likeness_coef.hl_d_y_or_coeff;
      }else{
    human_likeness_coef.Koeff_d(4,4) = human_likeness_coef.coeff_d_or;
      }
  }else{ human_likeness_coef.Koeff_d(4,4) = 0.0;}
  if((ui.checkBox_des_right_hand_q_z->isChecked())){
      if(usr_interface_input.hl_en){
    human_likeness_coef.Koeff_d(5,5) = human_likeness_coef.hl_d_z_or_coeff;
      }else{
    human_likeness_coef.Koeff_d(5,5) = human_likeness_coef.coeff_d_or;
      }
  }else{ human_likeness_coef.Koeff_d(5,5) = 0.0;}
}

void LocalPlanner::mes_pose() {
  ctx.curr_scene->getHumanoid()->getRightArmPosture(mes_pose_str.r_arm_posture_mes);
  ctx.curr_scene->getHumanoid()->getRightHandPosture(mes_pose_str.r_hand_posture_mes);
  // filtering the joint positions
  mes_pose_str.r_arm_posture.at(0) = lpf.lpf_joint_pos_1->update(mes_pose_str.r_arm_posture_mes.at(0));
  mes_pose_str.r_arm_posture.at(1) = lpf.lpf_joint_pos_2->update(mes_pose_str.r_arm_posture_mes.at(1));
  mes_pose_str.r_arm_posture.at(2) = lpf.lpf_joint_pos_3->update(mes_pose_str.r_arm_posture_mes.at(2));
  mes_pose_str.r_arm_posture.at(3) = lpf.lpf_joint_pos_4->update(mes_pose_str.r_arm_posture_mes.at(3));
  mes_pose_str.r_arm_posture.at(4) = lpf.lpf_joint_pos_5->update(mes_pose_str.r_arm_posture_mes.at(4));
  mes_pose_str.r_arm_posture.at(5) = lpf.lpf_joint_pos_6->update(mes_pose_str.r_arm_posture_mes.at(5));
  mes_pose_str.r_arm_posture.at(6) = lpf.lpf_joint_pos_7->update(mes_pose_str.r_arm_posture_mes.at(6));

#if HAND ==1
  mes_pose_str.r_hand_posture.at(0) = lpf.lpf_joint_pos_8->update(mes_pose_str.r_hand_posture_mes.at(0));
  mes_pose_str.r_hand_posture.at(1) = lpf.lpf_joint_pos_9->update(mes_pose_str.r_hand_posture_mes.at(1));
  mes_pose_str.r_hand_posture.at(2) = lpf.lpf_joint_pos_10->update(mes_pose_str.r_hand_posture_mes.at(2));
  mes_pose_str.r_hand_posture.at(3) = lpf.lpf_joint_pos_11->update(mes_pose_str.r_hand_posture_mes.at(3));
#elif HAND == 2
  mes_pose_str.r_hand_posture.at(0) = lpf.lpf_joint_pos_8->update(mes_pose_str.r_hand_posture_mes.at(0));
#endif


  ctx.curr_scene->getHumanoid()->getAllPos(1,mes_pose_str.r_hand_pos,mes_pose_str.r_wrist_pos,mes_pose_str.r_elbow_pos,mes_pose_str.r_shoulder_pos,mes_pose_str.r_arm_posture);
  ctx.curr_scene->getHumanoid()->getAllPos_q(1,mes_pose_str.r_hand_pos_q,mes_pose_str.r_wrist_pos_q,mes_pose_str.r_elbow_pos_q,mes_pose_str.r_shoulder_pos_q,mes_pose_str.r_arm_posture);

  mes_pose_str.r_hand_lin_pos.assign(mes_pose_str.r_hand_pos.begin(), mes_pose_str.r_hand_pos.begin()+3);
  mes_pose_str.r_hand_ang_pos.assign(mes_pose_str.r_hand_pos.begin()+3, mes_pose_str.r_hand_pos.begin()+6);
  mes_pose_str.r_hand_q.assign(mes_pose_str.r_hand_pos_q.begin()+3, mes_pose_str.r_hand_pos_q.begin()+7);
  mes_pose_str.r_wrist_lin_pos.assign(mes_pose_str.r_wrist_pos.begin(), mes_pose_str.r_wrist_pos.begin()+3);
  mes_pose_str.r_wrist_ang_pos.assign(mes_pose_str.r_wrist_pos.begin()+3, mes_pose_str.r_wrist_pos.begin()+6);
  mes_pose_str.r_elbow_lin_pos.assign(mes_pose_str.r_elbow_pos.begin(), mes_pose_str.r_elbow_pos.begin()+3);
  mes_pose_str.r_elbow_ang_pos.assign(mes_pose_str.r_elbow_pos.begin()+3, mes_pose_str.r_elbow_pos.begin()+6);
  mes_pose_str.r_shoulder_lin_pos.assign(mes_pose_str.r_shoulder_pos.begin(), mes_pose_str.r_shoulder_pos.begin()+3);
  mes_pose_str.r_shoulder_ang_pos.assign(mes_pose_str.r_shoulder_pos.begin()+3, mes_pose_str.r_shoulder_pos.begin()+6);

  if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
    ctx.curr_scene->getHumanoid()->getLeftArmPosture(mes_pose_str.l_arm_posture_mes);
    ctx.curr_scene->getHumanoid()->getLeftHandPosture(mes_pose_str.l_hand_posture_mes);
    // filtering the joint positions
    mes_pose_str.l_arm_posture.at(0) = lpf.lpf_joint_pos_1->update(mes_pose_str.l_arm_posture_mes.at(0));
    mes_pose_str.l_arm_posture.at(1) = lpf.lpf_joint_pos_2->update(mes_pose_str.l_arm_posture_mes.at(1));
    mes_pose_str.l_arm_posture.at(2) = lpf.lpf_joint_pos_3->update(mes_pose_str.l_arm_posture_mes.at(2));
    mes_pose_str.l_arm_posture.at(3) = lpf.lpf_joint_pos_4->update(mes_pose_str.l_arm_posture_mes.at(3));
    mes_pose_str.l_arm_posture.at(4) = lpf.lpf_joint_pos_5->update(mes_pose_str.l_arm_posture_mes.at(4));
    mes_pose_str.l_arm_posture.at(5) = lpf.lpf_joint_pos_6->update(mes_pose_str.l_arm_posture_mes.at(5));
    mes_pose_str.l_arm_posture.at(6) = lpf.lpf_joint_pos_7->update(mes_pose_str.l_arm_posture_mes.at(6));

  #if HAND ==1
    mes_pose_str.r_hand_posture.at(0) = lpf.lpf_joint_pos_8->update(mes_pose_str.r_hand_posture_mes.at(0));
    mes_pose_str.r_hand_posture.at(1) = lpf.lpf_joint_pos_9->update(mes_pose_str.r_hand_posture_mes.at(1));
    mes_pose_str.r_hand_posture.at(2) = lpf.lpf_joint_pos_10->update(mes_pose_str.r_hand_posture_mes.at(2));
    mes_pose_str.r_hand_posture.at(3) = lpf.lpf_joint_pos_11->update(mes_pose_str.r_hand_posture_mes.at(3));
  #elif HAND == 2
    mes_pose_str.l_hand_posture.at(0) = lpf.lpf_joint_pos_8->update(mes_pose_str.l_hand_posture_mes.at(0));
  #endif


    ctx.curr_scene->getHumanoid()->getAllPos(2,mes_pose_str.l_hand_pos,mes_pose_str.l_wrist_pos,mes_pose_str.l_elbow_pos,mes_pose_str.l_shoulder_pos,mes_pose_str.l_arm_posture);
    ctx.curr_scene->getHumanoid()->getAllPos_q(2,mes_pose_str.l_hand_pos_q,mes_pose_str.l_wrist_pos_q,mes_pose_str.l_elbow_pos_q,mes_pose_str.l_shoulder_pos_q,mes_pose_str.l_arm_posture);

    mes_pose_str.l_hand_lin_pos.assign(mes_pose_str.l_hand_pos.begin(), mes_pose_str.l_hand_pos.begin()+3);
    mes_pose_str.l_hand_ang_pos.assign(mes_pose_str.l_hand_pos.begin()+3, mes_pose_str.l_hand_pos.begin()+6);
    mes_pose_str.l_hand_q.assign(mes_pose_str.l_hand_pos_q.begin()+3, mes_pose_str.l_hand_pos_q.begin()+7);
    mes_pose_str.l_wrist_lin_pos.assign(mes_pose_str.l_wrist_pos.begin(), mes_pose_str.l_wrist_pos.begin()+3);
    mes_pose_str.l_wrist_ang_pos.assign(mes_pose_str.l_wrist_pos.begin()+3, mes_pose_str.l_wrist_pos.begin()+6);
    mes_pose_str.l_elbow_lin_pos.assign(mes_pose_str.l_elbow_pos.begin(), mes_pose_str.l_elbow_pos.begin()+3);
    mes_pose_str.l_elbow_ang_pos.assign(mes_pose_str.l_elbow_pos.begin()+3, mes_pose_str.l_elbow_pos.begin()+6);
    mes_pose_str.l_shoulder_lin_pos.assign(mes_pose_str.l_shoulder_pos.begin(), mes_pose_str.l_shoulder_pos.begin()+3);
    mes_pose_str.l_shoulder_ang_pos.assign(mes_pose_str.l_shoulder_pos.begin()+3, mes_pose_str.l_shoulder_pos.begin()+6);
  }

  //Get the rest of the measurements
  get_des_hand_vel();
  get_swivel_angle();
  get_joint_velocities();
  getAllVel();
  get_joint_accel();
  get_hand_accel();
  get_wrist_accel();
  get_elbow_accel();
  get_shoulder_accel();
  getAllAcc();
}

void LocalPlanner::get_des_hand_vel() {
  // get desired hand velocity
  VectorXd::Map(&des_pose.hand_pos_vec_xd[0], des_pose.hand_pos_vec_x.size()) = des_pose.hand_pos_vec_x;
  ctx.des_hand_pose_buff->push(des_pose.hand_pos_vec_xd);
  bool temp = ctx.des_hand_pose_buff->full();
 // if(ctx.samples_des_hand_pose==lpf.N_filter_length-1 && ctx.des_hand_pose_buff->full()){
  if(ctx.samples_des_hand_pose==lpf.N_filter_length-1 && temp){
      for(size_t i=0; i< des_pose.hand_pos_vec_xd.size();++i)
          {
              des_pose.hand_vel_vec_x(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_var.time_step,ctx.des_hand_pose_buff->at(i));
          }
  }else{ctx.samples_des_hand_pose++;}

  //dual arm
  if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
    // get desired hand velocity
    VectorXd::Map(&des_pose.hand_pos_vec_xd_left[0], des_pose.hand_pos_vec_x_left.size()) = des_pose.hand_pos_vec_x_left;
    ctx.des_hand_pose_buff_left->push(des_pose.hand_pos_vec_xd_left);
    bool temp = ctx.des_hand_pose_buff_left->full();
    if(ctx.samples_des_hand_pose_left==lpf.N_filter_length-1 && temp){
        for(size_t i=0; i< des_pose.hand_pos_vec_xd_left.size();++i)
            {
                des_pose.hand_vel_vec_x_left(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_var.time_step,ctx.des_hand_pose_buff_left->at(i));
            }
    }else{ctx.samples_des_hand_pose_left++;}
  }
}

void LocalPlanner::get_swivel_angle() {
  // ------------- SWIVEL ANGLE ----------------- //
  // predicted swivel angle
  //double alpha_predicted = RADTODEG*this->getPredictedSwivelAngle(hand_pos_vec_x);
  //BOOST_LOG_SEV(lg, info) << "alpha_predicted = " << alpha_predicted;
  // derivative of the predicted swivel angle
  //double der_alpha_predicted = RADTODEG*this->getDerivativePredictedSwivelAngle(hand_pos_vec_x,hand_vel_vec_x);
  //BOOST_LOG_SEV(lg, info) << "der_alpha_predicted = " << der_alpha_predicted;
  //this->pred_swivel_angle_ctrl.push_back(alpha_predicted);
  //this->pred_der_swivel_angle_ctrl.push_back(der_alpha_predicted);

  double curr_alpha_pos = ctx.curr_scene->getHumanoid()->getSwivelAngle(1,mes_pose_str.r_arm_posture);
  mes_pose_str.alpha_pos_read.at(0) = curr_alpha_pos;
  //BOOST_LOG_SEV(lg, info) << "curr_alpha_pos = " << curr_alpha_pos;
  ctx.alpha_pos_buff->push(mes_pose_str.alpha_pos_read);
  if(ctx.samples_alpha_pos==lpf.N_filter_length-1 && ctx.alpha_pos_buff->full()){
      mes_pose_str.alpha_vel_read.at(0) = getNoiseRobustDerivate(lpf.N_filter_length,time_var.time_step,ctx.alpha_pos_buff->at(0));
  }else{ctx.samples_alpha_pos++;}
  //BOOST_LOG_SEV(lg, info) << "alpha_vel_read.at(0)  = " << alpha_vel_read.at(0) ;
  ctx.alpha_vel_buff->push(mes_pose_str.alpha_vel_read);
  if(ctx.samples_alpha_vel==lpf.N_filter_length-1 && ctx.alpha_vel_buff->full()){
      mes_pose_str.alpha_acc_read.at(0) = getNoiseRobustDerivate(lpf.N_filter_length,time_var.time_step,ctx.alpha_vel_buff->at(0));
  }else{ctx.samples_alpha_vel++;}

  //dual arm
  if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
    double curr_alpha_pos_left = ctx.curr_scene->getHumanoid()->getSwivelAngle(2,mes_pose_str.l_arm_posture);
    mes_pose_str.alpha_pos_read_left.at(0) = curr_alpha_pos_left;
    ctx.alpha_pos_buff_left->push(mes_pose_str.alpha_pos_read_left);
    if(ctx.samples_alpha_pos_left==lpf.N_filter_length-1 && ctx.alpha_pos_buff_left->full()){
        mes_pose_str.alpha_vel_read_left.at(0) = getNoiseRobustDerivate(lpf.N_filter_length,time_var.time_step,ctx.alpha_pos_buff_left->at(0));
    }else{ctx.samples_alpha_pos_left++;}
    ctx.alpha_vel_buff_left->push(mes_pose_str.alpha_vel_read_left);
    if(ctx.samples_alpha_vel_left==lpf.N_filter_length-1 && ctx.alpha_vel_buff_left->full()){
        mes_pose_str.alpha_acc_read_left.at(0) = getNoiseRobustDerivate(lpf.N_filter_length,time_var.time_step,ctx.alpha_vel_buff_left->at(0));
    }else{ctx.samples_alpha_vel_left++;}
  }
}

void LocalPlanner::get_joint_velocities() {
  // get the joint velocities
  ctx.arm_pos_buff->push(mes_pose_str.r_arm_posture);
  ctx.hand_pos_buff->push(mes_pose_str.r_hand_posture);
  if(ctx.samples_pos==lpf.N_filter_length-1 && ctx.arm_pos_buff->full() && ctx.hand_pos_buff->full()){
      for(size_t i=0; i< mes_pose_str.r_arm_posture.size();++i)
      {
        mes_pose_str.r_arm_velocities_read.at(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_var.time_step,ctx.arm_pos_buff->at(i));
      }
      for(size_t i=0; i< mes_pose_str.r_hand_velocities_read.size();++i)
      {
        mes_pose_str.r_hand_velocities_read.at(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_var.time_step,ctx.hand_pos_buff->at(i));
      }
  }else{ctx.samples_pos++;}


  //dual arm
  if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
    // get the joint velocities
    ctx.arm_pos_buff_left->push(mes_pose_str.l_arm_posture);
    ctx.hand_pos_buff_left->push(mes_pose_str.l_hand_posture);
    if(ctx.samples_pos_left==lpf.N_filter_length-1 && ctx.arm_pos_buff_left->full() && ctx.hand_pos_buff_left->full()){
        for(size_t i=0; i< mes_pose_str.l_arm_posture.size();++i)
        {
          mes_pose_str.l_arm_velocities_read.at(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_var.time_step,ctx.arm_pos_buff_left->at(i));
        }
        for(size_t i=0; i< mes_pose_str.l_hand_velocities_read.size();++i)
        {
          mes_pose_str.l_hand_velocities_read.at(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_var.time_step,ctx.hand_pos_buff_left->at(i));
        }
    }else{ctx.samples_pos_left++;}
  }
}

void LocalPlanner::getAllVel() {

  ctx.curr_scene->getHumanoid()->getAllVel(1,mes_pose_str.r_hand_vel,mes_pose_str.r_wrist_vel,mes_pose_str.r_elbow_vel,mes_pose_str.r_shoulder_vel,mes_pose_str.r_arm_posture,mes_pose_str.r_arm_velocities_read);

  mes_pose_str.r_hand_lin_vel.assign(mes_pose_str.r_hand_vel.begin(), mes_pose_str.r_hand_vel.begin()+3);
  mes_pose_str.r_hand_ang_vel.assign(mes_pose_str.r_hand_vel.begin()+3, mes_pose_str.r_hand_vel.begin()+6);
  mes_pose_str.r_wrist_lin_vel.assign(mes_pose_str.r_wrist_vel.begin(), mes_pose_str.r_wrist_vel.begin()+3);
  mes_pose_str.r_wrist_ang_vel.assign(mes_pose_str.r_wrist_vel.begin()+3, mes_pose_str.r_wrist_vel.begin()+6);
  mes_pose_str.r_elbow_lin_vel.assign(mes_pose_str.r_elbow_vel.begin(), mes_pose_str.r_elbow_vel.begin()+3);
  mes_pose_str.r_elbow_ang_vel.assign(mes_pose_str.r_elbow_vel.begin()+3, mes_pose_str.r_elbow_vel.begin()+6);
  mes_pose_str.r_shoulder_lin_vel.assign(mes_pose_str.r_shoulder_vel.begin(), mes_pose_str.r_shoulder_vel.begin()+3);
  mes_pose_str.r_shoulder_ang_vel.assign(mes_pose_str.r_shoulder_vel.begin()+3, mes_pose_str.r_shoulder_vel.begin()+6);


  if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
    ctx.curr_scene->getHumanoid()->getAllVel(2,mes_pose_str.l_hand_vel,mes_pose_str.l_wrist_vel,mes_pose_str.l_elbow_vel,mes_pose_str.l_shoulder_vel,mes_pose_str.l_arm_posture,mes_pose_str.l_arm_velocities_read);

    mes_pose_str.l_hand_lin_vel.assign(mes_pose_str.l_hand_vel.begin(), mes_pose_str.l_hand_vel.begin()+3);
    mes_pose_str.l_hand_ang_vel.assign(mes_pose_str.l_hand_vel.begin()+3, mes_pose_str.l_hand_vel.begin()+6);
    mes_pose_str.l_wrist_lin_vel.assign(mes_pose_str.l_wrist_vel.begin(), mes_pose_str.l_wrist_vel.begin()+3);
    mes_pose_str.l_wrist_ang_vel.assign(mes_pose_str.l_wrist_vel.begin()+3, mes_pose_str.l_wrist_vel.begin()+6);
    mes_pose_str.l_elbow_lin_vel.assign(mes_pose_str.l_elbow_vel.begin(), mes_pose_str.l_elbow_vel.begin()+3);
    mes_pose_str.l_elbow_ang_vel.assign(mes_pose_str.l_elbow_vel.begin()+3, mes_pose_str.l_elbow_vel.begin()+6);
    mes_pose_str.l_shoulder_lin_vel.assign(mes_pose_str.l_shoulder_vel.begin(), mes_pose_str.l_shoulder_vel.begin()+3);
    mes_pose_str.l_shoulder_ang_vel.assign(mes_pose_str.l_shoulder_vel.begin()+3, mes_pose_str.l_shoulder_vel.begin()+6);
  }
}

void LocalPlanner::get_joint_accel() {
  ctx.arm_vel_buff->push(mes_pose_str.r_arm_velocities_read);
  ctx.hand_vel_buff->push(mes_pose_str.r_hand_velocities_read);
  if(ctx.samples_vel==lpf.N_filter_length-1 && ctx.arm_vel_buff->full() && ctx.hand_vel_buff->full()){
      for(size_t i=0; i< mes_pose_str.r_arm_velocities_read.size();++i)
      {
        mes_pose_str.r_arm_accelerations_read.at(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_var.time_step,ctx.arm_vel_buff->at(i));
      }
      for(size_t i=0; i< mes_pose_str.r_hand_velocities_read.size();++i)
      {
        mes_pose_str.r_hand_accelerations_read.at(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_var.time_step,ctx.hand_vel_buff->at(i));
      }
  }else{ctx.samples_vel++;}

  if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
    ctx.arm_vel_buff_left->push(mes_pose_str.l_arm_velocities_read);
    ctx.hand_vel_buff_left->push(mes_pose_str.l_hand_velocities_read);
    if(ctx.samples_vel_left==lpf.N_filter_length-1 && ctx.arm_vel_buff_left->full() && ctx.hand_vel_buff_left->full()){
        for(size_t i=0; i< mes_pose_str.l_arm_velocities_read.size();++i)
        {
          mes_pose_str.l_arm_accelerations_read.at(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_var.time_step,ctx.arm_vel_buff_left->at(i));
        }
        for(size_t i=0; i< mes_pose_str.l_hand_velocities_read.size();++i)
        {
          mes_pose_str.l_hand_accelerations_read.at(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_var.time_step,ctx.hand_vel_buff_left->at(i));
        }
    }else{ctx.samples_vel_left++;}
  }
}

void LocalPlanner::get_hand_accel() {
  // get the hand acceleration
  ctx.r_hand_vel_buff->push(mes_pose_str.r_hand_vel);
  if(ctx.samples_h_vel==lpf.N_filter_length-1 && ctx.r_hand_vel_buff->full()){
      for(size_t i=0; i< mes_pose_str.r_hand_vel.size();++i)
      {
          mes_pose_str.r_hand_acc_read.at(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_var.time_step,ctx.r_hand_vel_buff->at(i));
      }
  }else{ctx.samples_h_vel++;}

  if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
    // get the hand acceleration
    ctx.l_hand_vel_buff->push(mes_pose_str.l_hand_vel);
    if(ctx.samples_h_vel_left==lpf.N_filter_length-1 && ctx.l_hand_vel_buff->full()){
        for(size_t i=0; i< mes_pose_str.l_hand_vel.size();++i)
        {
            mes_pose_str.l_hand_acc_read.at(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_var.time_step,ctx.l_hand_vel_buff->at(i));
        }
    }else{ctx.samples_h_vel_left++;}
  }
}

void LocalPlanner::get_wrist_accel() {
  // get the wrist acceleration
  ctx.r_wrist_vel_buff->push(mes_pose_str.r_wrist_vel);
  if(ctx.samples_w_vel==lpf.N_filter_length-1 && ctx.r_wrist_vel_buff->full()){
      for(size_t i=0; i< mes_pose_str.r_wrist_vel.size();++i)
      {
          mes_pose_str.r_wrist_acc_read.at(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_var.time_step,ctx.r_wrist_vel_buff->at(i));
      }
  }else{ctx.samples_w_vel++;}

  if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
    // get the wrist acceleration
    ctx.l_wrist_vel_buff->push(mes_pose_str.l_wrist_vel);
    if(ctx.samples_w_vel_left==lpf.N_filter_length-1 && ctx.l_wrist_vel_buff->full()){
        for(size_t i=0; i< mes_pose_str.l_wrist_vel.size();++i)
        {
            mes_pose_str.l_wrist_acc_read.at(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_var.time_step,ctx.l_wrist_vel_buff->at(i));
        }
    }else{ctx.samples_w_vel_left++;}
  }
}

void LocalPlanner::get_elbow_accel() {
  // get the elbow acceleration
  ctx.r_elbow_vel_buff->push(mes_pose_str.r_elbow_vel);
  if(ctx.samples_e_vel==lpf.N_filter_length-1 && ctx.r_elbow_vel_buff->full()){
      for(size_t i=0; i< mes_pose_str.r_elbow_vel.size();++i)
      {
          mes_pose_str.r_elbow_acc_read.at(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_var.time_step,ctx.r_elbow_vel_buff->at(i));
      }
  }else{ctx.samples_e_vel++;}

  if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
    // get the elbow acceleration
    ctx.l_elbow_vel_buff->push(mes_pose_str.l_elbow_vel);
    if(ctx.samples_e_vel_left==lpf.N_filter_length-1 && ctx.l_elbow_vel_buff->full()){
        for(size_t i=0; i< mes_pose_str.l_elbow_vel.size();++i)
        {
            mes_pose_str.l_elbow_acc_read.at(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_var.time_step,ctx.l_elbow_vel_buff->at(i));
        }
    }else{ctx.samples_e_vel_left++;}
  }
}

void LocalPlanner::get_shoulder_accel() {
  // get the shoulder acceleration
  ctx.r_shoulder_vel_buff->push(mes_pose_str.r_shoulder_vel);
  if(ctx.samples_s_vel==lpf.N_filter_length-1 && ctx.r_shoulder_vel_buff->full()){
      for(size_t i=0; i< mes_pose_str.r_shoulder_vel.size();++i)
      {
          mes_pose_str.r_shoulder_acc_read.at(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_var.time_step,ctx.r_shoulder_vel_buff->at(i));
      }
  }else{
      ctx.samples_s_vel++;
      if(usr_interface_input.sim_robot){
          ctx.t_der_past = qnode.getSimTime();
      }else{
          ctx.t_der_past_ctrl = boost::chrono::duration_cast<msec>(Clock::now() - ctx.start_time_point);
      }
  }

  if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
    // get the shoulder acceleration
    ctx.l_shoulder_vel_buff->push(mes_pose_str.l_shoulder_vel);
    if(ctx.samples_s_vel_left==lpf.N_filter_length-1 && ctx.l_shoulder_vel_buff->full()){
        for(size_t i=0; i< mes_pose_str.l_shoulder_vel.size();++i)
        {
            mes_pose_str.l_shoulder_acc_read.at(i) = getNoiseRobustDerivate(lpf.N_filter_length,time_var.time_step,ctx.l_shoulder_vel_buff->at(i));
        }
    }else{
        ctx.samples_s_vel_left++;
        if(usr_interface_input.sim_robot){
            ctx.t_der_past = qnode.getSimTime();
        }else{
            ctx.t_der_past_ctrl = boost::chrono::duration_cast<msec>(Clock::now() - ctx.start_time_point);
        }
    }
  }
}

void LocalPlanner::getAllAcc(){

  //linha 1444
  VectorXd r_arm_accelerations_read_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(mes_pose_str.r_arm_accelerations_read.data(), mes_pose_str.r_arm_accelerations_read.size());
  mes_pose_str.r_hand_acc_read_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(mes_pose_str.r_hand_acc_read.data(), mes_pose_str.r_hand_acc_read.size());
  mes_pose_str.r_hand_accelerations_read_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(mes_pose_str.r_hand_accelerations_read.data(), mes_pose_str.r_hand_accelerations_read.size());

  // Jacobian
  ctx.curr_scene->getHumanoid()->getJacobian(1,mes_pose_str.r_arm_posture,ctx.Jacobian);
  ctx.hand_j_acc = mes_pose_str.r_hand_acc_read_vec - ctx.Jacobian*r_arm_accelerations_read_vec; // dot{J}*dot{q}
  MatrixXd Jacobian_alpha; ctx.curr_scene->getHumanoid()->getJacobianSwivel(1,mes_pose_str.r_arm_posture,Jacobian_alpha);
  ctx.alpha_j_acc = mes_pose_str.alpha_acc_read.at(0) - (Jacobian_alpha*r_arm_accelerations_read_vec)(0); // dot{J_alpha}*dot{q}

  mes_pose_str.r_hand_lin_acc.assign(mes_pose_str.r_hand_acc_read.begin(), mes_pose_str.r_hand_acc_read.begin()+3);
  mes_pose_str.r_hand_ang_acc.assign(mes_pose_str.r_hand_acc_read.begin()+3, mes_pose_str.r_hand_acc_read.begin()+6);
  mes_pose_str.r_wrist_lin_acc.assign(mes_pose_str.r_wrist_acc_read.begin(), mes_pose_str.r_wrist_acc_read.begin()+3);
  mes_pose_str.r_wrist_ang_acc.assign(mes_pose_str.r_wrist_acc_read.begin()+3, mes_pose_str.r_wrist_acc_read.begin()+6);
  mes_pose_str.r_elbow_lin_acc.assign(mes_pose_str.r_elbow_acc_read.begin(), mes_pose_str.r_elbow_acc_read.begin()+3);
  mes_pose_str.r_elbow_ang_acc.assign(mes_pose_str.r_elbow_acc_read.begin()+3, mes_pose_str.r_elbow_acc_read.begin()+6);
  mes_pose_str.r_shoulder_lin_acc.assign(mes_pose_str.r_shoulder_acc_read.begin(), mes_pose_str.r_shoulder_acc_read.begin()+3);
  mes_pose_str.r_shoulder_ang_acc.assign(mes_pose_str.r_shoulder_acc_read.begin()+3, mes_pose_str.r_shoulder_acc_read.begin()+6);

  mes_pose_str.r_hand_q_w = mes_pose_str.r_hand_q.at(3);

  mes_pose_str.r_hand_pos_vec << mes_pose_str.r_hand_lin_pos.at(0),mes_pose_str.r_hand_lin_pos.at(1),mes_pose_str.r_hand_lin_pos.at(2);
  mes_pose_str.r_hand_lin_vel_vec << mes_pose_str.r_hand_lin_vel.at(0),mes_pose_str.r_hand_lin_vel.at(1),mes_pose_str.r_hand_lin_vel.at(2);
  mes_pose_str.r_hand_ang_vel_vec << mes_pose_str.r_hand_ang_vel.at(0),mes_pose_str.r_hand_ang_vel.at(1),mes_pose_str.r_hand_ang_vel.at(2);
  mes_pose_str.r_hand_or_q_e << mes_pose_str.r_hand_q.at(0),mes_pose_str.r_hand_q.at(1),mes_pose_str.r_hand_q.at(2);

  // current hand angular velocity (quaternion)
  mes_pose_str.r_hand_ang_vel_q_e = 0.5*(mes_pose_str.r_hand_q_w*JointState.I_3*mes_pose_str.r_hand_ang_vel_vec-mes_pose_str.r_hand_or_q_e.cross(mes_pose_str.r_hand_ang_vel_vec));
  mes_pose_str.r_hand_ang_vel_q_w = -0.5*(mes_pose_str.r_hand_ang_vel_q_e.dot(mes_pose_str.r_hand_ang_vel_vec));

  if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
    VectorXd l_arm_accelerations_read_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(mes_pose_str.l_arm_accelerations_read.data(), mes_pose_str.l_arm_accelerations_read.size());
    mes_pose_str.l_hand_acc_read_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(mes_pose_str.l_hand_acc_read.data(), mes_pose_str.l_hand_acc_read.size());
    mes_pose_str.l_hand_accelerations_read_vec = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(mes_pose_str.l_hand_accelerations_read.data(), mes_pose_str.l_hand_accelerations_read.size());

    // Jacobian
    ctx.curr_scene->getHumanoid()->getJacobian(2,mes_pose_str.l_arm_posture,ctx.Jacobian_left);
    ctx.hand_j_acc_left = mes_pose_str.l_hand_acc_read_vec - ctx.Jacobian_left*l_arm_accelerations_read_vec; // dot{J}*dot{q}
    MatrixXd Jacobian_alpha_left; ctx.curr_scene->getHumanoid()->getJacobianSwivel(2,mes_pose_str.l_arm_posture,Jacobian_alpha_left);
    ctx.alpha_j_acc_left = mes_pose_str.alpha_acc_read_left.at(0) - (Jacobian_alpha_left*l_arm_accelerations_read_vec)(0); // dot{J_alpha}*dot{q}

    mes_pose_str.l_hand_lin_acc.assign(mes_pose_str.l_hand_acc_read.begin(), mes_pose_str.l_hand_acc_read.begin()+3);
    mes_pose_str.l_hand_ang_acc.assign(mes_pose_str.l_hand_acc_read.begin()+3, mes_pose_str.l_hand_acc_read.begin()+6);
    mes_pose_str.l_wrist_lin_acc.assign(mes_pose_str.l_wrist_acc_read.begin(), mes_pose_str.l_wrist_acc_read.begin()+3);
    mes_pose_str.l_wrist_ang_acc.assign(mes_pose_str.l_wrist_acc_read.begin()+3, mes_pose_str.l_wrist_acc_read.begin()+6);
    mes_pose_str.l_elbow_lin_acc.assign(mes_pose_str.l_elbow_acc_read.begin(), mes_pose_str.l_elbow_acc_read.begin()+3);
    mes_pose_str.l_elbow_ang_acc.assign(mes_pose_str.l_elbow_acc_read.begin()+3, mes_pose_str.l_elbow_acc_read.begin()+6);
    mes_pose_str.l_shoulder_lin_acc.assign(mes_pose_str.l_shoulder_acc_read.begin(), mes_pose_str.l_shoulder_acc_read.begin()+3);
    mes_pose_str.l_shoulder_ang_acc.assign(mes_pose_str.l_shoulder_acc_read.begin()+3, mes_pose_str.l_shoulder_acc_read.begin()+6);

    mes_pose_str.l_hand_q_w = mes_pose_str.l_hand_q.at(3);

    mes_pose_str.l_hand_pos_vec << mes_pose_str.l_hand_lin_pos.at(0),mes_pose_str.l_hand_lin_pos.at(1),mes_pose_str.l_hand_lin_pos.at(2);
    mes_pose_str.l_hand_lin_vel_vec << mes_pose_str.l_hand_lin_vel.at(0),mes_pose_str.l_hand_lin_vel.at(1),mes_pose_str.l_hand_lin_vel.at(2);
    mes_pose_str.l_hand_ang_vel_vec << mes_pose_str.l_hand_ang_vel.at(0),mes_pose_str.l_hand_ang_vel.at(1),mes_pose_str.l_hand_ang_vel.at(2);
    mes_pose_str.l_hand_or_q_e << mes_pose_str.l_hand_q.at(0),mes_pose_str.l_hand_q.at(1),mes_pose_str.l_hand_q.at(2);

    // current hand angular velocity (quaternion)
    mes_pose_str.l_hand_ang_vel_q_e = 0.5*(mes_pose_str.l_hand_q_w*JointState.I_3*mes_pose_str.l_hand_ang_vel_vec-mes_pose_str.l_hand_or_q_e.cross(mes_pose_str.l_hand_ang_vel_vec));
    mes_pose_str.l_hand_ang_vel_q_w = -0.5*(mes_pose_str.l_hand_ang_vel_q_e.dot(mes_pose_str.l_hand_ang_vel_vec));
  }
}

void LocalPlanner::calc_error() {

  //RETIRAR O CONTROLO DOS DEDOS???
  des_pose.hand_or_q_w_init_vec  = JointState.h_hand_or_q_init.at(3);
  des_pose.hand_pos_init_vec    << JointState.h_hand_pos_init.at(0),JointState.h_hand_pos_init.at(1),JointState.h_hand_pos_init.at(2);
  des_pose.hand_or_init_vec     << JointState.h_hand_or_init.at(0),JointState.h_hand_or_init.at(1),JointState.h_hand_or_init.at(2);
  des_pose.hand_or_q_e_init_vec << JointState.h_hand_or_q_init.at(0),JointState.h_hand_or_q_init.at(1),JointState.h_hand_or_q_init.at(2);

  error_var.error_f_pos = des_pose.des_hand_pos - des_pose.hand_pos_init_vec;
  error_var.error_f_orr(0) = des_pose.des_hand_or_q_x - des_pose.hand_or_q_e_init_vec(0);
  error_var.error_f_orr(1) = des_pose.des_hand_or_q_y - des_pose.hand_or_q_e_init_vec(1);
  error_var.error_f_orr(2) = des_pose.des_hand_or_q_z - des_pose.hand_or_q_e_init_vec(2);
  error_var.error_f_orr(3) = des_pose.des_hand_or_q_w - des_pose.hand_or_q_w_init_vec;
  error_var.error_f_tot << error_var.error_f_pos(0),error_var.error_f_pos(1),error_var.error_f_pos(2),error_var.error_f_orr(0),error_var.error_f_orr(1),error_var.error_f_orr(2),error_var.error_f_orr(3);

  error_var.error_f_fing_tot = JointState.jointsFinalPosition_hand - JointState.jointsInitPosition_hand;

  // error in position + orientation (quaternion)
  mes_pose_str.error_pos = des_pose.des_hand_pos - mes_pose_str.r_hand_pos_vec;



  //mes_pose_str.error_or = mes_pose_str.r_hand_q_w*mes_pose_str.des_hand_or_q_e - des_pose.des_hand_or_q_w*mes_pose_str.r_hand_or_q_e - mes_pose_str.des_hand_or_q_e.cross(mes_pose_str.r_hand_or_q_e);

  //teste
  // === INÍCIO SUBSTITUIÇÃO: cálculo robusto do erro de orientação ===
  // constroi quaternions (atenção à ordem ctor: Eigen::Quaterniond(w,x,y,z))
  Eigen::Quaterniond q_meas(mes_pose_str.r_hand_q_w,
                            mes_pose_str.r_hand_or_q_e(0),
                            mes_pose_str.r_hand_or_q_e(1),
                            mes_pose_str.r_hand_or_q_e(2));
  Eigen::Quaterniond q_des(des_pose.des_hand_or_q_w,
                           des_pose.des_hand_or_q_x,
                           des_pose.des_hand_or_q_y,
                           des_pose.des_hand_or_q_z);

  // normaliza
  q_meas.normalize();
  q_des.normalize();

  // forçar continuidade do sinal (evita -q jumps)
  if (q_des.coeffs().dot(q_meas.coeffs()) < 0.0) {
      q_des.coeffs() *= -1.0;
  }

  // erro multiplicativo q_err: rotação que leva q_meas -> q_des
  Eigen::Quaterniond q_err = q_des * q_meas.conjugate();
  q_err.normalize();

  // extrai axis-angle
  double w = (q_err.w() < -1.0) ? -1.0 :
                              (q_err.w() > 1.0) ? 1.0 :
                              q_err.w();

  double angle = 2.0 * acos(w); // in [0, pi]
  double s = sqrt(std::max(0.0, 1.0 - w*w));
  Eigen::Vector3d axis;
  if (s < 1e-6) {
      // small angle approximation: vector part ~ 0.5 * axis * angle
      axis = Eigen::Vector3d(q_err.x(), q_err.y(), q_err.z());
  } else {
      axis = Eigen::Vector3d(q_err.x(), q_err.y(), q_err.z()) / s;
  }
  Eigen::Vector3d rot_error = axis * angle; // vetor axis*angle em rad

  // deadband para evitar comandos por ruído (ajustar OR_DEADBAND abaixo)
  const double OR_DEADBAND = 1e-3;
  if (rot_error.norm() < OR_DEADBAND) rot_error.setZero();

  mes_pose_str.error_or = rot_error;
  // === FIM SUBSTITUIÇÃO ===







  error_var.error_tot << mes_pose_str.error_pos(0),mes_pose_str.error_pos(1),mes_pose_str.error_pos(2), mes_pose_str.error_or(0),mes_pose_str.error_or(1),mes_pose_str.error_or(2);

  //dual arm
  if((h_dual_results != nullptr) ||  (ui.radioButton_ctrl_task->isChecked() && (h_r_flag == 2))){
    des_pose.hand_or_q_w_init_vec_left  = JointState.h_hand_or_q_init_left.at(3);
    des_pose.hand_pos_init_vec_left    << JointState.h_hand_pos_init_left.at(0),JointState.h_hand_pos_init_left.at(1),JointState.h_hand_pos_init_left.at(2);
    des_pose.hand_or_init_vec_left     << JointState.h_hand_or_init_left.at(0),JointState.h_hand_or_init_left.at(1),JointState.h_hand_or_init_left.at(2);
    des_pose.hand_or_q_e_init_vec_left << JointState.h_hand_or_q_init_left.at(0),JointState.h_hand_or_q_init_left.at(1),JointState.h_hand_or_q_init_left.at(2);

    error_var.error_f_pos_left = des_pose.des_hand_pos_left - des_pose.hand_pos_init_vec_left;
    error_var.error_f_orr_left(0) = des_pose.des_hand_or_q_x_left - des_pose.hand_or_q_e_init_vec_left(0);
    error_var.error_f_orr_left(1) = des_pose.des_hand_or_q_y_left - des_pose.hand_or_q_e_init_vec_left(1);
    error_var.error_f_orr_left(2) = des_pose.des_hand_or_q_z_left - des_pose.hand_or_q_e_init_vec_left(2);
    error_var.error_f_orr_left(3) = des_pose.des_hand_or_q_w_left - des_pose.hand_or_q_w_init_vec_left;
    error_var.error_f_tot_left << error_var.error_f_pos_left(0),error_var.error_f_pos_left(1),error_var.error_f_pos_left(2),error_var.error_f_orr_left(0),error_var.error_f_orr_left(1),error_var.error_f_orr_left(2),error_var.error_f_orr_left(3);

    error_var.error_f_fing_tot_left = JointState.jointsFinalPosition_hand - JointState.jointsInitPosition_hand; //mudar

    // error in position + orientation (quaternion)
    mes_pose_str.error_pos_left = des_pose.des_hand_pos_left - mes_pose_str.l_hand_pos_vec;
    mes_pose_str.error_or_left = mes_pose_str.l_hand_q_w*mes_pose_str.des_hand_or_q_e_left - des_pose.des_hand_or_q_w_left*mes_pose_str.l_hand_or_q_e - mes_pose_str.des_hand_or_q_e_left.cross(mes_pose_str.l_hand_or_q_e);
    error_var.error_tot_left << mes_pose_str.error_pos_left(0),mes_pose_str.error_pos_left(1),mes_pose_str.error_pos_left(2), mes_pose_str.error_or_left(0),mes_pose_str.error_or_left(1),mes_pose_str.error_or_left(2);
  }

  //nao esta a fazer nada esta linha!!
  //Vector3d des_hand_orr_q_e_vec; des_hand_orr_q_e_vec << des_pose.des_hand_or_q_x,des_pose.des_hand_or_q_y,des_pose.des_hand_or_q_z;
  //Vector3d error_f_orr_1 = des_hand_or_q_w*hand_or_q_e_init_vec - hand_or_q_w_init_vec*des_hand_orr_q_e_vec - hand_or_q_e_init_vec.cross(des_hand_orr_q_e_vec);

//                VectorXd error_tot_init(6);
//                // error in velocity
//                Vector3d error_lin_vel = des_hand_lin_vel - r_hand_lin_vel_vec;
//                Vector3d error_ang_vel = r_hand_ang_vel_q_w*des_hand_or_q_e + r_hand_q_w*des_hand_ang_vel_q_e - des_hand_ang_vel_q_w*r_hand_or_q_e - des_hand_or_q_w*r_hand_ang_vel_q_e
//                                        - des_hand_ang_vel_q_e.cross(r_hand_or_q_e) - des_hand_or_q_e.cross(r_hand_ang_vel_q_e);
//                VectorXd der_error_tot(6); der_error_tot << error_lin_vel(0),error_lin_vel(1),error_lin_vel(2),error_ang_vel(0),error_ang_vel(1),error_ang_vel(2);
}

/******************************************************
*			Recording			*
* Records all the data from the robot during the	*
* simulation for later analysis.			*
*  							*
* Does not infer with the robots movement and thus	*
* should be only used if necessary to avoid delays	*
* 							*
*******************************************************/
void LocalPlanner::Recording() {

    // record the positions of the joints
    ctx.jointsPosition_ctrl.conservativeResize(ctx.jointsPosition_ctrl.rows()+1,JOINTS_ARM+JOINTS_HAND);
    for(size_t jj=0; jj < mes_pose_str.r_arm_posture_mes.size(); ++jj)
        ctx.jointsPosition_ctrl(ctx.jointsPosition_ctrl.rows()-1,jj) = mes_pose_str.r_arm_posture_mes.at(jj);
    for(size_t jj=0; jj < mes_pose_str.r_hand_posture_mes.size(); ++jj)
        ctx.jointsPosition_ctrl(ctx.jointsPosition_ctrl.rows()-1,mes_pose_str.r_arm_posture_mes.size()+jj) = mes_pose_str.r_hand_posture_mes.at(jj);

    // record the velocities of the joints
    ctx.jointsVelocity_ctrl.conservativeResize(ctx.jointsVelocity_ctrl.rows()+1,JOINTS_ARM+JOINTS_HAND);
    for(size_t jj=0; jj < mes_pose_str.r_arm_velocities_read.size(); ++jj)
        ctx.jointsVelocity_ctrl(ctx.jointsVelocity_ctrl.rows()-1,jj) = mes_pose_str.r_arm_velocities_read.at(jj);
    for(size_t jj=0; jj < mes_pose_str.r_hand_velocities_read.size(); ++jj)
        ctx.jointsVelocity_ctrl(ctx.jointsVelocity_ctrl.rows()-1,mes_pose_str.r_arm_velocities_read.size()+jj) = mes_pose_str.r_hand_velocities_read.at(jj);

    ctx.jointsVelocity_null_ctrl.conservativeResize(ctx.jointsVelocity_null_ctrl.rows()+1,JOINTS_ARM);
    for(size_t jj=0; jj < mes_pose_str.r_arm_null_velocities.size(); ++jj)
        ctx.jointsVelocity_null_ctrl(ctx.jointsVelocity_null_ctrl.rows()-1,jj) = mes_pose_str.r_arm_null_velocities(jj);


    // record the acceleration of the joints
    ctx.jointsAcceleration_ctrl.conservativeResize(ctx.jointsAcceleration_ctrl.rows()+1,JOINTS_ARM+JOINTS_HAND);
    for(size_t jj=0; jj < mes_pose_str.r_arm_accelerations_read.size(); ++jj)
        ctx.jointsAcceleration_ctrl(ctx.jointsAcceleration_ctrl.rows()-1,jj) = mes_pose_str.r_arm_accelerations_read.at(jj);
    for(size_t jj=0; jj < mes_pose_str.r_hand_accelerations_read.size(); ++jj)
        ctx.jointsAcceleration_ctrl(ctx.jointsAcceleration_ctrl.rows()-1,mes_pose_str.r_arm_accelerations_read.size()+jj) = mes_pose_str.r_hand_accelerations_read.at(jj);

    // desired hand pose
    vector<double> std_h_hand_des_pose(des_pose.h_hand_pose.size());
    VectorXd::Map(&std_h_hand_des_pose[0], des_pose.h_hand_pose.size()) = des_pose.h_hand_pose;
    ctx.handPosition_des_ctrl.push_back(std_h_hand_des_pose);
    // desired hand velocity
    vector<double> std_h_hand_des_vel(des_pose.h_hand_vel.size());
    VectorXd::Map(&std_h_hand_des_vel[0], des_pose.h_hand_vel.size()) = des_pose.h_hand_vel;
    ctx.handVelocity_des_ctrl.push_back(std_h_hand_des_vel);
    // desired hand acceleration
    vector<double> std_h_hand_des_acc(des_pose.h_hand_acc.size());
    VectorXd::Map(&std_h_hand_des_acc[0], des_pose.h_hand_acc.size()) = des_pose.h_hand_acc;
    ctx.handAcceleration_des_ctrl.push_back(std_h_hand_des_acc);

    // desired fingers positions
    vector<double> std_h_fing_pos(des_pose.h_fing_pos.size());
    VectorXd::Map(&std_h_fing_pos[0], des_pose.h_fing_pos.size()) = des_pose.h_fing_pos;
    ctx.fingPosition_des_ctrl.push_back(std_h_fing_pos);
    // fingers positions
    ctx.fingPosition_ctrl.push_back(mes_pose_str.r_hand_posture_mes);
    // desired fingers velocities
    vector<double> std_h_fing_vel(des_pose.h_fing_vel.size());
    VectorXd::Map(&std_h_fing_vel[0], des_pose.h_fing_vel.size()) = des_pose.h_fing_vel;
    ctx.fingVelocity_des_ctrl.push_back(std_h_fing_vel);
    // fingers velocities
    ctx.fingVelocity_ctrl.push_back(mes_pose_str.r_hand_velocities);
    // desired fingers accelerations
    vector<double> std_h_fing_acc(des_pose.h_fing_acc.size());
    VectorXd::Map(&std_h_fing_acc[0], des_pose.h_fing_acc.size()) = des_pose.h_fing_acc;
    ctx.fingAcceleration_des_ctrl.push_back(std_h_fing_acc);
    // fingers accelerations
    ctx.fingAcceleration_ctrl.push_back(mes_pose_str.r_hand_accelerations_read);

    // desired swivel angle position
    ctx.alpha_des_ctrl.push_back(error_var.des_alpha_pos);
    // current swivel angle position
    ctx.alpha_ctrl.push_back(mes_pose_str.alpha_pos_read.at(0));
    // desired swivel angle velocity
    ctx.alpha_des_vel_ctrl.push_back(error_var.des_alpha_vel);
    // current swivel angle velocity
    ctx.alpha_vel_ctrl.push_back(mes_pose_str.alpha_vel_read.at(0));
    // desired swivel angle acceleration
    ctx.alpha_des_acc_ctrl.push_back(error_var.des_alpha_acc);
    // current swivel angle acceleration
    ctx.alpha_acc_ctrl.push_back(mes_pose_str.alpha_acc_read.at(0));

    // operational space positions
    ctx.handPosition_ctrl.push_back(mes_pose_str.r_hand_lin_pos);
    ctx.handOrientation_ctrl.push_back(mes_pose_str.r_hand_ang_pos);
    ctx.handOrientation_q_ctrl.push_back(mes_pose_str.r_hand_q);
    ctx.wristPosition_ctrl.push_back(mes_pose_str.r_wrist_lin_pos);
    ctx.wristOrientation_ctrl.push_back(mes_pose_str.r_wrist_ang_pos);
    ctx.elbowPosition_ctrl.push_back(mes_pose_str.r_elbow_lin_pos);
    ctx.elbowOrientation_ctrl.push_back(mes_pose_str.r_elbow_ang_pos);
    ctx.shoulderPosition_ctrl.push_back(mes_pose_str.r_shoulder_lin_pos);
    ctx.shoulderOrientation_ctrl.push_back(mes_pose_str.r_shoulder_ang_pos);

    // operational space velocities
    ctx.handLinearVelocity_ctrl.push_back(mes_pose_str.r_hand_lin_vel);
    ctx.handAngularVelocity_ctrl.push_back(mes_pose_str.r_hand_ang_vel);
    ctx.wristLinearVelocity_ctrl.push_back(mes_pose_str.r_wrist_lin_vel);
    ctx.wristAngularVelocity_ctrl.push_back(mes_pose_str.r_wrist_ang_vel);
    ctx.elbowLinearVelocity_ctrl.push_back(mes_pose_str.r_elbow_lin_vel);
    ctx.elbowAngularVelocity_ctrl.push_back(mes_pose_str.r_elbow_ang_vel);
    ctx.shoulderLinearVelocity_ctrl.push_back(mes_pose_str.r_shoulder_lin_vel);
    ctx.shoulderAngularVelocity_ctrl.push_back(mes_pose_str.r_shoulder_ang_vel);
    ctx.handVelocityNorm_ctrl.push_back(sqrt(pow(mes_pose_str.r_hand_lin_vel.at(0),2)+pow(mes_pose_str.r_hand_lin_vel.at(1),2)+pow(mes_pose_str.r_hand_lin_vel.at(2),2)));

    // operational space accelerations
    ctx.handLinearAcceleration_ctrl.push_back(mes_pose_str.r_hand_lin_acc);
    ctx.handAngularAcceleration_ctrl.push_back(mes_pose_str.r_hand_ang_acc);
    ctx.wristLinearAcceleration_ctrl.push_back(mes_pose_str.r_wrist_lin_acc);
    ctx.wristAngularAcceleration_ctrl.push_back(mes_pose_str.r_wrist_ang_acc);
    ctx.elbowLinearAcceleration_ctrl.push_back(mes_pose_str.r_elbow_lin_acc);
    ctx.elbowAngularAcceleration_ctrl.push_back(mes_pose_str.r_elbow_ang_acc);
    ctx.shoulderLinearAcceleration_ctrl.push_back(mes_pose_str.r_shoulder_lin_acc);
    ctx.shoulderAngularAcceleration_ctrl.push_back(mes_pose_str.r_shoulder_ang_acc);
    ctx.handAccelerationNorm_ctrl.push_back(sqrt(pow(mes_pose_str.r_hand_lin_acc.at(0),2)+pow(mes_pose_str.r_hand_lin_acc.at(1),2)+pow(mes_pose_str.r_hand_lin_acc.at(2),2)));

    // errors
    if(usr_interface_input.hl_en){
        // hand
        ctx.error_pos_tot_norm.push_back(error_var.error_h_tot.block<3,1>(0,0).norm());
        ctx.error_or_tot_norm.push_back(error_var.error_h_tot.block<3,1>(3,0).norm());
        ctx.error_pos_or_tot_norm.push_back(error_var.error_h_tot.norm());
        ctx.error_lin_vel_tot_norm.push_back(error_var.der_error_h_tot.block<3,1>(0,0).norm());
        ctx.error_ang_vel_tot_norm.push_back(error_var.der_error_h_tot.block<3,1>(3,0).norm());
        ctx.error_vel_tot_norm.push_back(error_var.der_error_h_tot.norm());
        VectorXd error_acc_tot = error_var.h_hand_ref_acc - mes_pose_str.r_hand_acc_read_vec;
        ctx.error_lin_acc_tot_norm.push_back(error_acc_tot.block<3,1>(0,0).norm());
        ctx.error_ang_acc_tot_norm.push_back(error_acc_tot.block<3,1>(3,0).norm());
        ctx.error_acc_tot_norm.push_back(error_acc_tot.norm());
        // fingers
        vector<double> error_h_fing_tot_vec;  error_h_fing_tot_vec.resize(error_var.error_h_fing_tot.size());
        VectorXd::Map(&error_h_fing_tot_vec[0], error_var.error_h_fing_tot.size()) = error_var.error_h_fing_tot;
        ctx.error_fing_pos.push_back(error_h_fing_tot_vec);
        vector<double> der_error_h_fing_tot_vec;  der_error_h_fing_tot_vec.resize(error_var.der_error_h_fing_tot.size());
        VectorXd::Map(&der_error_h_fing_tot_vec[0], error_var.der_error_h_fing_tot.size()) = error_var.der_error_h_fing_tot;
        ctx.error_fing_vel.push_back(der_error_h_fing_tot_vec);
        VectorXd error_fing_acc_tot = error_var.h_fing_ref_acc - mes_pose_str.r_hand_accelerations_read_vec;
        vector<double> error_fing_acc_tot_vec;  error_fing_acc_tot_vec.resize(error_fing_acc_tot.size());
        VectorXd::Map(&error_fing_acc_tot_vec[0], error_fing_acc_tot.size()) = error_fing_acc_tot;
        ctx.error_fing_acc.push_back(error_fing_acc_tot_vec);
        // swivel angle
        ctx.error_alpha_pos.push_back(error_var.error_alpha_pos);
        ctx.error_alpha_vel.push_back(error_var.error_alpha_vel);
        ctx.error_alpha_acc.push_back(error_var.error_alpha_acc);

    }else{
        ctx.error_pos_tot_norm.push_back(error_var.error_trap_tot.block<3,1>(0,0).norm());
        ctx.error_or_tot_norm.push_back(error_var.error_trap_tot.block<3,1>(3,0).norm());
        ctx.error_pos_or_tot_norm.push_back(error_var.error_trap_tot.norm());
        ctx.error_lin_vel_tot_norm.push_back(error_var.der_error_trap_tot.block<3,1>(0,0).norm());
        ctx.error_ang_vel_tot_norm.push_back(error_var.der_error_trap_tot.block<3,1>(3,0).norm());
        ctx.error_vel_tot_norm.push_back(error_var.der_error_trap_tot.norm());
        VectorXd error_acc_tot = error_var.trap_hand_ref_acc - mes_pose_str.r_hand_acc_read_vec;
        ctx.error_lin_acc_tot_norm.push_back(error_acc_tot.block<3,1>(0,0).norm());
        ctx.error_ang_acc_tot_norm.push_back(error_acc_tot.block<3,1>(3,0).norm());
        ctx.error_acc_tot_norm.push_back(error_acc_tot.norm());
    }


    // time
    if(usr_interface_input.sim_robot){
        ctx.t_j_past = qnode.getSimTime();
        ctx.sim_time.push_back(ctx.t_j_past-ctx.t_der_past);
    }else{
        ctx.t_j_past_ctrl = Clock::now();
        Clock::time_point sim_tp = ctx.t_j_past_ctrl - ctx.t_der_past_ctrl;
        double d_sim_time = (boost::chrono::duration_cast<msec>(sim_tp-ctx.start_time_point)).count();
        ctx.sim_time.push_back(d_sim_time/1000);
    }
}



double LocalPlanner::getNoiseRobustDerivate(int N, double h, std::deque<double>& buff)
{
    // http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/#noiserobust_2
    static const std::runtime_error ex_cap(std::string("The capacity of the buffer differs the given filter length"));
    static const std::runtime_error ex_length(std::string("The filter length must be greater or equal to 5"));
    if(N>=5)
    {
        if(N==buff.size())
        {
            int M = (N-1)/2;
            int m = (N-3)/2;
            double sum=0.0;
            for(int k=1;k<=M;++k)
            {
                double c1 = this->binomialCoeff(2*m,(m-k+1));
                double c2 = this->binomialCoeff(2*m,(m-k-1));
                double ck = (1/pow(2,((2*m)+1)))*(c1-c2);
                sum+=ck*(buff[M+k]-buff[M-k]);
            }
            double der = sum/h;
            return der;
        }else{
            throw ex_cap;
        }
    }else{
        throw ex_length;
    }
}

int LocalPlanner::binomialCoeff(int n, int k)
{
    // https://www.geeksforgeeks.org/binomial-coefficient-dp-9/

    if (k>=0 && k<=n)
    {
        int C[n + 1][k + 1];
        int i, j;

        // Caculate value of Binomial Coefficient
        // in bottom up manner
        for (i = 0; i <= n; i++)
        {
            for (j = 0; j <= min(i, k); j++)
            {
                // Base Cases
                if (j == 0 || j == i)
                    C[i][j] = 1;

                // Calculate value using previosly
                // stored values
                else
                    C[i][j] = C[i - 1][j - 1] +
                              C[i - 1][j];
            }
        }

        return C[n][k];
    }else{
        return 0;
    }
}

