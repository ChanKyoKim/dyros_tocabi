#include "dyros_red_controller/red_controller.h"
#include "dyros_red_controller/terminal.h"
#include "dyros_red_controller/wholebody_controller.h"
#include "dyros_red_msgs/TaskCommand.h"
#include <tf/transform_datatypes.h>
#include "stdlib.h"
#include <fstream>

#include <iostream>
#include <string>

std::mutex mtx;
std::mutex mtx_rbdl;
std::mutex mtx_dc;
std::mutex mtx_terminal;
std::mutex mtx_ncurse;

RedController::RedController(DataContainer &dc_global, StateManager &sm, DynamicsManager &dm) : dc(dc_global), s_(sm), d_(dm), red_(dc_global.red_)
{
    initialize();

    task_command = dc.nh.subscribe("/dyros_red/taskcommand", 100, &RedController::TaskCommandCallback, this);
    point_pub = dc.nh.advertise<geometry_msgs::PolygonStamped>("/dyros_red/cdata_pub", 1);
    pointpub_msg.polygon.points.resize(13);
    point_pub2 = dc.nh.advertise<geometry_msgs::PolygonStamped>("/dyros_red/point_pub2", 1);
    //pointpub_msg.polygon.points.reserve(20);
}

void RedController::pubfromcontroller()
{
    /*
    * Point pub info : 
    * 0 : com position
    * 1 : com velocity
    * 2 : com acceleration
    * 3 : com angular momentum
    * 4 : com position desired
    * 5 : com velocity desired
    * 6 : com acceleration desired
    * 7 : fstar
    */

    pointpub_msg.header.stamp = ros::Time::now();

    geometry_msgs::Point32 point;

    point.x = dc.red_.com_.pos(0);
    point.y = dc.red_.com_.pos(1);
    point.z = dc.red_.com_.pos(2);
    pointpub_msg.polygon.points[0] = point;

    point.x = dc.red_.com_.vel(0);
    point.y = dc.red_.com_.vel(1);
    point.z = dc.red_.com_.vel(2);
    pointpub_msg.polygon.points[1] = point;

    point.x = dc.red_.com_.accel(0);
    point.y = dc.red_.com_.accel(1);
    point.z = dc.red_.com_.accel(2);
    //pointpub_msg.polygon.points.push_back(point);

    point.x = dc.red_.com_.angular_momentum(0);
    point.y = dc.red_.com_.angular_momentum(1);
    point.z = dc.red_.com_.angular_momentum(2);
    //pointpub_msg.polygon.points.push_back(point);

    point.x = dc.red_.link_[COM_id].x_traj(0);
    point.y = dc.red_.link_[COM_id].x_traj(1);
    point.z = dc.red_.link_[COM_id].x_traj(2);
    //pointpub_msg.polygon.points.push_back(point);

    point.x = dc.red_.link_[COM_id].v_traj(0);
    point.y = dc.red_.link_[COM_id].v_traj(1);
    point.z = dc.red_.link_[COM_id].v_traj(2);
    //pointpub_msg.polygon.points.push_back(point);

    point.x = dc.red_.link_[COM_id].a_traj(0);
    point.y = dc.red_.link_[COM_id].a_traj(1);
    point.z = dc.red_.link_[COM_id].a_traj(2);
    //pointpub_msg.polygon.points.push_back(point);

    point.x = dc.red_.fstar(0);
    point.y = dc.red_.fstar(1);
    point.z = dc.red_.fstar(2);
    //pointpub_msg.polygon.points.push_back(point);

    point.x = dc.red_.ZMP(0); //from task torque -> contact force -> zmp
    point.y = dc.red_.ZMP(1);
    point.z = dc.red_.ZMP(2);
    pointpub_msg.polygon.points[2] = point;

    point.x = dc.red_.ZMP_local(0); //from acceleration trajecoty -> tasktorque -> contactforce -> zmp
    point.y = dc.red_.ZMP_local(1);
    point.z = dc.red_.ZMP_local(2);
    pointpub_msg.polygon.points[3] = point;

    point.x = dc.red_.ZMP_eqn_calc(0); //from acceleration trajectory with zmp equation : xddot = w^2(x-p),  zmp = x - xddot/w^2
    point.y = dc.red_.ZMP_eqn_calc(1);
    point.z = dc.red_.ZMP_eqn_calc(2);
    pointpub_msg.polygon.points[4] = point;

    point.x = dc.red_.ZMP_desired(0); //from acceleration trajectory with zmp equation : xddot = w^2(x-p),  zmp = x - xddot/w^2
    point.y = dc.red_.ZMP_desired(1);
    point.z = dc.red_.ZMP_desired(2);
    pointpub_msg.polygon.points[5] = point;

    point.x = dc.red_.ZMP_ft(0); //calc from ft sensor
    point.y = dc.red_.ZMP_ft(1);
    point.z = dc.red_.ZMP_ft(2);
    pointpub_msg.polygon.points[6] = point;

    point.x = dc.red_.LH_FT(0);
    point.y = dc.red_.LH_FT(1);
    point.z = dc.red_.LH_FT(2);
    pointpub_msg.polygon.points[7] = point;

    if (dc.red_.ContactForce.size() == 18)
    {
        point.x = dc.red_.ContactForce(12 + 0);
        point.y = dc.red_.ContactForce(12 + 1);
        point.z = dc.red_.ContactForce(12 + 2);
    }
    else
    {
        point.x = 0.0;
        point.y = 0.0;
        point.z = 0.0;
    }

    pointpub_msg.polygon.points[8] = point;

    point.x = dc.red_.link_[Left_Hand].xpos_contact(0);
    point.y = dc.red_.link_[Left_Hand].xpos_contact(1);
    point.z = dc.red_.link_[Left_Hand].xpos_contact(2);
    pointpub_msg.polygon.points[9] = point;

    point_pub.publish(pointpub_msg);
}

void RedController::TaskCommandCallback(const dyros_red_msgs::TaskCommandConstPtr &msg)
{
    tc.command_time = control_time_;
    tc.traj_time = msg->time;
    tc.ratio = msg->ratio;
    tc.angle = msg->angle;
    tc.height = msg->height;
    tc.mode = msg->mode;
    tc.task_init = true;

    red_.link_[Right_Foot].Set_initpos();
    red_.link_[Left_Foot].Set_initpos();
    red_.link_[Right_Hand].Set_initpos();
    red_.link_[Left_Hand].Set_initpos();
    red_.link_[Pelvis].Set_initpos();
    red_.link_[Upper_Body].Set_initpos();
    red_.link_[COM_id].Set_initpos();

    task_switch = true;

    std::cout << "init set - COM x : " << red_.link_[COM_id].x_init(0) << "\t y : " << red_.link_[COM_id].x_init(1) << std::endl;
    //????????? ?????? ????????? 
    std::cout << "###############  COMMAND RECEIVED  ###############" << std::endl;
}

void RedController::stateThread()
{
    s_.connect();
    s_.stateThread();
}

void RedController::dynamicsThreadHigh()
{
    std::cout << "DynamicsThreadHigh : READY ?" << std::endl;
    ros::Rate r(4000);
    while ((!dc.connected) && (!dc.shutdown) && ros::ok())
    {
        r.sleep();
    }
    while ((!dc.firstcalcdone) && (!dc.shutdown) && ros::ok())
    {
        r.sleep();
    }
    std::cout << "DynamicsThreadHigh : START" << std::endl;
    std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
    int cnt = 0;
    std::chrono::duration<double> time_now = std::chrono::high_resolution_clock::now() - start_time;
    bool set_q_init = true;
    while (!dc.shutdown && ros::ok())
    {
        time_now = std::chrono::high_resolution_clock::now() - start_time;

        if (dc.mode == "simulation")
        {
        }
        else if (dc.mode == "realrobot")
        {
        }
        if (dc.positionControl)
        {
            if (set_q_init)
            {
                q_desired_ = q_;
                set_q_init = false;
            }
            for (int i = 0; i < MODEL_DOF; i++)
            {
                torque_desired(i) = Kps[i] * (q_desired_(i) - q_(i)) - Kvs[i] * (q_dot_(i));
            }
        }

        if (dc.emergencyoff)
        {
            torque_desired.setZero();
        }
        cnt++;
        mtx.lock();
        if ((abs(red_.roll) > 30.0 / 180.0 * 3.141592) || (abs(red_.pitch) > 30.0 / 180.0 * 3.141592))
        {
            //torque_desired.setZero();
        }
        s_.sendCommand(torque_desired, sim_time);
        mtx.unlock();

        r.sleep();
    }
}

void RedController::dynamicsThreadLow()
{
    std::cout << "DynamicsThreadLow : READY ?" << std::endl;
    ros::Rate r(2000);
    int calc_count = 0;
    int ThreadCount = 0;
    int i = 1;

    while ((!dc.connected) && (!dc.shutdown) && ros::ok())
    {
        r.sleep();
    }
    while ((!dc.firstcalcdone) && (!dc.shutdown) && ros::ok())
    {
        r.sleep();
    }

    Wholebody_controller wc_(dc, red_);

    std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
    std::chrono::seconds sec1(1);

    std::chrono::duration<double> sec = std::chrono::high_resolution_clock::now() - start_time;
    bool display = false;
    std::chrono::high_resolution_clock::time_point start_time2 = std::chrono::high_resolution_clock::now();

    int contact_number = 2;
    int link_id[contact_number];
    int total_dof_ = MODEL_DOF;
    Eigen::MatrixXd Lambda_c, J_C_INV_T, N_C, I37, Slc_k, Slc_k_T, W, W_inv;
    Eigen::MatrixXd P_c;
    Eigen::VectorXd Grav_ref;
    Eigen::VectorXd G;
    Eigen::MatrixXd J_g;
    Eigen::MatrixXd aa;
    Eigen::VectorXd torque_grav, torque_task;
    Eigen::MatrixXd ppinv;
    Eigen::MatrixXd tg_temp;
    Eigen::MatrixXd A_matrix_inverse;
    Eigen::MatrixXd J_C;
    Eigen::MatrixXd J_C_temp;
    Eigen::MatrixXd Jcon[2];
    Eigen::VectorXd qtemp[2], conp[2];

    Eigen::MatrixXd J_task;
    Eigen::VectorXd f_star;
    Eigen::MatrixXd J_lefthand;
    Eigen::MatrixXd J_test;
    Eigen::MatrixXd J_test_inverse;

    int task_number;

    J_C.setZero(contact_number * 6, MODEL_DOF_VIRTUAL);
    N_C.setZero(total_dof_ + 6, MODEL_DOF_VIRTUAL);
    bool first = true;

    Eigen::Vector3d task_desired;

    acceleration_estimated_before.setZero();
    q_dot_before_.setZero();

    VectorQd TorqueDesiredLocal, TorqueContact;
    TorqueDesiredLocal.setZero();
    TorqueContact.setZero();

    Vector12d fc_redis;
    double fc_ratio;
    fc_redis.setZero();

    Vector3d kp_, kd_, kpa_, kda_;
    for (int i = 0; i < 3; i++)
    {
        kp_(i) = 400;
        kd_(i) = 40;
        kpa_(i) = 400;
        kda_(i) = 40;
    }

    //kd_(1) = 120;

    std::cout << "DynamicsThreadLow : START" << std::endl;
    int dynthread_cnt = 0;

    //const char *file_name = "/home/saga/sim_data.txt";

    std::string path = "/home/saga/red_sim_data/";
    std::string current_time;

    time_t now = std::time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
    current_time = buf;
    std::string file_name = path + "sim_data" + current_time + ".txt";
    out = std::ofstream(file_name.c_str());

    //Control Loop Start
    while (!dc.shutdown && ros::ok())
    {
        static double est;
        std::chrono::high_resolution_clock::time_point dyn_loop_start = std::chrono::high_resolution_clock::now();
        dynthread_cnt++;
        if (control_time_ == 0)
        {
            first = true;
            task_switch = false;
        }
        if ((dyn_loop_start - start_time2) > sec1)
        {
            start_time2 = std::chrono::high_resolution_clock::now();
            if (dc.checkfreqency)
                std::cout << "dynamics thread : " << dynthread_cnt << " hz, time : " << est << std::endl;
            dynthread_cnt = 0;
            est = 0;
        }
        getState(); //link data override

        //Task link gain setting.
        red_.link_[COM_id].pos_p_gain = kp_;
        red_.link_[COM_id].pos_d_gain = kd_;
        red_.link_[COM_id].rot_p_gain = red_.link_[Pelvis].rot_p_gain = kpa_;
        red_.link_[COM_id].rot_d_gain = red_.link_[Pelvis].rot_d_gain = kda_;
        red_.link_[Pelvis].pos_p_gain = kp_;
        red_.link_[Pelvis].pos_d_gain = kd_;

        red_.link_[Right_Foot].pos_p_gain = red_.link_[Left_Foot].pos_p_gain = kp_;
        red_.link_[Right_Foot].pos_d_gain = red_.link_[Left_Foot].pos_d_gain = kd_;
        red_.link_[Right_Foot].rot_p_gain = red_.link_[Left_Foot].rot_p_gain = kpa_;
        red_.link_[Right_Foot].rot_d_gain = red_.link_[Left_Foot].rot_d_gain = kda_;

        red_.link_[Right_Hand].pos_p_gain = red_.link_[Left_Hand].pos_p_gain = kp_;
        red_.link_[Right_Hand].pos_d_gain = red_.link_[Left_Hand].pos_d_gain = kd_;
        red_.link_[Right_Hand].rot_p_gain = red_.link_[Left_Hand].rot_p_gain = kpa_ * 4;
        red_.link_[Right_Hand].rot_d_gain = red_.link_[Left_Hand].rot_d_gain = kda_ * 2;

        red_.link_[Upper_Body].rot_p_gain = kpa_ * 4;
        red_.link_[Upper_Body].rot_d_gain = kda_ * 2;

        red_.link_[COM_id].pos_p_gain = kp_;
        red_.link_[COM_id].pos_d_gain = kd_;

        wc_.init(red_);
        wc_.update(red_);

        sec = std::chrono::high_resolution_clock::now() - start_time;
        if (sec.count() - control_time_ > 0.01)
        {
            // std::cout << "diff ::" << sec.count() - control_time_ << std::endl; //<<" dyn_low current time : " << control_time_ << "   chrono : " << sec.count() << std::endl;
        }
        ///////////////////////////////////////////////////////////////////////////////////////
        /////////////              Controller Code Here !                     /////////////////
        ///////////////////////////////////////////////////////////////////////////////////////

        torque_task.setZero(MODEL_DOF);
        TorqueContact.setZero();

        if (dc.gravityMode)
        {
            std::cout << "Task Turned Off,, gravity compensation only !" << std::endl;
            task_switch = false;
            dc.gravityMode = false;
        }

        if (task_switch)
        {
            if (tc.mode == 0) //Pelvis position control
            {
                wc_.set_contact(red_, 1, 1);

                //torque_grav = wc_.gravity_compensation_torque(red_, dc.fixedgravity);
                //torque_grav = wc_.gravity_compensation_torque(dc.fixedgravity);
                task_number = 6;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task = red_.link_[Pelvis].Jac;

                red_.link_[Pelvis].x_desired = tc.ratio * red_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * red_.link_[Right_Foot].xpos;
                red_.link_[Pelvis].x_desired(2) = tc.height + tc.ratio * red_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * red_.link_[Right_Foot].xpos(2);
                red_.link_[Pelvis].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);

                //red_.link_[Pelvis].rot_desired = Matrix3d::Identity();

                f_star = wc_.getfstar6d(red_, Pelvis);
                //torque_task = wc_.task_control_torque(red_, J_task, f_star);
                torque_grav.setZero();
                torque_task = wc_.task_control_torque_QP2(red_, J_task, f_star);

                cr_mode = 2;
                //torque_task = wc_.task_control_torque(J_task, f_star);
            }
            else if (tc.mode == 1) //COM position control
            {
                wc_.set_contact(red_, 1, 1);

                //torque_grav = wc_.gravity_compensation_torque(red_, dc.fixedgravity);
                //torque_grav = wc_.gravity_compensation_torque(dc.fixedgravity);
                task_number = 6;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task = red_.link_[COM_id].Jac;
                J_task.block(0, 0, 3, MODEL_DOF_VIRTUAL) = red_.link_[COM_id].Jac_COM_p;

                red_.link_[COM_id].x_desired = tc.ratio * red_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * red_.link_[Right_Foot].xpos;
                red_.link_[COM_id].x_desired(2) = tc.height + tc.ratio * red_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * red_.link_[Right_Foot].xpos(2);
                red_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);

                //red_.link_[Pelvis].rot_desired = Matrix3d::Identity();

                f_star = wc_.getfstar6d(red_, COM_id);
                //std::cout<<"f_star : "<<std::endl<<red_.link_[COM_id].f_star<<std::endl;
                //torque_task = wc_.task_control_torque(red_, J_task, f_star);

                torque_grav.setZero();
                //std::chrono::high_resolution_clock::time_point stc = std::chrono::high_resolution_clock::now();
                torque_task = wc_.task_control_torque_QP2(red_, J_task, f_star);

                //std::chrono::duration<double> slc = std::chrono::high_resolution_clock::now() - stc;
                //std::cout << "time spend : " << slc.count() << std::endl;
                cr_mode = 2;

                /*
                MatrixXd lambda_;
                std::cout << "####################" << std::endl;
                lambda_ = (J_task * red_.A_matrix_inverse * J_task.transpose()).inverse();

                MatrixXd jtinv_;
                jtinv_ = (red_.A_matrix_inverse * J_task.transpose() * lambda_).transpose();
                jtinv_ = lambda_ * J_task * red_.A_matrix_inverse;

                std::cout << "####################" << std::endl;
                std::cout << "operational space : " << std::endl
                          << lambda_ * f_star + jtinv_ * red_.G << std::endl;
                std::cout << "contact consistant : " << std::endl
                          << red_.lambda * f_star + red_.J_task_inv_T * red_.G << std::endl;

                //torque_task = wc_.task_control_torque(J_task, f_star);
                f_star.segment(0, 3) = red_.link_[COM_id].a_traj;

                TorqueDesiredLocal = wc_.task_control_torque(red_, J_task, f_star) + torque_grav;

                torque_grav.setZero();
                //TorqueDesiredLocal = wc_.task_control_torque_QP(red_, J_task, f_star);
                cr_mode = 2;

                red_.ContactForce = wc_.get_contact_force(red_, TorqueDesiredLocal);
                red_.ZMP_local = wc_.GetZMPpos(red_);

                red_.ZMP_ft = wc_.GetZMPpos_fromFT(red_);*/

                //std::cout << "contact force from controller :::: " << std::endl
                //          << wc_.get_contact_force(red_, torque_task) << std::endl;
            }
            else if (tc.mode == 2) //COM to Left foot, then switch double support to single support
            {
                if (control_time_ < tc.command_time + tc.traj_time)
                {
                    wc_.set_contact(red_, 1, 1);
                    if (tc.ratio >= 0.5)
                    {
                        red_.link_[COM_id].x_desired = red_.link_[Left_Foot].xpos;
                        red_.link_[COM_id].x_desired(2) = tc.height + red_.link_[Left_Foot].xpos(2);
                    }
                    else if (tc.ratio < 0.5)
                    {
                        red_.link_[COM_id].x_desired = red_.link_[Right_Foot].xpos;
                        red_.link_[COM_id].x_desired(2) = tc.height + red_.link_[Right_Foot].xpos(2);
                    }
                }
                else if (tc.ratio < 0.5)
                {
                    wc_.set_contact(red_, 0, 1);
                }
                else if (tc.ratio > 0.5)
                {
                    wc_.set_contact(red_, 1, 0);
                }

                torque_grav = wc_.gravity_compensation_torque(red_, dc.fixedgravity);
                task_number = 6;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task = red_.link_[COM_id].Jac;

                red_.link_[COM_id].rot_desired = Matrix3d::Identity();

                red_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                f_star = wc_.getfstar6d(red_, COM_id);

                torque_task = wc_.task_control_torque(red_, J_task, f_star);
            }
            else if (tc.mode == 3) //COM to Left foot, then switch double support to single support while holding com rotation.
            {
                if (control_time_ < tc.command_time + tc.traj_time)
                {
                    wc_.set_contact(red_, 1, 1);
                }
                else
                {
                    wc_.set_contact(red_, 1, 0);
                }

                torque_grav = wc_.gravity_compensation_torque(red_, dc.fixedgravity);
                task_number = 6;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task = red_.link_[COM_id].Jac;

                red_.link_[COM_id].x_desired = red_.link_[Left_Foot].xpos;
                red_.link_[COM_id].x_desired(2) = tc.height + red_.link_[Left_Foot].xpos(2);
                red_.link_[COM_id].rot_desired = Matrix3d::Identity();

                red_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                red_.link_[COM_id].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);
                f_star = wc_.getfstar6d(red_, COM_id);

                torque_task = wc_.task_control_torque(red_, J_task, f_star);
            }
            else if (tc.mode == 4) //left foot controller
            {
                //if(red_.contact_part[0].)
                int task_link;
                if (red_.ee_[0].contact)
                {
                    wc_.set_contact(red_, 1, 0);
                    task_link = Right_Foot;
                }
                else if (red_.ee_[1].contact)
                {
                    wc_.set_contact(red_, 0, 1);
                    task_link = Left_Foot;
                }

                torque_grav = wc_.gravity_compensation_torque(red_, dc.fixedgravity);
                task_number = 12;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);
                J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = red_.link_[COM_id].Jac;

                J_task.block(6, 0, 6, MODEL_DOF_VIRTUAL) = red_.link_[task_link].Jac;
                red_.link_[COM_id].x_desired = red_.link_[COM_id].x_init;
                red_.link_[COM_id].rot_desired = Matrix3d::Identity();

                red_.link_[task_link].x_desired(0) = red_.link_[task_link].x_init(0) + tc.ratio;
                red_.link_[task_link].x_desired(1) = red_.link_[task_link].x_init(1);
                red_.link_[task_link].x_desired(2) = red_.link_[task_link].x_init(2) + tc.height;
                red_.link_[task_link].rot_desired = Matrix3d::Identity();

                red_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                red_.link_[COM_id].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                red_.link_[task_link].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                red_.link_[task_link].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                f_star.segment(0, 6) = wc_.getfstar6d(red_, COM_id);
                f_star.segment(6, 6) = wc_.getfstar6d(red_, task_link);

                torque_task = wc_.task_control_torque(red_, J_task, f_star);
                //red_.link_[Right_Foot].x_desired = tc.
            }
            else if (tc.mode == 5)
            {

                if (red_.ee_[0].contact && red_.ee_[1].contact)
                {
                    red_.ContactForce = red_.ContactForce_FT;
                    red_.ZMP_ft = wc_.GetZMPpos(red_);
                }
                else if (red_.ee_[0].contact)
                {
                    red_.ContactForce = red_.ContactForce_FT.segment(0, 6);
                    red_.ZMP_ft = wc_.GetZMPpos(red_);
                }
                else if (red_.ee_[1].contact)
                {
                    red_.ContactForce = red_.ContactForce_FT.segment(6, 6);
                    red_.ZMP_ft = wc_.GetZMPpos(red_);
                }

                red_.ZMP_error = red_.ZMP_desired - red_.ZMP_ft;

                wc_.set_contact(red_, 1, 1);
                torque_grav = wc_.gravity_compensation_torque(red_, dc.fixedgravity);
                task_number = 6;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = red_.link_[COM_id].Jac;

                red_.link_[COM_id].x_desired = tc.ratio * red_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * red_.link_[Right_Foot].xpos;
                red_.link_[COM_id].x_desired(2) = tc.height + tc.ratio * red_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * red_.link_[Right_Foot].xpos(2);
                red_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);

                double r_dis, l_dis;

                r_dis = abs(red_.link_[COM_id].xpos(1) - red_.link_[Right_Foot].xpos(1));
                l_dis = abs(red_.link_[COM_id].xpos(1) - red_.link_[Left_Foot].xpos(1));

                bool transition;
                double t = 1.0;
                static double com_height = red_.com_.pos(2) - (red_.link_[Right_Foot].xpos(2) + red_.link_[Left_Foot].xpos(2)) * 0.5;

                double tc = sqrt(com_height / 9.81);
                static double st;
                double tf = 1.0;
                static int left_c = 1;
                static int right_c = 1;
                static int left_t = 0;
                static int right_t = 0;
                static int step = 0;
                static double tn;

                double d_t = 1.0;
                double t_gain = 0.05;
                double freeze_tick = 10;

                int transition_step = 100;
                int transition_step2 = 200;

                double f_star_y;

                if (l_dis < r_dis)
                {
                    //red_.ZMP_desired(1) = red_.link_[Left_Foot].xpos(1);
                    tn = control_time_ - st;
                    if (right_c == 1)
                    {
                        st = control_time_;
                    }
                    if (step > transition_step2)
                    {
                        red_.ZMP_desired(1) = (red_.link_[COM_id].xpos(1) * cosh((tf - tn) / tc) + tc * red_.link_[COM_id].v(1) * sinh((tf - tn) / tc)) / (cosh((tf - tn) / tc) - 1);
                        std::cout << "red_.ZMP_desired : " << red_.ZMP_desired(1) << std::endl;
                    }

                    if (right_c == 1)
                    {
                        if (step <= transition_step2)
                        {
                            red_.ZMP_desired(1) = red_.link_[Left_Foot].xpos(1);
                        }
                        else
                        {

                            std::cout << "transition!" << std::endl;
                        }
                        if (step > transition_step)
                        {
                            red_.ZMP_desired(1) = (red_.link_[COM_id].xpos(1) * cosh(tf / tc) + tc * red_.link_[COM_id].v(1) * sinh(tf / tc)) / (cosh(tf / tc) - 1);
                        }

                        right_c = 0;
                        red_.ZMP_error(1) = 0.0;
                        std::cout << step << " left close : " << tn << " des zmp : " << red_.ZMP_desired(1) << "com p : " << red_.link_[COM_id].xpos(1) << " com v : " << red_.link_[COM_id].v(1) << std::endl;
                        red_.ZMP_command(1) = red_.ZMP_desired(1);
                        right_t = 0;
                        step++;
                        if ((step > 3) && (tn > d_t))
                        {
                            //std::cout<< (red_.link_[COM_id].xpos(1) * cosh(tn / tc) + tc * red_.link_[COM_id].v(1) * sinh(tn / tc) - 0.0) / (cosh(tn / tc) - 1) - (red_.link_[COM_id].xpos(1) * cosh(d_t / tc) + tc * red_.link_[COM_id].v(1) * sinh(d_t / tc) - 0.0) / (cosh(d_t / tc) - 1)<<std::endl;
                            //red_.ZMP_mod(1) = (red_.link_[COM_id].xpos(1) * cosh(d_t / tc) + tc * red_.link_[COM_id].v(1) * sinh(d_t / tc) - 0.0) / (cosh(d_t / tc) - 1) - (red_.link_[COM_id].xpos(1) * cosh(tn / tc) + tc * red_.link_[COM_id].v(1) * sinh(tn / tc) - 0.0) / (cosh(tn / tc) - 1);
                            red_.ZMP_mod = red_.ZMP_desired * (tn - d_t) * t_gain;
                            red_.ZMP_desired(1) = red_.ZMP_desired(1) + red_.ZMP_mod(1);
                            red_.ZMP_command(1) = red_.ZMP_desired(1);
                            std::cout << "lc add : " << red_.ZMP_mod(1) << std::endl;
                        }
                        else
                        {
                            red_.ZMP_mod(1) = 0;
                        }
                    }

                    if (right_t < freeze_tick)
                    {
                        red_.ZMP_error(1) = 0.0;
                        right_t++;
                    }

                    //tf = 2.0 - control_time_ + st;
                    //if (tf < 0)
                    //    tf = 0;
                    //ZMP_desired(1) = (red_.link_[COM_id].xpos(1) * cosh(tf / tc) + tc * red_.link_[COM_id].v(1) * sinh(tf / tc) - 0.0) / (cosh(tf / tc) - 1);

                    //tf = 2.0 - control_time_ + st;
                    //ZMP_desired(1) = (red_.link_[COM_id].xpos(1) * cosh(tf/tc) + tc * red_.link_[COM_id].v(1) * sinh(tf/tc) -0.0)/(cosh(tf/tc)-1);

                    left_c = 1;
                }
                else if (r_dis < l_dis)
                {
                    tn = control_time_ - st;
                    //red_.ZMP_desired(1) = red_.link_[Right_Foot].xpos(1);
                    if (left_c == 1)
                    {
                        st = control_time_;
                    }
                    if (step > transition_step2)
                    {
                        red_.ZMP_desired(1) = (red_.link_[COM_id].xpos(1) * cosh((tf - tn) / tc) + tc * red_.link_[COM_id].v(1) * sinh((tf - tn) / tc)) / (cosh((tf - tn) / tc) - 1);
                        std::cout << "red_.ZMP_desired : " << red_.ZMP_desired(1) << std::endl;
                    }

                    if (left_c == 1)
                    {
                        if (step <= transition_step2)
                        {
                            red_.ZMP_desired(1) = red_.link_[Right_Foot].xpos(1);
                        }
                        else
                        {

                            std::cout << "transition!" << std::endl;
                        }

                        if (step > transition_step)
                            red_.ZMP_desired(1) = (red_.link_[COM_id].xpos(1) * cosh(tf / tc) + tc * red_.link_[COM_id].v(1) * sinh(tf / tc)) / (cosh(tf / tc) - 1);

                        left_c = 0;
                        red_.ZMP_error(1) = 0.0;
                        std::cout << step << " right close : " << tn << " des zmp : " << red_.ZMP_desired(1) << "com p : " << red_.link_[COM_id].xpos(1) << " com v : " << red_.link_[COM_id].v(1) << std::endl;
                        red_.ZMP_command(1) = red_.ZMP_desired(1);
                        left_t = 0;
                        step++;
                        if ((step > 3) && (tn > d_t))
                        {
                            //std::cout<< (red_.link_[COM_id].xpos(1) * cosh(tn / tc) + tc * red_.link_[COM_id].v(1) * sinh(tn / tc) - 0.0) / (cosh(tn / tc) - 1) - (red_.link_[COM_id].xpos(1) * cosh(d_t / tc) + tc * red_.link_[COM_id].v(1) * sinh(d_t / tc) - 0.0) / (cosh(d_t / tc) - 1)<<std::endl;
                            //red_.ZMP_mod(1) = (red_.link_[COM_id].xpos(1) * cosh(d_t / tc) + tc * red_.link_[COM_id].v(1) * sinh(d_t / tc) - 0.0) / (cosh(d_t / tc) - 1) - (red_.link_[COM_id].xpos(1) * cosh(tn / tc) + tc * red_.link_[COM_id].v(1) * sinh(tn / tc) - 0.0) / (cosh(tn / tc) - 1);
                            //red_.ZMP_desired(1) = (red_.link_[COM_id].xpos(1) * cosh(tf / tc) + tc * red_.link_[COM_id].v(1) * sinh(tf / tc) - 0.0) / (cosh(tf / tc) - 1);
                            red_.ZMP_mod = red_.ZMP_desired * (tn - d_t) * t_gain;
                            std::cout << "rc add : " << red_.ZMP_mod(1) << std::endl;
                            red_.ZMP_desired(1) = red_.ZMP_desired(1) + red_.ZMP_mod(1) * 6;
                            red_.ZMP_command(1) = red_.ZMP_desired(1);
                        }
                        else
                        {
                            red_.ZMP_mod(1) = 0;
                        }
                        //tf = 2.0 - control_time_ + st;
                    }

                    if (left_t < freeze_tick)
                    {
                        red_.ZMP_error(1) = 0.0;
                        left_t++;
                    }
                    //tf = 2.0 - control_time_ + st;
                    //if (tf < 0)
                    //    tf = 0;
                    //red_.ZMP_desired(1) = (red_.link_[COM_id].xpos(1) * cosh(tf / tc) + tc * red_.link_[COM_id].v(1) * sinh(tf / tc)) / (cosh(tf / tc) - 1);
                    //ZMP_desired(1) = (red_.link_[COM_id].xpos(1) * cosh(tf / tc) + tc * red_.link_[COM_id].v(1) * sinh(tf / tc) - 0.0) / (cosh(tf / tc) - 1);

                    right_c = 1;
                }
                red_.ZMP_desired(0) = red_.link_[COM_id].xpos(0);

                //wc_.set_zmp_control(red_, red_.ZMP_desired.segment(0, 2), 1.0);

                red_.ZMP_command = red_.ZMP_command + 0.05 * red_.ZMP_error; //+ rk_.ZMP_mod;
                f_star_y = 9.81 / (red_.com_.pos(2) - red_.link_[Right_Foot].xpos(2) * 0.5 - red_.link_[Left_Foot].xpos(2) * 0.5) * (red_.com_.pos(1) - red_.ZMP_command(1));

                f_star = wc_.getfstar6d(red_, COM_id);
                f_star(1) = f_star_y;
                torque_task = wc_.task_control_torque(red_, J_task, f_star);
                VectorXd cf_pre = wc_.get_contact_force(red_, torque_task + torque_grav);

                //red_.ZMP_desired.segment(0, 2);
                red_.ZMP_desired(2) = 0.0;
            }
            else if (tc.mode == 6)
            {
                static int loop_temp;
                static int loop_;
                static bool cgen_init;
                static bool loop_cnged;
                static bool walking_init;
                static double foot_height;
                static bool zmperror_reset;

                double footstep_y_length = 0.1024;
                //QP_switch = true;
                red_.ee_[0].contact = true;
                red_.ee_[1].contact = true;

                //right_foot_contact_ = true;
                //left_foot_contact_ = true;
                double time_segment_origin = tc.traj_time;
                static double time_segment = tc.traj_time;
                static double loop_start_time;
                static double loop_end_time;

                if (tc.task_init)
                {
                    std::cout << "Control init!!!!!!!!!!!!!!!" << std::endl;
                    cgen_init = true;
                    loop_cnged = false;
                    walking_init = true;
                    foot_height = (red_.link_[Right_Foot].xpos(2) + red_.link_[Left_Foot].xpos(2)) / 2.0;
                    zmperror_reset = true;
                    time_segment = tc.traj_time;
                    loop_start_time = 0.0;
                    loop_end_time = 0.0;
                    tc.task_init = false;
                }

                double step_length = tc.ratio;

                double task_time = control_time_ - tc.command_time;

                loop_temp = loop_;
                loop_ = (int)(task_time / time_segment);
                double loop_time = task_time - (double)loop_ * time_segment;

                double lr_st, lr_mt, lr_et;
                lr_st = time_segment / 8.0;
                lr_mt = time_segment / 8.0 * 4.0;
                lr_et = time_segment / 8.0 * 7.0;

                if ((double)loop_ > 0.1)
                {
                    if (loop_ % 2)
                    {
                        if ((loop_time < lr_et))
                        {

                            red_.ee_[1].contact = false;
                            red_.ee_[0].contact = true;
                        }
                        else
                        {
                            red_.ee_[1].contact = true;
                            red_.ee_[0].contact = true;
                        }
                    }
                    else
                    {
                        if ((loop_time < lr_et))
                        {
                            red_.ee_[1].contact = true;
                            red_.ee_[0].contact = false;
                        }
                        else
                        {
                            red_.ee_[1].contact = true;
                            red_.ee_[0].contact = true;
                        }
                    }
                }

                //(model_.link_[model_.COM_id].x_init - zmp)*cosh(loop_time/time_segment)+time_segment * model_.link_[model_.COM_id]
                task_desired.setZero();
                task_desired(0) = red_.link_[COM_id].x_init(0);
                task_desired(2) = tc.height + foot_height;

                red_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time, task_desired);

                red_.ZMP_error = red_.ZMP_desired - red_.ZMP_ft;

                // model_.link_[model_.COM_id].x_traj.segment(0, 2) = (cx_init - zmp) * cosh(loop_time * w_) + cv_init * sinh(loop_time * w_) / w_ + zmp;

                // model_.link_[model_.COM_id].v_traj.segment(0, 2) = (cx_init - zmp) * w_ * sinh(loop_time * w_) + cv_init * cosh(loop_time * w_);

                // std::cout << " xtraj : " << std::endl;
                //std::cout << model_.link_[model_.COM_id].x_traj.segment(0, 2) << std::endl;
                //std::cout << "vtrah : " << std::endl;
                //std::cout << model_.link_[model_.COM_id].v_traj.segment(0, 2) << std::endl;

                if (red_.ee_[1].contact && red_.ee_[0].contact)
                {
                    walking_init = true;
                    task_number = 6;
                    J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                    f_star.setZero(task_number);
                    J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = red_.link_[COM_id].Jac;

                    wc_.set_contact(red_, 1, 1);
                    torque_grav = wc_.gravity_compensation_torque(red_);

                    red_.link_[Pelvis].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Eigen::Matrix3d::Identity(), false);

                    f_star.segment(0, 6) = wc_.getfstar6d(red_, COM_id);
                }
                else if (red_.ee_[0].contact)
                {
                    if (walking_init)
                    {
                        red_.link_[Right_Foot].x_init = red_.link_[Right_Foot].xpos;
                        walking_init = false;
                    }
                    task_number = 12;

                    wc_.set_contact(red_, 1, 0);
                    J_task.setZero(task_number, total_dof_ + 6);
                    f_star.setZero(task_number);

                    J_task.block(0, 0, 6, total_dof_ + 6) = red_.link_[COM_id].Jac;
                    J_task.block(6, 0, 6, total_dof_ + 6) = red_.link_[Right_Foot].Jac;

                    torque_grav = wc_.gravity_compensation_torque(red_);

                    red_.link_[Pelvis].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Eigen::Matrix3d::Identity(), false);

                    Vector3d lf_desired;
                    lf_desired = red_.link_[Right_Foot].x_init;
                    lf_desired(1) = -footstep_y_length;
                    lf_desired(2) = lf_desired(2) + 0.04;

                    red_.link_[Right_Foot].Set_Trajectory_from_quintic(control_time_, tc.command_time + (double)loop_ * time_segment + lr_st, tc.command_time + (double)loop_ * time_segment + lr_mt, lf_desired);

                    Vector3d lf_init = lf_desired;

                    lf_desired(2) = lf_desired(2) - 0.04;
                    if (loop_time > lr_mt)
                        red_.link_[Right_Foot].Set_Trajectory_from_quintic(control_time_, tc.command_time + (double)loop_ * time_segment + lr_mt, tc.command_time + (double)loop_ * time_segment + lr_et, lf_init, lf_desired);

                    Eigen::Vector3d quintic = DyrosMath::QuinticSpline(control_time_, tc.command_time + (double)loop_ * time_segment + lr_st, tc.command_time + (double)loop_ * time_segment + lr_et - 0.04, red_.link_[Right_Foot].x_init(0), 0, 0, red_.link_[Left_Foot].xpos(0) + step_length, 0, 0);
                    red_.link_[Right_Foot].x_traj(0) = quintic(0);
                    red_.link_[Right_Foot].v_traj(0) = quintic(1);

                    red_.link_[Right_Foot].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Eigen::Matrix3d::Identity(), false);

                    f_star.segment(0, 6) = wc_.getfstar6d(red_, COM_id);
                    f_star.segment(6, 6) = wc_.getfstar6d(red_, Right_Foot);
                }
                else if (red_.ee_[1].contact) // rightfoot contact
                {

                    Vector3d lf_desired;
                    if (walking_init)
                    {
                        red_.link_[Left_Foot].x_init = red_.link_[Left_Foot].xpos;
                        walking_init = false;
                    }
                    task_number = 12;
                    wc_.set_contact(red_, 0, 1);

                    J_task.setZero(task_number, total_dof_ + 6);
                    f_star.setZero(task_number);

                    J_task.block(0, 0, 6, total_dof_ + 6) = red_.link_[COM_id].Jac;
                    J_task.block(6, 0, 6, total_dof_ + 6) = red_.link_[Left_Foot].Jac;

                    torque_grav = wc_.gravity_compensation_torque(red_);

                    red_.link_[Pelvis].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Eigen::Matrix3d::Identity(), false);
                    //model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);

                    lf_desired = red_.link_[Left_Foot].x_init;

                    lf_desired(1) = footstep_y_length;
                    lf_desired(2) = lf_desired(2) + 0.04;

                    red_.link_[Left_Foot].Set_Trajectory_from_quintic(control_time_, tc.command_time + (double)loop_ * time_segment + lr_st, tc.command_time + (double)loop_ * time_segment + lr_mt, lf_desired);

                    Vector3d lf_init = lf_desired;

                    lf_desired(2) = lf_desired(2) - 0.04;
                    if (loop_time > lr_mt)
                        red_.link_[Left_Foot].Set_Trajectory_from_quintic(control_time_, tc.command_time + (double)loop_ * time_segment + lr_mt, tc.command_time + (double)loop_ * time_segment + lr_et, lf_init, lf_desired);

                    Eigen::Vector3d quintic = DyrosMath::QuinticSpline(control_time_, tc.command_time + (double)loop_ * time_segment + lr_st, tc.command_time + (double)loop_ * time_segment + lr_et - 0.04, red_.link_[Left_Foot].x_init(0), 0, 0, red_.link_[Right_Foot].xpos(0) + step_length, 0, 0);
                    red_.link_[Left_Foot].x_traj(0) = quintic(0);
                    red_.link_[Left_Foot].v_traj(0) = quintic(1);

                    red_.link_[Left_Foot].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Eigen::Matrix3d::Identity(), false);

                    f_star.segment(0, 6) = wc_.getfstar6d(red_, COM_id);
                    f_star.segment(6, 6) = wc_.getfstar6d(red_, Left_Foot);
                }

                Vector2d cp_current = red_.com_.CP;
                double w_ = sqrt(9.81 / red_.com_.pos(2));
                double b_ = exp(w_ * (time_segment - loop_time));

                Vector2d desired_cp;
                Vector2d right_cp, left_cp, cp_mod;
                //cp_mod << -0.02, -0.00747;
                //right_cp << -0.04, -0.095;
                //left_cp << -0.04, 0.095;
                cp_mod << -0.0, -0.00747;
                right_cp << 0.02, -0.09;
                left_cp << 0.02, 0.09;

                if (loop_ - loop_temp)
                {
                    loop_cnged = true;
                    cgen_init = true;
                }
                double st_temp;

                if (loop_ % 2)
                {
                    st_temp = step_length;
                    if (loop_ == 1)
                        st_temp = step_length / 2.0;

                    desired_cp = right_cp;
                    desired_cp(0) = right_cp(0) + ((double)loop_) * st_temp + red_.link_[COM_id].x_init(0);
                }
                else
                {
                    st_temp = step_length;
                    if (loop_ == 1)
                        st_temp = step_length / 2.0;
                    desired_cp = left_cp;
                    desired_cp(0) = left_cp(0) + ((double)loop_) * st_temp + red_.link_[COM_id].x_init(0);
                }

                Vector2d zmp = 1 / (1 - b_) * desired_cp - b_ / (1 - b_) * red_.com_.CP;

                if (cgen_init)
                {
                    tc.command_time = tc.command_time + time_segment - time_segment_origin;
                    time_segment = time_segment_origin;
                    std::cout << "###############################################" << std::endl;
                    std::cout << "loop : " << loop_ << " loop time : " << loop_time << std::endl;
                    //cx_init = model_.com_.pos.segment(0, 2);
                    //cv_init = model_.com_.vel.segment(0, 2);
                    std::cout << "desired cp   x : " << desired_cp(0) << "  y : " << desired_cp(1) << std::endl;
                    std::cout << "zmp gen   x : " << zmp(0) << "  y : " << zmp(1) << std::endl;
                    zmperror_reset = true;
                    //std::cout << "c CP" << std::endl;
                    //std::cout << model_.com_.CP << std::endl;

                    //zmp = 1 / (1 - b_) * desired_cp - b_ / (1 - b_) * model_.com_.CP;
                    //cgen_init = false;
                }

                /*
                if (zmp(1) > 0.11)
                {
                    zmp(1) = 0.11;
                }
                if (zmp(1) < -0.11)
                {
                    zmp(1) = -0.11;
                }*/

                double y_margin, x_margin;
                double over_ratio;
                y_margin = 0.04;
                x_margin = 0.11;

                if (loop_ > 0)
                {
                    if (loop_ % 2)
                    {
                        if (zmp(1) > (red_.link_[Left_Foot].xpos_contact(1) + y_margin))
                        {
                            //std::cout << "ZMP regulate active : " << red_.link_[Left_Foot].xpos_contact(1) << "\t zmp y : " << zmp(1) << std::endl;
                            zmp(1) = red_.link_[Left_Foot].xpos_contact(1) + y_margin;

                            //std::cout << loop_ << "Left ZMP regulate active : " << red_.link_[Left_Foot].xpos_contact(1) << "\t zmp y : " << zmp(1) << std::endl;
                        }
                        if (zmp(1) < (red_.link_[Left_Foot].xpos_contact(1) - y_margin))
                        {
                            //std::cout << "ZMP regulate active : " << red_.link_[Left_Foot].xpos_contact(1) << "\t zmp y : " << zmp(1) << std::endl;
                            zmp(1) = red_.link_[Left_Foot].xpos_contact(1) - y_margin;
                            //std::cout << loop_ << "Left ZMP regulate active : " << red_.link_[Left_Foot].xpos_contact(1) << "\t zmp y : " << zmp(1) << std::endl;
                        }

                        if (zmp(0) < (red_.link_[Left_Foot].xpos_contact(0) - x_margin))
                        {
                            zmp(0) = (red_.link_[Left_Foot].xpos_contact(0) - x_margin);
                        }
                        if (zmp(0) > (red_.link_[Left_Foot].xpos_contact(0) + x_margin))
                        {
                            zmp(0) = (red_.link_[Left_Foot].xpos_contact(0) + x_margin);
                        }
                    }
                    else
                    {
                        if (zmp(1) > (red_.link_[Right_Foot].xpos_contact(1) + y_margin))
                        {
                            zmp(1) = red_.link_[Right_Foot].xpos_contact(1) + y_margin;
                            //std::cout << loop_ << "Right ZMP regulate active : " << red_.link_[Left_Foot].xpos_contact(1) << "\t zmp y : " << zmp(1) << std::endl;
                        }
                        if (zmp(1) < (red_.link_[Right_Foot].xpos_contact(1) - y_margin))
                        {
                            zmp(1) = red_.link_[Right_Foot].xpos_contact(1) - y_margin;
                            //std::cout << loop_ << "Right ZMP regulate active : " << red_.link_[Left_Foot].xpos_contact(1) << "\t zmp y : " << zmp(1) << std::endl;
                        }

                        if (zmp(0) < (red_.link_[Right_Foot].xpos_contact(0) - x_margin))
                        {
                            zmp(0) = (red_.link_[Right_Foot].xpos_contact(0) - x_margin);
                        }
                        if (zmp(0) > (red_.link_[Right_Foot].xpos_contact(0) + x_margin))
                        {
                            zmp(0) = (red_.link_[Right_Foot].xpos_contact(0) + x_margin);
                        }
                    }
                }

                double extend_t = 1 / w_ * log((desired_cp(1) - zmp(1)) / (cp_current(1) - zmp(1))) - time_segment + loop_time;
                if (extend_t > 0.0001)
                {
                    std::cout << extend_t << std::endl;
                    time_segment = time_segment + extend_t;
                }
                // est_cp_ = b_ * (model_.com_.pos.segment(0, 2) + model_.com_.vel.segment(0, 2) / w_) + (1 - b_) * zmp;
                // std::cout << "estimated_cp" << std::endl;
                // std::cout << est_cp_ << std::endl;
                if (cgen_init)
                {
                    std::cout << "1 ZMP : " << zmp(1) << std::endl;
                    cgen_init = false;
                }

                red_.ZMP_desired(0) = zmp(0);
                red_.ZMP_desired(1) = zmp(1);

                wc_.set_zmp_control(red_, zmp, 1.0);
                //wc_.set_zmp_feedback_control(red_, zmperror_reset);
                //MatrixXd damping_matrix;
                //damping_matrix.setIdentity(total_dof_, total_dof_);

                // //torque_dc_.segment(0, 12) = q_dot_.segment(0, 12) * 2.0;
                // std::cout << "zmp_des" << std::endl;
                // std::cout << zmp << std::endl;
                //std::cout << "zmp_cur" << std::endl;
                //std::cout << model_.com_.ZMP << std::endl;
                //std::cout << "Sensor ZMP" << std::endl;
                //std::cout << body_zmp_ << std::endl;

                //single support test at tc_
                // single support , 0.1 ~ 0.9s foot up -> down just for 4cm?
                // at loop_ = 0( first phase)
                // at loop_ = 1, zmp at right foot.
                // at each loop, 0~0.1 double support 0.1~ 0.9 singlesupport 0.9~1.0 double support
                //

                red_.ZMP_ft = wc_.GetZMPpos_fromFT(red_);
                torque_task = wc_.task_control_torque(red_, J_task, f_star);
            }
            else if (tc.mode == 7)
            {
                static int loop_temp;
                static int loop_;
                static bool cgen_init;
                static bool loop_cnged;
                static bool walking_init;
                static double foot_height;

                static bool zmperror_reset;

                //QP_switch = true;
                red_.ee_[0].contact = true;
                red_.ee_[1].contact = true;

                //right_foot_contact_ = true;
                //left_foot_contact_ = true;
                double time_segment_origin = tc.traj_time;
                static double time_segment;
                static double loop_start_time;
                static double loop_end_time;

                if (tc.task_init)
                {
                    cgen_init = true;
                    loop_cnged = false;
                    walking_init = true;
                    foot_height = (red_.link_[Right_Foot].xpos(2) + red_.link_[Left_Foot].xpos(2)) / 2.0;
                    zmperror_reset = true;
                    time_segment = tc.traj_time;
                    loop_start_time = 0.0;
                    loop_end_time = 0.0;
                    tc.task_init = false;
                }

                double step_length = tc.ratio;
                double task_time = control_time_ - tc.command_time;

                loop_temp = loop_;
                loop_ = (int)(task_time / time_segment);
                double loop_time = task_time - (double)loop_ * time_segment;

                double lr_st, lr_mt, lr_et;
                lr_st = time_segment / 8.0;
                lr_mt = time_segment / 8.0 * 4.0;
                lr_et = time_segment / 8.0 * 7.0;

                if ((double)loop_ > 0.1)
                {
                    if (loop_ % 2)
                    {
                        if ((loop_time < lr_et))
                        {

                            red_.ee_[1].contact = false;
                            red_.ee_[0].contact = true;
                        }
                        else
                        {
                            red_.ee_[1].contact = true;
                            red_.ee_[0].contact = true;
                        }
                    }
                    else
                    {
                        if ((loop_time < lr_et))
                        {
                            red_.ee_[1].contact = true;
                            red_.ee_[0].contact = false;
                        }
                        else
                        {
                            red_.ee_[1].contact = true;
                            red_.ee_[0].contact = true;
                        }
                    }
                }

                //(model_.link_[model_.COM_id].x_init - zmp)*cosh(loop_time/time_segment)+time_segment * model_.link_[model_.COM_id]
                task_desired.setZero();
                task_desired(0) = red_.link_[COM_id].x_init(0);
                task_desired(2) = tc.height + foot_height;

                red_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time, task_desired);
                red_.link_[Upper_Body].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Matrix3d::Identity(), false);

                red_.ZMP_error = red_.ZMP_desired - red_.ZMP_ft;

                // model_.link_[model_.COM_id].x_traj.segment(0, 2) = (cx_init - zmp) * cosh(loop_time * w_) + cv_init * sinh(loop_time * w_) / w_ + zmp;

                // model_.link_[model_.COM_id].v_traj.segment(0, 2) = (cx_init - zmp) * w_ * sinh(loop_time * w_) + cv_init * cosh(loop_time * w_);

                // std::cout << " xtraj : " << std::endl;
                //std::cout << model_.link_[model_.COM_id].x_traj.segment(0, 2) << std::endl;
                //std::cout << "vtrah : " << std::endl;
                //std::cout << model_.link_[model_.COM_id].v_traj.segment(0, 2) << std::endl;

                if (red_.ee_[1].contact && red_.ee_[0].contact)
                {
                    walking_init = true;
                    task_number = 9;
                    J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                    f_star.setZero(task_number);
                    J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = red_.link_[COM_id].Jac;
                    J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = red_.link_[Upper_Body].Jac_COM_r;

                    wc_.set_contact(red_, 1, 1);
                    torque_grav = wc_.gravity_compensation_torque(red_);

                    red_.link_[Pelvis].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Eigen::Matrix3d::Identity(), false);

                    f_star.segment(0, 6) = wc_.getfstar6d(red_, COM_id);
                    f_star.segment(6, 3) = wc_.getfstar_rot(red_, Upper_Body);
                }
                else if (red_.ee_[0].contact)
                {
                    if (walking_init)
                    {
                        red_.link_[Right_Foot].x_init = red_.link_[Right_Foot].xpos;
                        walking_init = false;
                    }
                    task_number = 15;

                    wc_.set_contact(red_, 1, 0);
                    J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                    f_star.setZero(task_number);

                    J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = red_.link_[COM_id].Jac;
                    J_task.block(6, 0, 6, MODEL_DOF_VIRTUAL) = red_.link_[Right_Foot].Jac;
                    J_task.block(12, 0, 3, MODEL_DOF_VIRTUAL) = red_.link_[Upper_Body].Jac_COM_r;

                    torque_grav = wc_.gravity_compensation_torque(red_);

                    red_.link_[Pelvis].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Eigen::Matrix3d::Identity(), false);

                    Vector3d lf_desired;
                    lf_desired = red_.link_[Right_Foot].x_init;
                    lf_desired(1) = -0.1024;
                    lf_desired(2) = lf_desired(2) + 0.04;

                    red_.link_[Right_Foot].Set_Trajectory_from_quintic(control_time_, tc.command_time + (double)loop_ * time_segment + lr_st, tc.command_time + (double)loop_ * time_segment + lr_mt, lf_desired);

                    Vector3d lf_init = lf_desired;

                    lf_desired(2) = lf_desired(2) - 0.04;
                    if (loop_time > lr_mt)
                        red_.link_[Right_Foot].Set_Trajectory_from_quintic(control_time_, tc.command_time + (double)loop_ * time_segment + lr_mt, tc.command_time + (double)loop_ * time_segment + lr_et, lf_init, lf_desired);

                    Eigen::Vector3d quintic = DyrosMath::QuinticSpline(control_time_, tc.command_time + (double)loop_ * time_segment + lr_st, tc.command_time + (double)loop_ * time_segment + lr_et - 0.04, red_.link_[Right_Foot].x_init(0), 0, 0, red_.link_[Left_Foot].xpos(0) + step_length, 0, 0);
                    red_.link_[Right_Foot].x_traj(0) = quintic(0);
                    red_.link_[Right_Foot].v_traj(0) = quintic(1);

                    red_.link_[Right_Foot].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Eigen::Matrix3d::Identity(), false);

                    f_star.segment(0, 6) = wc_.getfstar6d(red_, COM_id);
                    f_star.segment(6, 6) = wc_.getfstar6d(red_, Right_Foot);
                    f_star.segment(12, 3) = wc_.getfstar_rot(red_, Upper_Body);
                }
                else if (red_.ee_[1].contact) // rightfoot contact
                {

                    Vector3d lf_desired;
                    if (walking_init)
                    {
                        red_.link_[Left_Foot].x_init = red_.link_[Left_Foot].xpos;
                        walking_init = false;
                    }
                    task_number = 15;
                    wc_.set_contact(red_, 0, 1);

                    J_task.setZero(task_number, total_dof_ + 6);
                    f_star.setZero(task_number);

                    J_task.block(0, 0, 6, total_dof_ + 6) = red_.link_[COM_id].Jac;
                    J_task.block(6, 0, 6, total_dof_ + 6) = red_.link_[Left_Foot].Jac;
                    J_task.block(12, 0, 3, MODEL_DOF_VIRTUAL) = red_.link_[Upper_Body].Jac_COM_r;

                    torque_grav = wc_.gravity_compensation_torque(red_);

                    red_.link_[Pelvis].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Eigen::Matrix3d::Identity(), false);
                    //model_.Link_Set_Trajectory_rotation(model_.Upper_Body, control_time_, tc_.taskcommand_.command_time, tc_.taskcommand_.command_time + tc_.taskcommand_.traj_time, Eigen::Matrix3d::Identity(), false);

                    lf_desired = red_.link_[Left_Foot].x_init;

                    lf_desired(1) = 0.1024;
                    lf_desired(2) = lf_desired(2) + 0.04;

                    red_.link_[Left_Foot].Set_Trajectory_from_quintic(control_time_, tc.command_time + (double)loop_ * time_segment + lr_st, tc.command_time + (double)loop_ * time_segment + lr_mt, lf_desired);

                    Vector3d lf_init = lf_desired;

                    lf_desired(2) = lf_desired(2) - 0.04;
                    if (loop_time > lr_mt)
                        red_.link_[Left_Foot].Set_Trajectory_from_quintic(control_time_, tc.command_time + (double)loop_ * time_segment + lr_mt, tc.command_time + (double)loop_ * time_segment + lr_et, lf_init, lf_desired);

                    Eigen::Vector3d quintic = DyrosMath::QuinticSpline(control_time_, tc.command_time + (double)loop_ * time_segment + lr_st, tc.command_time + (double)loop_ * time_segment + lr_et - 0.04, red_.link_[Left_Foot].x_init(0), 0, 0, red_.link_[Right_Foot].xpos(0) + step_length, 0, 0);
                    red_.link_[Left_Foot].x_traj(0) = quintic(0);
                    red_.link_[Left_Foot].v_traj(0) = quintic(1);

                    red_.link_[Left_Foot].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, Eigen::Matrix3d::Identity(), false);

                    f_star.segment(0, 6) = wc_.getfstar6d(red_, COM_id);
                    f_star.segment(6, 6) = wc_.getfstar6d(red_, Left_Foot);
                    f_star.segment(12, 3) = wc_.getfstar_rot(red_, Upper_Body);
                }

                Vector2d cp_current = red_.com_.CP;
                double w_ = sqrt(9.81 / red_.com_.pos(2));
                double b_ = exp(w_ * (time_segment - loop_time));

                Vector2d desired_cp;
                Vector2d right_cp, left_cp, cp_mod;
                //cp_mod << -0.02, -0.00747;
                //right_cp << -0.04, -0.095;
                //left_cp << -0.04, 0.095;
                cp_mod << -0.0, -0.00747;
                right_cp << 0.0, -0.09;
                left_cp << 0.0, 0.09;

                if (loop_ - loop_temp)
                {
                    loop_cnged = true;
                    cgen_init = true;
                }
                double st_temp;

                if (loop_ % 2)
                {
                    st_temp = step_length;
                    if (loop_ == 1)
                        st_temp = step_length / 2.0;

                    desired_cp = right_cp;
                    desired_cp(0) = right_cp(0) + ((double)loop_) * st_temp + red_.link_[COM_id].x_init(0);
                }
                else
                {
                    st_temp = step_length;
                    if (loop_ == 1)
                        st_temp = step_length / 2.0;
                    desired_cp = left_cp;
                    desired_cp(0) = left_cp(0) + ((double)loop_) * st_temp + red_.link_[COM_id].x_init(0);
                }

                Vector2d zmp = 1 / (1 - b_) * desired_cp - b_ / (1 - b_) * red_.com_.CP;

                if (cgen_init)
                {
                    tc.command_time = tc.command_time + time_segment - time_segment_origin;
                    time_segment = time_segment_origin;
                    std::cout << "###############################################" << std::endl;
                    std::cout << "loop : " << loop_ << " loop time : " << loop_time << std::endl;
                    //cx_init = model_.com_.pos.segment(0, 2);
                    //cv_init = model_.com_.vel.segment(0, 2);
                    std::cout << "desired cp   x : " << desired_cp(0) << "  y : " << desired_cp(1) << std::endl;
                    std::cout << "zmp gen   x : " << zmp(0) << "  y : " << zmp(1) << std::endl;
                    zmperror_reset = true;
                    //std::cout << "c CP" << std::endl;
                    //std::cout << model_.com_.CP << std::endl;

                    //zmp = 1 / (1 - b_) * desired_cp - b_ / (1 - b_) * model_.com_.CP;
                    //cgen_init = false;
                }

                /*
                if (zmp(1) > 0.11)
                {
                    zmp(1) = 0.11;
                }
                if (zmp(1) < -0.11)
                {
                    zmp(1) = -0.11;
                }*/

                double y_margin, x_margin;
                double over_ratio;
                y_margin = 0.04;
                x_margin = 0.11;

                if (loop_ > 0)
                {
                    if (loop_ % 2)
                    {
                        if (zmp(1) > (red_.link_[Left_Foot].xpos_contact(1) + y_margin))
                        {
                            //std::cout << "ZMP regulate active : " << red_.link_[Left_Foot].xpos_contact(1) << "\t zmp y : " << zmp(1) << std::endl;
                            zmp(1) = red_.link_[Left_Foot].xpos_contact(1) + y_margin;

                            //std::cout << loop_ << "Left ZMP regulate active : " << red_.link_[Left_Foot].xpos_contact(1) << "\t zmp y : " << zmp(1) << std::endl;
                        }
                        if (zmp(1) < (red_.link_[Left_Foot].xpos_contact(1) - y_margin))
                        {
                            //std::cout << "ZMP regulate active : " << red_.link_[Left_Foot].xpos_contact(1) << "\t zmp y : " << zmp(1) << std::endl;
                            zmp(1) = red_.link_[Left_Foot].xpos_contact(1) - y_margin;
                            //std::cout << loop_ << "Left ZMP regulate active : " << red_.link_[Left_Foot].xpos_contact(1) << "\t zmp y : " << zmp(1) << std::endl;
                        }

                        if (zmp(0) < (red_.link_[Left_Foot].xpos_contact(0) - x_margin))
                        {
                            zmp(0) = (red_.link_[Left_Foot].xpos_contact(0) - x_margin);
                        }
                        if (zmp(0) > (red_.link_[Left_Foot].xpos_contact(0) + x_margin))
                        {
                            zmp(0) = (red_.link_[Left_Foot].xpos_contact(0) + x_margin);
                        }
                    }
                    else
                    {
                        if (zmp(1) > (red_.link_[Right_Foot].xpos_contact(1) + y_margin))
                        {
                            zmp(1) = red_.link_[Right_Foot].xpos_contact(1) + y_margin;
                            //std::cout << loop_ << "Right ZMP regulate active : " << red_.link_[Left_Foot].xpos_contact(1) << "\t zmp y : " << zmp(1) << std::endl;
                        }
                        if (zmp(1) < (red_.link_[Right_Foot].xpos_contact(1) - y_margin))
                        {
                            zmp(1) = red_.link_[Right_Foot].xpos_contact(1) - y_margin;
                            //std::cout << loop_ << "Right ZMP regulate active : " << red_.link_[Left_Foot].xpos_contact(1) << "\t zmp y : " << zmp(1) << std::endl;
                        }

                        if (zmp(0) < (red_.link_[Right_Foot].xpos_contact(0) - x_margin))
                        {
                            zmp(0) = (red_.link_[Right_Foot].xpos_contact(0) - x_margin);
                        }
                        if (zmp(0) > (red_.link_[Right_Foot].xpos_contact(0) + x_margin))
                        {
                            zmp(0) = (red_.link_[Right_Foot].xpos_contact(0) + x_margin);
                        }
                    }
                }

                double extend_t = 1 / w_ * log((desired_cp(1) - zmp(1)) / (cp_current(1) - zmp(1))) - time_segment + loop_time;
                //if (extend_t > 0.0001)
                //{
                //    std::cout << extend_t << std::endl;
                //    time_segment = time_segment + extend_t;
                //}
                // est_cp_ = b_ * (model_.com_.pos.segment(0, 2) + model_.com_.vel.segment(0, 2) / w_) + (1 - b_) * zmp;
                // std::cout << "estimated_cp" << std::endl;
                // std::cout << est_cp_ << std::endl;
                if (cgen_init)
                {
                    std::cout << "1 ZMP : " << zmp(1) << std::endl;
                    cgen_init = false;
                }

                //wc_.set_zmp_control(red_, zmp, 1.0);
                wc_.set_zmp_feedback_control(red_, zmp, zmperror_reset);
                //MatrixXd damping_matrix;
                //damping_matrix.setIdentity(total_dof_, total_dof_);

                // //torque_dc_.segment(0, 12) = q_dot_.segment(0, 12) * 2.0;
                // std::cout << "zmp_des" << std::endl;
                // std::cout << zmp << std::endl;
                //std::cout << "zmp_cur" << std::endl;
                //std::cout << model_.com_.ZMP << std::endl;
                //std::cout << "Sensor ZMP" << std::endl;
                //std::cout << body_zmp_ << std::endl;

                //single support test at tc_
                // single support , 0.1 ~ 0.9s foot up -> down just for 4cm?
                // at loop_ = 0( first phase)
                // at loop_ = 1, zmp at right foot.
                // at each loop, 0~0.1 double support 0.1~ 0.9 singlesupport 0.9~1.0 double support
                //
                torque_task = wc_.task_control_torque(red_, J_task, f_star);
            }
            else if (tc.mode == 7)
            {
                //left is plus !
                static int phase = 0; //first phase -> left foot contact!
                static bool phase_init = true;
                static bool phase_end = false;

                red_.ee_[LEFT].contact = true;
                red_.ee_[RIGHT].contact = true;

                double step_legth = 0.0;
                double step_time = tc.traj_time;

                int last_phase = 10;

                static double phase_start_time;
                static double phase_end_time;

                static Vector2d cp_desired;
                Vector2d cp_current = red_.com_.CP;

                if (phase_init)
                {
                    phase_start_time = control_time_;
                    phase_end_time = control_time_ + step_time;

                    //init all link in init
                    red_.link_[Left_Foot].Set_initpos();
                    red_.link_[Right_Foot].Set_initpos();
                    red_.link_[COM_id].Set_initpos();

                    phase_init = false;
                }

                //double support phase : start&end
                //single support phase : left&right foot swing.

                if (phase == 0)
                {
                }
                else if (phase == last_phase)
                {
                }
                else
                {
                }

                if (control_time_ >= phase_end_time) //phase ending condition!
                {
                    phase_end = true;
                }

                if (phase_end) //phase end!
                {
                    phase_end = false;
                    phase_init = true;
                }
            }
            else if (tc.mode == 8)
            {
                //Left hand contact
                wc_.set_contact(red_, 1, 1, 1, 0);

                red_.task_force_control = false;

                torque_grav = wc_.gravity_compensation_torque(red_, dc.fixedgravity);
                //torque_grav = wc_.gravity_compensation_torque(dc.fixedgravity);
                task_number = 9;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task.block(0, 0, 3, MODEL_DOF_VIRTUAL) = red_.link_[COM_id].Jac.block(0, 0, 3, MODEL_DOF_VIRTUAL);
                J_task.block(3, 0, 6, MODEL_DOF_VIRTUAL) = red_.link_[Right_Hand].Jac;

                red_.link_[COM_id].x_desired = red_.link_[COM_id].x_init;
                red_.link_[COM_id].x_desired(0) += 0.2;
                red_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);

                red_.link_[Right_Hand].x_desired = red_.link_[Right_Hand].x_init;
                red_.link_[Right_Hand].x_desired(0) += 0.9;
                red_.link_[Right_Hand].x_desired(2) += 0.2;
                red_.link_[Right_Hand].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);

                red_.link_[Right_Hand].rot_desired = DyrosMath::rotateWithX(3.141592) * DyrosMath::rotateWithY(0) * DyrosMath::rotateWithZ(3.141592 / 2.0);

                red_.link_[Right_Hand].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                f_star.segment(0, 3) = wc_.getfstar_tra(red_, COM_id);
                f_star.segment(3, 6) = wc_.getfstar6d(red_, Right_Hand);

                torque_task = wc_.task_control_torque(red_, J_task, f_star);

                //red_.link_[Pelvis].rot_desired = Matrix3d::Identity();

                //f_star = wc_.getfstar6d(red_, COM_id);
                //torque_task = wc_.task_control_torque(red_, J_task, f_star);
                //torque_task = wc_.task_control_torque(J_task, f_star);

                if (1) //control_time_ > tc.command_time + tc.traj_time)
                {
                    std::cout << "##########" << std::endl
                              << "task time : \t" << control_time_ - tc.command_time << std::endl
                              << "dis pos x : \t" << red_.com_.pos(0) - red_.link_[Right_Foot].xpos_contact(0) << std::endl
                              << "lHand f z : \t" << red_.LH_FT(2) - 9.81 << std::endl
                              << std::endl;
                }

                cr_mode = 1;
            }
            else if (tc.mode == 9)
            {
                //Both hand
                wc_.set_contact(red_, 1, 1, 1, 1);

                torque_grav = wc_.gravity_compensation_torque(red_, dc.fixedgravity);
                //torque_grav = wc_.gravity_compensation_torque(dc.fixedgravity);
                task_number = 6;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task = red_.link_[COM_id].Jac;

                red_.link_[COM_id].x_desired = tc.ratio * red_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * red_.link_[Right_Foot].xpos;
                red_.link_[COM_id].x_desired(2) = tc.height + tc.ratio * red_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * red_.link_[Right_Foot].xpos(2);
                red_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);

                //red_.link_[Pelvis].rot_desired = Matrix3d::Identity();

                //f_star = wc_.getfstar6d(red_, COM_id);
                //torque_task = wc_.task_control_torque(red_, J_task, f_star);
                //torque_task = wc_.task_control_torque(J_task, f_star);

                cr_mode = 1;
            }
            else if (tc.mode == 10)   // ?????? ?????? - ?????? ?????? - ?????? ?????????
            {
                static bool reinit_lh = true;
                if (tc.task_init)
                {
                    reinit_lh = true;
                    tc.task_init = false;
                }

                wc_.set_contact(red_, 1, 1); 
                torque_grav = wc_.gravity_compensation_torque(red_, dc.fixedgravity);

                task_number = 12;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = red_.link_[Upper_Body].Jac_COM;
                //J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = red_.link_[Upper_Body].Jac_COM_r;
                J_task.block(6, 0, 6, MODEL_DOF_VIRTUAL) = red_.link_[Left_Hand].Jac_COM;

                red_.link_[Upper_Body].x_desired = tc.ratio * red_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * red_.link_[Right_Foot].xpos;
                //red_.link_[COM_id].x_desired(0) += 0.1;

                red_.link_[Upper_Body].x_desired(2) = -0.05+tc.height + tc.ratio * red_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * red_.link_[Right_Foot].xpos(2);
                red_.link_[Upper_Body].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                red_.link_[Upper_Body].rot_desired = Matrix3d::Identity();
                red_.link_[Upper_Body].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                //red_.link_[Upper_Body].rot_desired = Matrix3d::Identity();
                //red_.link_[Upper_Body].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                red_.link_[Left_Hand].x_desired = red_.link_[Left_Hand].x_init;
                red_.link_[Left_Hand].x_desired(2) -= 0.05;
                red_.link_[Left_Hand].x_desired(1) += 0.22;
                red_.link_[Left_Hand].x_desired(0) -= 0.2;
                red_.link_[Left_Hand].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);

                red_.link_[Left_Hand].rot_desired = red_.link_[Left_Hand].rot_init*DyrosMath::rotateWithX(0) * DyrosMath::rotateWithY(3.141592 / 2.0) * DyrosMath::rotateWithZ(3.141592 / 2.0);  //?????????
                red_.link_[Left_Hand].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                /*if (control_time_ >= tc.command_time + tc.traj_time)
                {
                    if (reinit_lh)
                    {
                        red_.link_[Left_Hand].Set_initpos();
                        reinit_lh = false;
                    }
                    red_.link_[Left_Hand].x_desired = red_.link_[Left_Hand].x_init;
                    red_.link_[Left_Hand].x_desired(2) -= 0.06;
                    red_.link_[Left_Hand].Set_Trajectory_from_quintic(control_time_, tc.command_time + tc.traj_time, tc.command_time + 2 * tc.traj_time);

                    red_.link_[Left_Hand].rot_desired = DyrosMath::rotateWithX(-3.141592 / 2.0) * DyrosMath::rotateWithY(0) * DyrosMath::rotateWithZ(0);
                    red_.link_[Left_Hand].Set_Trajectory_rotation(control_time_, tc.command_time + tc.traj_time, tc.command_time + 2 * tc.traj_time, false);
                }

                

                if (control_time_ >= tc.command_time + 2 * tc.traj_time)
                {
                    //f_star.segment(6, 6).setZero();
                    MatrixXd task_slc;
                    task_slc.setIdentity(task_number, task_number);

                    VectorXd force_desired;
                    force_desired.setZero(task_number);
                    force_desired(8) = -15.0;
                    task_slc(8, 8) = 0.0;

                    wc_.set_force_control(red_, task_slc, force_desired);
                    //f_star(8) = -10.0;
                    //f_star(11) = 15.0;
                }*/

                //f_star.segment(0, 6) = wc_.getfstar6d(red_, COM_id);
                f_star.segment(0, 6) = wc_.getfstar6d(red_, Upper_Body);
                f_star.segment(6, 6) = wc_.getfstar6d(red_, Left_Hand);
                torque_task = wc_.task_control_torque(red_, J_task, f_star);

                if (control_time_ >= tc.command_time + 3 * tc.traj_time)
                {
                    tc.mode = 8;
                    tc.command_time = control_time_;
                    tc.task_init = true;
                    red_.link_[Right_Hand].Set_initpos();
                    red_.link_[COM_id].Set_initpos();
                }

                std::cout << "f_star.segment(0, 6) " << std::endl;
                std::cout << f_star.segment(0, 6) << std::endl
                          << std::endl;
                std::cout << "f_star.segment(6, 6) " << std::endl;
                std::cout << f_star.segment(6, 6) << std::endl
                          << std::endl;
                std::cout << "f_star.segment(0, 3) " << std::endl;
                std::cout << f_star.segment(0, 3) << std::endl
                          << std::endl;
            }
            else if (tc.mode == 11)
            {
                wc_.set_contact(red_, 1, 1);
                torque_grav = wc_.gravity_compensation_torque(red_, dc.fixedgravity);

                task_number = 3;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);                                 

                J_task.block(0, 0, 3, MODEL_DOF_VIRTUAL) = red_.link_[Left_Hand].Jac_COM_r;

                std::cout << "Jacobian matrix :::: " << std::endl;
                std::cout << red_.link_[Left_Hand].Jac_COM_r << std::endl;
                std::cout << "length of this matrix = " << (sizeof(red_.link_[Left_Hand].Jac_COM_r)/sizeof(red_.link_[Left_Hand].Jac_COM_r[0])) << std::endl;

                red_.link_[COM_id].x_desired = tc.ratio * red_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * red_.link_[Right_Foot].xpos;  
                //red_.link_[COM_id].x_desired(0) += 0.1;

                red_.link_[COM_id].x_desired(2) = tc.height + tc.ratio * red_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * red_.link_[Right_Foot].xpos(2);
                red_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                red_.link_[COM_id].rot_desired = Matrix3d::Identity();
                red_.link_[COM_id].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                red_.link_[Left_Hand].x_desired = red_.link_[Left_Hand].x_init;
                //red_.link_[Left_Hand].x_desired(0) += 0.04; 
                red_.link_[Left_Hand].x_desired(2) += 0.02; // -= 0.06;
                red_.link_[Left_Hand].x_desired(1) -= 0.05;  //original 0.05

                // Eigen::MatrixXd<double, 6, 8> j_right_hand = red_.link_[Right_Hand].Jac.block<6, 8>(0, 28)
                // j_right_hand.inverse() 

                red_.link_[Left_Hand].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);

                red_.link_[Left_Hand].rot_desired = DyrosMath::rotateWithX(-3.141592 / 2.0) * DyrosMath::rotateWithY(0) * DyrosMath::rotateWithZ(0);
                red_.link_[Left_Hand].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                std::cout << "rotation information :::: " << std::endl;
                std::cout << red_.link_[Left_Hand].rot_desired.eulerAngles(0, 1, 2) << std::endl
                          << std::endl;
                std::cout << red_.link_[Left_Hand].Rotm.eulerAngles(0, 1, 2) << std::endl;
        
                f_star.segment(0, 3) = wc_.getfstar_rot(red_, Left_Hand);

                
                std::cout << f_star.segment(0, 3) << std::endl
                          << std::endl;

                torque_task = wc_.task_control_torque(red_, J_task, f_star);

                std::cout << "lambda : " << std::endl
                          << red_.lambda << std::endl;
                std::cout << "torque_task : " << std::endl
                          << torque_task << std::endl;
            }
            else if (tc.mode == 12)
            {
                wc_.set_contact(red_, 1, 1);
                //torque_grav = wc_.gravity_compensation_torque(red_, dc.fixedgravity);

                task_number = 9;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = red_.link_[COM_id].Jac;
                J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = red_.link_[Upper_Body].Jac_COM_r;

                red_.link_[COM_id].x_desired = tc.ratio * red_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * red_.link_[Right_Foot].xpos;
                //red_.link_[COM_id].x_desired(0) += 0.1;

                red_.link_[COM_id].x_desired(2) = tc.height + tc.ratio * red_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * red_.link_[Right_Foot].xpos(2);
                red_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                red_.link_[COM_id].rot_desired = Matrix3d::Identity();
                red_.link_[COM_id].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                red_.link_[Upper_Body].rot_desired = DyrosMath::rotateWithY(tc.angle) * Matrix3d::Identity();
                red_.link_[Upper_Body].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                //std::cout << "rotation information :::: " << std::endl;
                //std::cout << red_.link_[Upper_Body].rot_desired.eulerAngles(0, 1, 2) << std::endl
                //          << std::endl;
                //std::cout << red_.link_[Upper_Body].Rotm.eulerAngles(0, 1, 2) << std::endl;

                f_star.segment(0, 6) = wc_.getfstar6d(red_, COM_id);
                f_star.segment(6, 3) = wc_.getfstar_rot(red_, Upper_Body);
                //f_star.segment(6, 3) = wc_.getfstar6d(red_, Left_Hand);

                //std::cout << f_star.segment(6, 3) << std::endl
                //          << std::endl;
                torque_grav.setZero();
                torque_task = wc_.task_control_torque_QP2(red_, J_task, f_star);
                cr_mode = 2;
                Vector3d lrot_eulr, rrot_eulr;
                lrot_eulr = DyrosMath::rot2Euler(red_.link_[Left_Foot].Rotm);
                rrot_eulr = DyrosMath::rot2Euler(red_.link_[Right_Foot].Rotm);

                double lz_y, rz_y;
                if (control_time_ == tc.command_time)
                {
                    out << "t \t ce \t lz \t rz \t lee \t re \t lfz \t rfz \t ly \r ry" << std::endl;
                }
                if (control_time_ < tc.command_time + tc.traj_time)
                {
                    out << control_time_ << "\t" << red_.link_[COM_id].x_traj(1) - red_.link_[COM_id].xpos(1) << "\t" << red_.ContactForce(3) / red_.ContactForce(2) << "\t" << red_.ContactForce(9) / red_.ContactForce(8) << "\t" << lrot_eulr(0) << "\t" << rrot_eulr(0) << "\t" << red_.ContactForce(2) << "\t" << red_.ContactForce(8) << "\t" << red_.link_[Left_Foot].xpos(1) << "\t" << red_.link_[Right_Foot].xpos(1) << std::endl;
                }

                //std::cout << "cf: " << std::endl
                //          << wc_.get_contact_force(red_, torque_task + torque_grav);
            }
            else if (tc.mode == 13)  
            {   // ?????? 13??? ??????
                wc_.set_contact(red_, 1, 1);
                //torque_grav = wc_.gravity_compensation_torque(red_, dc.fixedgravity);

                task_number = 9 + 12;
                J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                f_star.setZero(task_number);

                J_task.block(0, 0, 6, MODEL_DOF_VIRTUAL) = red_.link_[COM_id].Jac;
                J_task.block(6, 0, 3, MODEL_DOF_VIRTUAL) = red_.link_[Upper_Body].Jac_COM_r;
                J_task.block(9, 0, 6, MODEL_DOF_VIRTUAL) = red_.link_[Left_Hand].Jac_COM;
                J_task.block(15, 0, 6, MODEL_DOF_VIRTUAL) = red_.link_[Right_Hand].Jac_COM;

                red_.link_[COM_id].x_desired = tc.ratio * red_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * red_.link_[Right_Foot].xpos;
                //red_.link_[COM_id].x_desired(0) += 0.1;

                red_.link_[COM_id].x_desired(2) = tc.height + tc.ratio * red_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * red_.link_[Right_Foot].xpos(2);
                red_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                red_.link_[COM_id].rot_desired = Matrix3d::Identity();
                red_.link_[COM_id].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                red_.link_[Upper_Body].rot_desired = DyrosMath::rotateWithY(tc.angle) * Matrix3d::Identity();
                red_.link_[Upper_Body].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                Vector3d ldi = red_.link_[Left_Hand].x_init - red_.link_[Pelvis].x_init;
                Vector3d rdi = red_.link_[Right_Hand].x_init - red_.link_[Pelvis].x_init;

                red_.link_[Left_Hand].rot_desired = Matrix3d::Identity();
                red_.link_[Left_Hand].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time, red_.link_[Pelvis].xpos + ldi, -red_.link_[Pelvis].v, red_.link_[Pelvis].xpos + ldi, -red_.link_[Pelvis].v);
                red_.link_[Left_Hand].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, true);

                red_.link_[Right_Hand].rot_desired = Matrix3d::Identity();
                red_.link_[Right_Hand].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time, red_.link_[Pelvis].xpos + rdi, -red_.link_[Pelvis].v, red_.link_[Pelvis].xpos + rdi, -red_.link_[Pelvis].v);
                red_.link_[Right_Hand].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, true);

                //std::cout << "rotation information :::: " << std::endl;
                //std::cout << red_.link_[Upper_Body].rot_desired.eulerAngles(0, 1, 2) << std::endl
                //          << std::endl;
                //std::cout << red_.link_[Upper_Body].Rotm.eulerAngles(0, 1, 2) << std::endl;

                f_star.segment(0, 6) = wc_.getfstar6d(red_, COM_id);
                f_star.segment(6, 3) = wc_.getfstar_rot(red_, Upper_Body);
                f_star.segment(9, 6) = wc_.getfstar6d(red_, Left_Hand);
                f_star.segment(15, 6) = wc_.getfstar6d(red_, Right_Hand);

                //f_star.segment(6, 3) = wc_.getfstar6d(red_, Left_Hand);

                //std::cout << f_star.segment(6, 3) << std::endl
                //          << std::endl;
                torque_grav.setZero();
                torque_task = wc_.task_control_torque_QP2(red_, J_task, f_star);
                cr_mode = 2;

                //std::cout << "cf: " << std::endl
                //          << wc_.get_contact_force(red_, torque_task + torque_grav);           
            }
            else if (tc.mode == 14)  //mode 11  ?????? ?????? ????????? ??????
            {
                wc_.set_contact(red_, 1, 1);
                torque_grav = wc_.gravity_compensation_torque(red_, dc.fixedgravity);

                task_number = 3;
                // J_task.setZero(task_number, MODEL_DOF_VIRTUAL);
                // f_star.setZero(task_number);                                 

                // J_task.block(0, 0, 3, MODEL_DOF_VIRTUAL) = red_.link_[Left_Hand].Jac_COM_r;

                // red_.link_[COM_id].x_desired = tc.ratio * red_.link_[Left_Foot].xpos + (1.0 - tc.ratio) * red_.link_[Right_Foot].xpos;  
                // //red_.link_[COM_id].x_desired(0) += 0.1;

                // red_.link_[COM_id].x_desired(2) = tc.height + tc.ratio * red_.link_[Left_Foot].xpos(2) + (1.0 - tc.ratio) * red_.link_[Right_Foot].xpos(2);
                // red_.link_[COM_id].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                // red_.link_[COM_id].rot_desired = Matrix3d::Identity();
                // red_.link_[COM_id].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);

                // red_.link_[Left_Hand].x_desired = red_.link_[Left_Hand].x_init;
                // //red_.link_[Left_Hand].x_desired(0) += 0.04; 
                // red_.link_[Left_Hand].x_desired(2) += 0.02; // -= 0.06;
                // red_.link_[Left_Hand].x_desired(1) -= 0.05;  //original 0.05

                // // Eigen::MatrixXd<double, 6, 8> j_right_hand = red_.link_[Right_Hand].Jac.block<6, 8>(0, 28)
                // // j_right_hand.inverse() 

                // red_.link_[Left_Hand].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                // red_.link_[Left_Hand].rot_desired = DyrosMath::rotateWithX(-3.141592 / 2.0) * DyrosMath::rotateWithY(0) * DyrosMath::rotateWithZ(0);
                // red_.link_[Left_Hand].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);
                        
                // f_star.segment(0, 3) = wc_.getfstar_rot(red_, Left_Hand);
                
                // torque_task = wc_.task_control_torque(red_, J_task, f_star);
                
            

                Eigen::Matrix3x1d x_1;
                x_1(0,0)= -211.717;
                x_1(1,0)= 28.6508;
                x_1(2,0)= -21.9734;
                Eigen::Matrix2x2d x_2;
                x_2(0,0)= 1;
                x_2(0,1)= 0;
                x_2(1,0)= 0;
                x_2(1,1)= 1;
                Eigen::Matrix4x3d x_3;
                x_3(0,0)=1;
                x_3(0,1)=0;
                x_3(0,2)=2;
                x_3(1,0)=3;
                x_3(1,1)=1;
                x_3(1,2)=4;
                x_3(2,0)=2;
                x_3(2,1)=3;
                x_3(2,2)=2;
                x_3(3,0)=1;
                x_3(3,1)=3;
                x_3(3,2)=0;


                std::cout << x_1 << std::endl << std::endl;
                std::cout << "x_2 inverse" << x_2.inverse() << std::endl << std::endl;
                std::cout << "x_3" << x_3 << std::endl << std::endl;
                std::cout << "x_3 inverse" << x_3.inverse() << std::endl << std::endl;
                J_lefthand.setZero(task_number, MODEL_DOF_VIRTUAL);
                J_lefthand.block(0,0,3, MODEL_DOF_VIRTUAL) = red_.link_[Left_Hand].Jac_COM_p;

                J_test.setZero(task_number, 8);
                J_test.block(0,0,3,8) = red_.link_[Left_Hand].Jac_COM_p.block(0,21,3,8);
                std::cout << J_lefthand << std::endl;
                std::cout << "J_test" << J_test << std::endl;
                J_test_inverse.setZero(8,task_number);
                //J_test_inverse.block(0,0,8,3) = J_test.inverse();
                //std::cout << "J_test_inverse" << J_test_inverse << std::endl;
                f_star.setZero(task_number);
                f_star.segment(0,3) = wc_.getfstar_rot(red_, Left_Hand);
                torque_task = wc_.task_control_torque(red_, J_lefthand, f_star);
            }       
            else if (tc.mode == 15)  //?????? ?????? ??????
            {
                ///////// Jacobian based ik arm controller (Daegyu)/////////////////
                ///////// TEST SOURCE ////////
                wc_.set_contact(red_, 1, 1);
                torque_grav = wc_.gravity_compensation_torque(red_, dc.fixedgravity);
                task_number = 6;
                J_task.setZero(task_number, 8);
                // J_task.block(0, 0, 6, 8) = tocabi_.link_[Left_Hand].Jac.block(0, 23, 6, 8); // set left arm jacobian as task jacobian
                J_task.block(0, 0, 6, 8) = red_.link_[Left_Hand].Jac.block(0,21,6,8);
                Eigen::Matrix<rScalar, 8, 6> J_task_inv;
                std::cout << "J_task: " << J_task << std::endl << std::endl;
                J_task_inv = DyrosMath::pinv_SVD(J_task); //Pseudo inverse
                std::cout << "J_task_inv: " << J_task_inv << std::endl << std::endl;


                red_.link_[Left_Hand].x_desired = red_.link_[Left_Hand].x_init;
                red_.link_[Left_Hand].x_desired(0) = red_.link_[Left_Hand].x_init(0) + 0.00;
                red_.link_[Left_Hand].x_desired(1) = red_.link_[Left_Hand].x_init(1) + 0.00;
                red_.link_[Left_Hand].x_desired(2) = red_.link_[Left_Hand].x_init(2) + 0.04; // set target x so as to elevate left arm 0.2m higher
                
                //red_.link_[Left_Hand].rot_desired = red_.link_[Left_Hand].rot_init;
                red_.link_[COM_id].rot_desired = Matrix3d::Identity();
                
                red_.link_[Left_Hand].Set_Trajectory_from_quintic(control_time_, tc.command_time, tc.command_time + tc.traj_time);
                red_.link_[Left_Hand].Set_Trajectory_rotation(control_time_, tc.command_time, tc.command_time + tc.traj_time, false);
                
                Eigen::Vector6d x_dot_desired;
                Eigen::Vector3d error_v;
                Eigen::Vector3d error_w;
                Eigen::Vector3d k_pos;
                Eigen::Vector3d k_rot;
                Eigen::VectorXd q_dot_desired_; // test
                Eigen::VectorXd q_desired_; //test
                Eigen::VectorXd q_desired_pre_; //test
                k_pos(0) = 10;
                k_pos(1) = 10;
                k_pos(2) = 10;
                k_rot(0) = 1;
                k_rot(1) = 1;
                k_rot(2) = 1;
                error_v = red_.link_[Left_Hand].x_traj -  red_.link_[Left_Hand].xpos;
                error_w = DyrosMath::getPhi(red_.link_[Left_Hand].Rotm, red_.link_[Left_Hand].r_traj);
                // for(int i = 0; i<3; i++)
                // {
                //     x_dot_desired(i) = tocabi_.link_[Left_Hand].v_traj(i) + tocabi_.link_[Left_Hand].pos_p_gain(i)*error_v(i); // linear velocity
                //     x_dot_desired(i+3) = tocabi_.link_[Left_Hand].w_traj(i) + tocabi_.link_[Left_Hand].rot_p_gain(i)*error_w(i);
                // }
                for(int i = 0; i<3; i++)
                {
                    x_dot_desired(i) = 0.0 + k_pos(i)*error_v(i); // linear velocity ,????????? 0?????? ??????
                    
                    //x_dot_desired(i) = red_.link_[Left_Hand].v_traj(i) + k_pos(i)*error_v(i); // linear velocity
                    
                    x_dot_desired(i+3) = 0.0;//tocabi_.link_[Left_Hand].w_traj(i) + k_rot(i)*error_w(i);
                }

                std::cout << x_dot_desired << std::endl;
                q_dot_desired_.setZero(8,1);
                q_dot_desired_.block(0,0,8,1) = J_task_inv*x_dot_desired;
                std::cout << "q_dot_desired_ = " << q_dot_desired_ << std::endl;

                q_desired_.setZero(8,1);
                q_desired_pre_.setZero(8,1);
                //q_desired_.block(0,0,8,1) = q_desired_pre_(0,0,8,1) + q_dot_desired_.block(0,0,8,1)/dc.dym_hz;
                //?????????

                Eigen::Vector3d q_1;
                q_1(0)= -211.717;
                q_1(1)= 28.6508;
                q_1(2)= -21.9734;
                
                // Eigen::Vector3d q_2;
                // q_2 = q_1 / 2.0;
                // std::cout << "q_2 = " << q_2 <<std::endl;

                // ???????????? ??????????????? ????????? ?????? ?????? ?????? ????????? ??????/////////////////////??????????????? ???????????? ???-------> q_desired_pre_ = (?????? 8??? ????????? ?????? ???);
                
                //std::cout << "q_dot_desired_ = " << q_dot_desired_.block(0,0,8,1) / 15.0 << std::endl;
                q_desired_.block(0,0,8,1) = q_dot_desired_.block(0,0,8,1) / dc.dym_hz_chankyo;  //?????? qdot ?????? 0?????? ??????
                //q_desired_pre_ = q_desired_;
                
                
                
                // //q_desired_.segment<8>(15) = q_.segment<8>(15) + q_dot_desired_.segment<8>(15)/dc.dym_hz;
                // q_desired_.segment<8>(15) = q_desired_pre_.segment<8>(15) + q_dot_desired_.segment<8>(15)/dc.dym_hz;
                // q_desired_pre_ = q_desired_;
                
                
                Eigen::MatrixXd kp(8,1);
                Eigen::MatrixXd kv(8,1);
                for(int i = 0; i<8; i++)
                {
                    kp(i) = 25;
                    kv(i) = 10;
                }
                torque_task.setZero(MODEL_DOF);
                for(int i = 0; i<8; i++)
                {
                    torque_task(i+15) = kp(i)*(q_desired_(i)) + kv(i)*(q_dot_desired_(i));
                    //torque_task(i+15) = kp(i)*(q_desired_(i) - q_(i+15)) + kv(i)*(q_dot_desired_(i+15) - q_dot_(i+15));
                }
                // ///////////Tracking////////////////
                // // fout << tocabi_.link_[Left_Hand].x_traj(0) << "\t"<< tocabi_.link_[Left_Hand].x_traj(1) << "\t"<< tocabi_.link_[Left_Hand].x_traj(2) << "\t"<< \
                // // tocabi_.link_[Left_Hand].xpos(0) << "\t"<< tocabi_.link_[Left_Hand].xpos(1) << "\t"<< tocabi_.link_[Left_Hand].xpos(2) << "\t"<< \
                // // q_desired_(15) << "\t"<< q_desired_(16) << "\t"<< q_desired_(17) << "\t"<< q_desired_(18) << "\t"<< q_desired_(19) << "\t"<< q_desired_(20) << "\t"<< q_desired_(21) << "\t"<< q_desired_(22) << "\t"<< \
                // // q_(15) << "\t" <<  q_(16) << "\t" <<  q_(17) << "\t" <<  q_(18) << "\t" <<  q_(19) << "\t" <<  q_(20) << "\t" <<  q_(21) << "\t" <<  q_(22) <<endl;                // if(control_time_ > tc.command_time + tc.traj_time)
                // //     fout.close();             
            }

        }
        else
        {
            wc_.set_contact(red_, 1, 1);
            torque_grav = wc_.gravity_compensation_torque(red_, dc.fixedgravity);
            //torque_grav = wc_.task_control_torque_QP_gravity(red_);
        }

        TorqueDesiredLocal = torque_grav + torque_task;

        if (dc.torqueredis)
        {
            dc.torqueredis = false;
            cr_mode++;
            if (cr_mode > 2)
                cr_mode = 0;

            if (cr_mode == 0)
                std::cout << "contact torque redistribution by yslee " << std::endl;
            else if (cr_mode == 1)
                std::cout << "contact torque redistribution by qp " << std::endl;
            else if (cr_mode == 2)
                std::cout << "contact torque redistribution disabled " << std::endl;
        }

        if (cr_mode == 0)
        {
            TorqueContact = wc_.contact_force_redistribution_torque(red_, TorqueDesiredLocal, fc_redis, fc_ratio);
        }
        else if (cr_mode == 1)
        {
            TorqueContact = wc_.contact_torque_calc_from_QP(red_, TorqueDesiredLocal);
        }

        //acceleration_estimated = (A_matrix_inverse * N_C * Slc_k_T * (TorqueDesiredLocal - torque_grav)).segment(6, MODEL_DOF);
        //acceleration_observed = q_dot_ - q_dot_before_;
        //q_dot_before_ = q_dot_;
        //std::cout << "acceleration_observed : " << std::endl;
        //std::cout << acceleration_observed << std::endl;
        //acceleration_differance = acceleration_observed - acceleration_estimated_before;
        //acceleration_estimated_before = acceleration_estimated;

        ///////////////////////////////////////////////////////////////////////////////////////
        //////////////////              Controller Code End             ///////////////////////
        ///////////////////////////////////////////////////////////////////////////////////////

        mtx.lock();
        torque_desired = TorqueDesiredLocal + TorqueContact;
        //dc.accel_dif = acceleration_differance;
        //dc.accel_obsrvd = acceleration_observed;
        mtx.unlock();

        //wc_.task_control_torque(J_task,Eigen)
        //wc_.get_contact_force(TorqueDesiredLocal);
        //red_.ZMP_local = wc_.GetZMPpos();

        red_.ContactForce = wc_.get_contact_force(red_, torque_desired);
        red_.ZMP = wc_.GetZMPpos(red_);

        //red_.ZMP_eqn_calc(0) = (red_.link_[COM_id].x_traj(0) * 9.8 - red_.com_.pos(2) * red_.link_[COM_id].a_traj(0)) / 9.8;
        red_.ZMP_eqn_calc(0) = (red_.link_[COM_id].x_traj(1) * 9.81 - (red_.com_.pos(2) - red_.link_[Right_Foot].xpos(2) * 0.5 - red_.link_[Left_Foot].xpos(2) * 0.5) * red_.link_[COM_id].a_traj(1)) / 9.81;
        red_.ZMP_eqn_calc(1) = (red_.link_[COM_id].x_traj(1) * 9.81 - (red_.com_.pos(2) - red_.link_[Right_Foot].xpos(2) * 0.5 - red_.link_[Left_Foot].xpos(2) * 0.5) * red_.link_[COM_id].a_traj(1)) / 9.81 + red_.com_.angular_momentum(0) / (red_.com_.mass * 9.81);
        red_.ZMP_eqn_calc(2) = 0.0;

        //pubfromcontroller();

        //std::cout << "ZMP desired : " << red_.ZMP_desired(1) << "\tZMP ft : " << red_.ZMP_ft(1) << "\tZMP error : " << red_.ZMP_error(1) << std::endl;
        //std::cout << "zmp error x : "<< zmp1(0) <<"  y : "<< zmp1(1)<<std::endl;

        //VectorXd tau_coriolis;
        //RigidBodyDynamics::NonlinearEffects(model_,red_.q_virtual_,red_.q_dot_virtual_,tau_coriolis)

        if (dc.shutdown)
            break;
        first = false;

        std::chrono::duration<double> elapsed_time = std::chrono::high_resolution_clock::now() - dyn_loop_start;

        est += elapsed_time.count();

        //std::this_thread::sleep_until(dyn_loop_start + dc.dym_timestep);
    }
}

void RedController::tuiThread()
{
    while (!dc.shutdown && ros::ok())
    {
        static double before_time;

        mtx_terminal.lock();
        if (dc.ncurse_mode)
            mvprintw(4, 30, "I'm alive : %f", ros::Time::now().toSec());
        mtx_terminal.unlock();
        for (int i = 0; i < 40; i++)
        {
            if (dc.Tq_[i].update && dc.ncurse_mode)
            {
                mtx_terminal.lock();
                mvprintw(dc.Tq_[i].y, dc.Tq_[i].x, dc.Tq_[i].text);
                dc.Tq_[i].update = false;
                if (dc.Tq_[i].clr_line)
                {
                    move(dc.Tq_[i].y, 0);
                    clrtoeol();
                }
                mtx_terminal.unlock();
            }
            else if (dc.Tq_[i].update && (!dc.ncurse_mode))
            {
                std::cout << dc.Tq_[i].text << std::endl;
                dc.Tq_[i].update = false;
            }
        }
        if (dc.ncurse_mode)
        {
            if (getch() == 'q')
            {
                dc.shutdown = true;
            }
        }
        else if (!ros::ok())
        {
            dc.shutdown = true;
        }

        if (control_time_ != before_time)
            pubfromcontroller();

        before_time = control_time_;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void RedController::getState()
{
    int count = 0;
    if (dc.testmode == false)
    {
        while ((time == dc.time) && ros::ok())
        {
            std::this_thread::sleep_for(std::chrono::nanoseconds(50));
            count++;
        }
    }
    mtx_dc.lock();

    time = dc.time;
    control_time_ = dc.time;
    sim_time = dc.sim_time;
    dym_hz = dc.dym_hz;
    stm_hz = dc.stm_hz;
    q_ = dc.q_;
    q_virtual_ = dc.q_virtual_;
    q_dot_ = dc.q_dot_;
    q_dot_virtual_ = dc.q_dot_virtual_;
    q_ddot_virtual_ = dc.q_ddot_virtual_;
    torque_ = dc.torque_;

    red_.q_ = dc.q_;
    red_.q_virtual_ = dc.q_virtual_;
    red_.q_dot_ = dc.q_dot_;
    red_.q_dot_virtual_ = dc.q_dot_virtual_;
    red_.q_ddot_virtual_ = dc.q_ddot_virtual_;

    static bool first_run = true;
    if (first_run)
    {
        for (int i = 0; i < LINK_NUMBER + 1; i++)
        {

            red_.link_[i] = dc.link_[i];
        }
        first_run = false;
    }
    for (int i = 0; i < LINK_NUMBER + 1; i++)
    {
        red_.link_[i].xpos = dc.link_[i].xpos;
        red_.link_[i].xipos = dc.link_[i].xipos;
        red_.link_[i].Rotm = dc.link_[i].Rotm;
        red_.link_[i].Jac = dc.link_[i].Jac;
        red_.link_[i].Jac_COM = dc.link_[i].Jac_COM;
        red_.link_[i].Jac_COM_p = dc.link_[i].Jac_COM_p;
        red_.link_[i].Jac_COM_r = dc.link_[i].Jac_COM_r;
        red_.link_[i].COM_position = dc.link_[i].COM_position;
        //red_.link_[i].xpos_contact = dc.link_[i].xpos_contact;
        red_.link_[i].v = dc.link_[i].v;
        red_.link_[i].w = dc.link_[i].w;
    }

    red_.yaw_radian = dc.yaw_radian;
    red_.roll = dc.roll;
    red_.pitch = dc.pitch;
    red_.yaw = dc.yaw;

    red_.A_ = dc.A_;
    red_.com_ = dc.com_;

    mtx_dc.unlock();
}

void RedController::initialize()
{
    torque_desired.setZero();
}

void RedController::ContinuityChecker(double data)
{
}

void RedController::ZMPmonitor()
{
}