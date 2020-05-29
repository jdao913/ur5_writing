#include "mjxmacro.h"
#include "uitools.h"
#include "stdio.h"
#include "string.h"
#include "mujoco.h"
// #include "ur5.h"
#include "mj_robot.h"
#include "control.h"
#include <eigen3/Eigen/Dense>
// #include <eigen3/Eigen/Eigen>
#include <cmath>
#include <iostream>
#include <vector>
#include <tuple>
#include <math.h>

using namespace Eigen;

// Returns the desired taskspace position, velocity, and acceleration of a 5th order polynomial trajectory
// given the inputted start and end state, the total time, and the current time. Returns the values
// as a 9 long vector of position, velocity, acceleration, with 3 values for each (xyz)
VectorXd poly_traj5(VectorXd start, VectorXd end, double total_t, double curr_t) {
    assert (curr_t < total_t);
    VectorXd state_diff = end - start;
    int state_size = start.size() / 3;      // Assume fully actuated
    VectorXd pos = start.head(state_size) + (10/pow(total_t,3)*pow(curr_t,3) - 15/pow(total_t,4)*pow(curr_t,4) + 6/pow(total_t,5)*pow(curr_t,5)) * state_diff.head(state_size);
    VectorXd vel = (30/pow(total_t,3)*pow(curr_t,2) - 60/pow(total_t,4)*pow(curr_t,3) + 30/pow(total_t,5)*pow(curr_t,4)) * state_diff.segment(state_size, state_size);
    VectorXd accel = (60/pow(total_t,3)*curr_t - 180/pow(total_t,4)*pow(curr_t,2) + 120/pow(total_t,5)*pow(curr_t,3)) * state_diff.tail(state_size);
    // Vector3d desired_state(pos, vel, accel);
    VectorXd desired_state(start.size());
    desired_state << pos, vel, accel;
    return desired_state;

}

// Returns torques for the desired position and velocity and inputted Kp and Kd gains using a simple PD controller
std::tuple<VectorXd, VectorXd> joint_PID(mjr_t *r, VectorXd des_pos, VectorXd des_vel, VectorXd des_accel, VectorXd prev_error, double Kp, double Ki, double Kd) {
    // Get current joint position and velocity
    VectorXd qpos(r->m->nq);
    VectorXd qvel(r->m->nv);
    mju_copy(qpos.data(), r->d->qpos, r->m->nq);
    mju_copy(qvel.data(), r->d->qvel, r->m->nv);
    // std::cout << "des pos\n" << des_pos << std::endl;
    // std::cout << "des vel\n" << des_vel << std::endl;
    // std::cout << "des accel\n" << des_accel << std::endl;

    // Compute commanded joint acceleration
    VectorXd pos_error = des_pos - qpos;
    VectorXd vel_error = des_vel - qvel;
    VectorXd int_error = prev_error + pos_error;
    VectorXd command_accel = des_accel + Kp*pos_error + Ki*int_error + Kd*vel_error;

    // Get current inertia matrix and h matrix
    mj_forward(r->m, r->d);
    MatrixXd fullM(r->m->nv, r->m->nv);
    MatrixXd h(r->m->nv, 1);
    mj_fullM(r->m, fullM.data(), r->d->qM);
    mj_rne(r->m, r->d, 1, h.data());

    // Compute torques from acceleration
    VectorXd torque = fullM*command_accel + h;

    return {torque, int_error};
}

// Given a desired taskspace position, uses inverse kinematics to find the corresponding joint position
// then using a PD controller with the inputted gains, returns the corresponding torque
VectorXd inv_kin(mjr_t *r, Vector3d des_pos, double Kp, double Kd) {
    // dx = x_des - x_curr;
    // dq = Jp_inv * dx;
    // torque = kp * dq + kd * (-qvel);
    // Get current end-effector position
    int ee_id = mj_name2id(r->m, mjOBJ_BODY, "EE");     // Get end-effector body id in mujoco model
    Vector3d ee_pos;
    mju_copy(ee_pos.data(), &r->d->xpos[3*ee_id], 3);

    // Get current end-effector Jacobian
    MatrixXd Jp(3, r->m->nv);       // position Jacobian
    MatrixXd Jr(3, r->m->nv);       // rotation Jacobian
    mj_jacBody(r->m, r->d, Jp.data(), Jr.data(), ee_id);

    // Compute Jacobian inverse
    CompleteOrthogonalDecomposition<MatrixXd> cod(Jp);
    MatrixXd Jp_pinv = cod.pseudoInverse();

    VectorXd task_pos_error = des_pos - ee_pos;
    VectorXd q_error = Jp_pinv * task_pos_error;
    VectorXd qvel(r->m->nv);
    mju_copy(qvel.data(), r->d->qvel, r->m->nv);
    // VectorXd torque = Kp*q_error + Kd*(-qvel);
    VectorXd command_accel = Kp*q_error + Kd*(-qvel);

    // Get current inertia matrix and h matrix
    mj_forward(r->m, r->d);
    MatrixXd fullM(r->m->nv, r->m->nv);
    MatrixXd h(r->m->nv, 1);
    mj_fullM(r->m, fullM.data(), r->d->qM);
    mj_rne(r->m, r->d, 1, h.data());

    // Compute torques from acceleration
    VectorXd torque = fullM*command_accel + h;

    return torque;
}

std::tuple<VectorXd, VectorXd, MatrixXd> get_torques(mjr_t *r, VectorXd desired_state, VectorXd prev_vel_error, MatrixXd prev_J, double Kp, double Ki, double Kd) {
    mj_forward(r->m, r->d);
    // Get current jointspace inertia matrix
    // mj_crb(r->m, r->d);      // Call mj_crb first to compute qM
    MatrixXd fullM(r->m->nv, r->m->nv);
    mj_fullM(r->m, fullM.data(), r->d->qM);
    // std::cout << "jointspace inertia M: \n" << fullM << "\n";
    // Get current end-effector Jacobian
    MatrixXd Jp(3, r->m->nv);       // position Jacobian
    MatrixXd Jr(3, r->m->nv);       // rotation Jacobian
    int ee_id = mj_name2id(r->m, mjOBJ_BODY, "EE");     // Get end-effector body id in mujoco model
    mj_jacBody(r->m, r->d, Jp.data(), Jr.data(), ee_id);
    MatrixXd Jdot = (1/r->m->opt.timestep) * (prev_J - Jp);
    mj_comVel(r->m, r->d);
    Vector3d ee_pos;
    Vector3d ee_vel;
    mju_copy(ee_pos.data(), &r->d->xpos[3*ee_id], 3);
    mju_copy(ee_vel.data(), &r->d->cvel[6*ee_id+3], 3);
    // printf("got ee pos and vel\n");
    // std::cout << "jacobian : \n" << Jp << "\n";
    // Compute Jacobian inverse
    CompleteOrthogonalDecomposition<MatrixXd> cod(Jp);
    MatrixXd Jp_pinv = cod.pseudoInverse();
    // std::cout << "Jacobian pInv: \n" << Jp_pinv << "\n";
    // Compute taskpace inertia matrix
    // lambda = (J*Mq*JT).inverse();
    MatrixXd cond_mat = 0.001*MatrixXd::Identity(3, 3);
    MatrixXd taskM = (Jp*fullM*Jp.transpose() + cond_mat).inverse();
    // MatrixXd taskM = Jp_pinv.transpose() * fullM * Jp_pinv;
    std::cout << "taskspace inertia M: \n" << taskM << "\n";
    // Get jointspace h matrix
    MatrixXd h(r->m->nv, 1);
    mj_rne(r->m, r->d, 1, h.data());
    // Compute taskspace h matrix
    VectorXd qdot = VectorXd::Zero(r->m->nv);
    for (int i = 0; i < r->m->nv; i++) {
        qdot(i) = r->d->qvel[i];
    }
    // std::cout << "qdot:\n" << qdot << std::endl;
    // std::cout << "J inv xdot:\n" << Jp_pinv*ee_vel << std::endl;
    MatrixXd eta = Jp_pinv.transpose() * h;// - taskM*Jdot*qdot;     // Ignore Jdot term
    // Compute torques
    // std::cout << "desired accels: \n" << desired_state.tail(3) << "\n";
    VectorXd pos_error = desired_state.head(3) - ee_pos;
    VectorXd d_error = desired_state.segment(3, 3)-ee_vel;
    prev_vel_error += pos_error;


    VectorXd torque = Jp.transpose() * (taskM * (desired_state.tail(3)+Kp*pos_error+Kd*d_error+Ki*prev_vel_error) + eta);

    return {torque, prev_vel_error, Jp};
}

// Given the inputted desired taskspace state (pos, vel, accel), the current state of the robot r,
// and the P-PI control gains, returns the corresponding taskspace acceleration command for a P-PI
// controller with a feedforward component. Also requires the velocity error from the previous timestep.
// Returns a (3, 2) Matrix, where the first column is the acceleration command and the second column
// is the current integrated velocity error
MatrixXd get_task_accel(mjr_t *r, VectorXd desired_state, VectorXd prev_vel_error, double Kv, double Kp, double Ki) {
    // Given x, dx, ddx (commands), y, dy (current state)
    // Get end effector position and velocity
    Vector3d ee_pos;
    Vector3d ee_vel;
    int ee_id = mj_name2id(r->m, mjOBJ_BODY, "EE");     // Get end-effector body id in mujoco model
    mj_comVel(r->m, r->d);
    mju_copy(ee_pos.data(), &r->d->xpos[3*ee_id], 3);
    mju_copy(ee_vel.data(), &r->d->cvel[6*ee_id+3], 3);
    // vel_command = Kv*(x - y) + dx
    // std::cout << "desired task pos:\n" << desired_state.head(3) << std::endl;
    // std::cout << "desired task vel:\n" << desired_state.segment(3, 3) << std::endl;
    // std::cout << "desired task accel:\n" << desired_state.tail(3) << std::endl;
    VectorXd vel_command = Kv*(desired_state.head(3) - ee_pos) + desired_state.segment(3, 3);
    // Int Vel Error = int(vel_command - dy)
    VectorXd int_vel_error = prev_vel_error + (vel_command - ee_vel);
    // Acc Command = kp*(vel_command - dy) + Kp*Ki*int vel error + ddx
    VectorXd acc_command = Kp*(vel_command - ee_vel) + Kp*Ki*int_vel_error + desired_state.tail(3);
    Matrix<double, 3, 2> output;
    output << acc_command, int_vel_error;
    // std::cout << "int vel error:\n" << int_vel_error << std::endl;
    // std::cout << "acc command:\n" << acc_command << std::endl;
    return output;

}

// Given the inputted desired taskspace state (pos, vel, accel), the current state of the robot r,
// and the PID control gains, returns the corresponding taskspace acceleration command for a PID
// controller with a feedforward component. Also requires the velocity error from the previous timestep.
// Returns a (3, 2) Matrix, where the first column is the acceleration command and the second column
// is the current integrated velocity error
MatrixXd get_task_accel_PID(mjr_t *r, VectorXd desired_state, VectorXd prev_pos_error, double Kv, double Kp, double Ki) {
    // Given x, dx, ddx (commands), y, dy (current state)
    // Get end effector position and velocity
    Vector3d ee_pos;
    Vector3d ee_vel;
    int ee_id = mj_name2id(r->m, mjOBJ_BODY, "EE");     // Get end-effector body id in mujoco model
    mj_comVel(r->m, r->d);
    mju_copy(ee_pos.data(), &r->d->xpos[3*ee_id], 3);
    mju_copy(ee_vel.data(), &r->d->cvel[6*ee_id+3], 3);
    // vel_command = Kv*(x - y) + dx
    // std::cout << "desired task pos:\n" << desired_state.head(3) << std::endl;
    // std::cout << "desired task vel:\n" << desired_state.segment(3, 3) << std::endl;
    // std::cout << "desired task accel:\n" << desired_state.tail(3) << std::endl;
    VectorXd d_error = (desired_state.segment(3, 3) - ee_vel);
    VectorXd pos_error = (desired_state.head(3) - ee_pos);
    VectorXd int_error = (prev_pos_error + pos_error);
    // Acc Command = kp*(vel_command - dy) + Kp*Ki*int vel error + ddx
    VectorXd acc_command = desired_state.tail(3) + Kp*pos_error + Ki*int_error + Kv*d_error;
    Matrix<double, 3, 2> output;
    output << acc_command, int_error;
    // std::cout << "int vel error:\n" << int_vel_error << std::endl;
    // std::cout << "acc command:\n" << acc_command << std::endl;
    return output;

}

// Use jacobian to transform taskspace acceleration into jointspace acceleration
std::tuple <VectorXd, MatrixXd> task2joint_accel(mjr_t *r, VectorXd task_accel, MatrixXd prev_J) {
    // Get current end-effector Jacobian
    MatrixXd Jp(3, r->m->nv);       // position Jacobian
    MatrixXd Jr(3, r->m->nv);       // rotation Jacobian
    int ee_id = mj_name2id(r->m, mjOBJ_BODY, "EE");     // Get end-effector body id in mujoco model
    mj_jacBody(r->m, r->d, Jp.data(), Jr.data(), ee_id);
    // Compute Jacobian pseudo-inverse
    CompleteOrthogonalDecomposition<MatrixXd> cod(Jp);
    MatrixXd Jp_pinv = cod.pseudoInverse();
    // Compute Jdot
    MatrixXd Jdot = (1/r->m->opt.timestep) * (prev_J - Jp);
    // Compute jointspace accel
    VectorXd qdot = VectorXd::Zero(r->m->nv);
    for (int i = 0; i < r->m->nv; i++) {
        qdot(i) = r->d->qvel[i];
    }
    // mju_copy(qdot.data(), r->d->qvel, r->m->nv);
    printf("copied qdot\n");
    VectorXd qddot = Jp_pinv * (task_accel - Jdot*qdot);     // Ignore Jdot term for now
    return {qddot, Jp};
}

// Use inverse dynamics to solve for joint torques given the desired joint accelerations
VectorXd inv_dyn(mjr_t *r, VectorXd joint_accel) {
    // Get current jointspace inertia matrix
    mj_crb(r->m, r->d);      // Call mj_crb first to compute qM
    MatrixXd fullM(r->m->nv, r->m->nv);
    mj_fullM(r->m, fullM.data(), r->d->qM);
    // Get jointspace h matrix
    MatrixXd h(r->m->nv, 1);
    mj_rne(r->m, r->d, 0, h.data());
    // Compute torque
    VectorXd torque = fullM * joint_accel + h;
    return torque;
}

// main function
int main(int argc, const char** argv) {

    // activate software
    const char* key_buf = getenv("MUJOCO_KEY_PATH");
    mj_activate(key_buf);

    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // ur5_t* robot = ur5_init();
    mjr_t* robot = mjr_init("./model/ur5.xml");
    std::cout << "nu: " << robot->m->nu << "\n";
    mj_forward(robot->m, robot->d);
    int ee_id = mj_name2id(robot->m, mjOBJ_BODY, "EE");     // Get end-effector body id in mujoco model

    int nq = robot->m->nq;
    int nv = robot->m->nv;

    printf("nq: %i\t nv: %i\n", nq, nv);

    Vector3d ee_pos;
    mju_copy(ee_pos.data(), &robot->d->xpos[3*ee_id], 3);
    std::cout << "init ee pos\n" << ee_pos << std::endl;

    // Joint PID testing setup
    VectorXd init_pos(robot->m->nq);
    VectorXd init_vel = VectorXd::Zero(robot->m->nv);
    VectorXd init_accel = VectorXd::Zero(robot->m->nv);
    mju_copy(init_pos.data(), robot->d->qpos, robot->m->nq);
    VectorXd start(nq+nv+nv);
    start << init_pos, init_vel, init_accel;
    std::cout << "start pos:\n" << start << "\n";
    VectorXd des_pos = VectorXd::Zero(robot->m->nq);
    int test_ind = std::stoi(argv[4]);
    des_pos(test_ind) = M_PI / 4;
    for (int i = 0; i < nq; i++) {
        des_pos(i) = M_PI/4;
    }
    VectorXd end(nq+nv+nv);
    end << des_pos, init_vel, init_accel;
    std::cout << "end pos:\n" << end << std::endl;
    double total_t = 2.0;
    // double curr_t = .2;
    // VectorXd desired_state = poly_traj5(start, end, total_t , curr_t);
    // std::cout << desired_state << "\n";

    // MatrixXd torque = get_torques(robot, desired_state);
    // std::cout << torque << "\n";
    // Vector3d int_vel_error = Vector3d::Zero(3);
    // MatrixXd accel = get_task_accel(robot, desired_state, int_vel_error, 1.0, 1.0, 1.0);
    // std::cout << "task accel output:\n" << accel << std::endl;
    // std::cout << "task accel:\n" << accel.col(0) << std::endl;
    // VectorXd joint_accel = task2joint_accel(robot, accel.col(0));
    // std::cout << "joint accel:\n" << joint_accel << std::endl;
    // VectorXd torque = inv_dyn(robot, joint_accel);
    // std::cout << "torque:\n" << torque << std::endl;

    Vector3d traj_start(0.0, 0.0, 0.6);
    Vector3d traj_vel(0.1, 0.0, 0.0);
    Vector3d int_vel_error = Vector3d::Zero(3);
    VectorXd int_qpos_error = VectorXd::Zero(nq);
    // double Kp = 0.02;
    // double Ki = 0.0;
    // double Kv = 0.01;
    double Kp = std::stod(argv[1]);
    double Ki = std::stod(argv[2]);
    double Kv = std::stod(argv[3]);
    // printf("gains: Kp=%f\tKi=%f\tKv=%f\n", Kp, Ki, Kv);
    // Get inital Jacobian for Jdot later
    // Get current end-effector Jacobian
    MatrixXd prev_Jp(3, robot->m->nv);       // position Jacobian
    MatrixXd Jr(3, robot->m->nv);       // rotation Jacobian
    mj_jacBody(robot->m, robot->d, prev_Jp.data(), Jr.data(), ee_id);
    // std::cout << "init J:\n" << prev_Jp << std::endl;
 
    // bool render_state = ur5_render(robot);
    bool render_state = mjr_render(robot);
    double render_time = 60.0;
    // run main loop, target real-time simulation and 60 fps rendering
    double clear_time = 0.0;
    while(robot->d->time < total_t and render_state) {
        
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        if (!robot->paused) {
            // SIMULATION WITH NO CONTROL
            // mjtNum simstart = robot->d->time;
            // while( robot->d->time - simstart < 1.0/render_time )
                // mj_step(robot->m, robot->d);

            // Test single joint with PD controller
            // VectorXd des_state = poly_traj5(start, end, total_t , robot->d->time);
            // VectorXd des_state;
            // des_state = end.replicate(1, 1);
            // VectorXd des_pos = VectorXd::Zero(robot->m->nq);
            // des_pos(1) = M_PI / 2.0;
            // VectorXd des_vel = VectorXd::Zero(robot->m->nv);
            // VectorXd des_accel = VectorXd::Zero(robot->m->nv);
            // VectorXd torque;
            // std::tie(torque, int_qpos_error) = joint_PID(robot, des_state.head(robot->m->nq), des_state.segment(robot->m->nq, robot->m->nv), des_state.tail(robot->m->nv), int_qpos_error, Kp, Ki, Kv);
            // mju_copy(robot->d->ctrl, torque.data(), robot->m->nu);
            // mj_step(robot->m, robot->d);
            // printf("qpos: ");
            // for (int i = 0; i < robot->m->nq; i++) {
            //     printf("%f", robot->d->qpos[i]);
            //     if (i != robot->m->nq-1) {
            //         printf(", ");
            //     }
            // }
            // printf("\n");

            // Taskspace inverse kinematics
            Vector3d desired_state(-0.2, 0.0, 0.9);
            VectorXd torque = inv_kin(robot, desired_state, Kp, Kv);
            mju_copy(robot->d->ctrl, torque.data(), robot->m->nu);
            mj_step(robot->m, robot->d);
            printf("ee pos: %f, %f, %f\n", robot->d->xpos[3*ee_id], robot->d->xpos[3*ee_id+1], robot->d->xpos[3*ee_id+2]);

            // TASKSPACE INVERSE DYNAMICS METHOD
            // VectorXd desired_state = poly_traj5(start, end, total_t , robot->d->time);
            // VectorXd desired_state(9);
            // desired_state << 2.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            // // std::cout << desired_state << std::endl;
            // VectorXd torque, new_int_vel_error;
            // MatrixXd new_Jp;
            // std::tie(torque, new_int_vel_error, new_Jp) = get_torques(robot, desired_state, int_vel_error, prev_Jp, Kp, Ki, Kv);
            // prev_Jp = new_Jp.replicate(1,1);
            // int_vel_error = new_int_vel_error.replicate(1, 1);
            // mju_copy(robot->d->ctrl, torque.data(), robot->m->nu);
            // mj_step(robot->m, robot->d);
            // // std::cout << "torques: \n" << torque << "\n";
            // printf("ee pos: %f, %f, %f\n", robot->d->xpos[3*ee_id], robot->d->xpos[3*ee_id+1], robot->d->xpos[3*ee_id+2]);

            // TASKSPACE ACCELERATION -> JOINTSPACE INVERSE DYNAMICS METHOD
            // Vector3d desired_pos = traj_start + robot->d->time*traj_vel;
            // Matrix<double, 9, 1> desired_state;
            // desired_state << desired_pos, traj_vel, Vector3d::Zero(3);
        //     std::cout << "desired state:\n" << desired_state << std::endl;
        //     // MatrixXd task_accel = get_task_accel(robot, desired_state, int_vel_error, Kv, Kp, Ki);
        //     MatrixXd task_accel = get_task_accel_PID(robot, desired_state, int_vel_error, Kv, Kp, Ki);
        //     int_vel_error += task_accel.col(1);
        //     // printf("got task accel\n");
        //     auto [joint_accel, curr_Jp] = task2joint_accel(robot, task_accel.col(0), prev_Jp);
        //     // printf("converted to joint accel\n");
        //     VectorXd torque = inv_dyn(robot, joint_accel);
        //     mju_copy(robot->d->ctrl, torque.data(), robot->m->nu);
        //     mj_step(robot->m, robot->d);
        //     prev_Jp = curr_Jp.replicate(1,1);
        //     // std::cout << "prev Jp\n" << prev_Jp << std::endl;
        //     // std::cout << "curr Jp\n" << curr_Jp << std::endl;
        //     printf("ee pos: %f, %f, %f\n", robot->d->xpos[3*ee_id], robot->d->xpos[3*ee_id+1], robot->d->xpos[3*ee_id+2]);
        }

        // render_state = ur5_render(robot);
        render_state = mjr_render(robot);
    }

    while (render_state) {
        render_state = mjr_render(robot);
    }
    // ur5_free(robot);
    mjr_free(robot);

    return 1;
}