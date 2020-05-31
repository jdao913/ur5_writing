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
#include <fstream>
#include <sstream>

using namespace Eigen;


typedef Matrix<double, -1, -1, RowMajor> MatrixXd_rowMaj;

IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");

// Returns the desired n-dim position (taskspace or jointspace) of a 5th order polynomial trajectory
// given the inputted start and end state, the total time, and the current time. Returns the values
// as a 3*n long vector of position, velocity, acceleration, with n values for each
VectorXd poly_traj5(VectorXd start, VectorXd end, double total_t, double curr_t) {
    assert (curr_t < total_t);
    VectorXd state_diff = end - start;
    VectorXd pos = start + (10/pow(total_t,3)*pow(curr_t,3) - 15/pow(total_t,4)*pow(curr_t,4) + 6/pow(total_t,5)*pow(curr_t,5)) * state_diff;
    VectorXd vel = (30/pow(total_t,3)*pow(curr_t,2) - 60/pow(total_t,4)*pow(curr_t,3) + 30/pow(total_t,5)*pow(curr_t,4)) * state_diff;
    VectorXd accel = (60/pow(total_t,3)*curr_t - 180/pow(total_t,4)*pow(curr_t,2) + 120/pow(total_t,5)*pow(curr_t,3)) * state_diff;
    VectorXd desired_state(start.size()*3);
    desired_state << pos, vel, accel;
    return desired_state;

}

// Joint PID controller. Takes in a mjr_t robot, desired position, velocity, and acceleration, along with PID gains and a sum of 
// previous position error. Returns the computed torque and current integrated position error.
std::tuple<VectorXd, VectorXd> joint_PID(mjr_t *r, VectorXd des_pos, VectorXd des_vel, VectorXd des_accel, VectorXd prev_error, 
                                        double Kp, double Ki, double Kd) {
    // Get current joint position and velocity
    VectorXd qpos(r->m->nq);
    VectorXd qvel(r->m->nv);
    mju_copy(qpos.data(), r->d->qpos, r->m->nq);
    mju_copy(qvel.data(), r->d->qvel, r->m->nv);

    // Compute commanded joint acceleration
    VectorXd pos_error = des_pos - qpos;
    VectorXd vel_error = des_vel - qvel;
    VectorXd int_error = prev_error + pos_error;
    VectorXd command_accel = des_accel + Kp*pos_error + Ki*int_error + Kd*vel_error;

    // Get current inertia matrix and h matrix
    mj_crb(r->m, r->d);
    MatrixXd_rowMaj fullM(r->m->nv, r->m->nv);
    MatrixXd_rowMaj h(r->m->nv, 1);
    mj_fullM(r->m, fullM.data(), r->d->qM);
    mj_rne(r->m, r->d, 1, h.data());

    // Compute torques from acceleration
    VectorXd torque = fullM*command_accel + h;

    return {torque, int_error};
}

// Given a desired taskspace position, uses inverse kinematics to find the corresponding joint position
// then using a PD controller with the inputted gains, returns the corresponding torque
VectorXd inv_kin_control(mjr_t *r, Vector3d des_pos, Vector3d des_vel, double Kp, double Kd) {
    // dx = x_des - x_curr;
    // dq = Jp_inv * dx;
    // torque = kp * dq + kd * (-qvel);
    // Get current end-effector position
    int ee_id = mj_name2id(r->m, mjOBJ_SITE, "EE");     // Get end-effector body id in mujoco model
    int link4 = mj_name2id(r->m, mjOBJ_SITE, "link1");     // Get end-effector body id in mujoco model
    mj_comVel(r->m, r->d);
    Vector3d ee_pos;
    Vector3d ee_vel;
    mju_copy(ee_pos.data(), &r->d->site_xpos[3*ee_id], 3);
    mju_copy(ee_vel.data(), &r->d->cvel[6*link4+3], 3);

    // Get current end-effector Jacobian
    MatrixXd_rowMaj Jp(3, r->m->nv);       // position Jacobian
    // MatrixXd_rowMaj Jr(3, r->m->nv);       // rotation Jacobian
    mj_forward(r->m, r->d);
    mj_jacSite(r->m, r->d, Jp.data(), NULL, ee_id);
    std::cout << "Jacobian\n" << Jp << std::endl;

    // Compute Jacobian inverse
    CompleteOrthogonalDecomposition<MatrixXd_rowMaj> cod(Jp);
    MatrixXd_rowMaj Jp_pinv = cod.pseudoInverse();

    VectorXd task_pos_error = des_pos - ee_pos;
    // VectorXd task_vel_error = des_vel - ee_vel;
    VectorXd q_error = Jp_pinv * task_pos_error;
    // VectorXd qv_error = Jp_pinv * task_vel_error;
    VectorXd qvel(r->m->nv);
    mju_copy(qvel.data(), r->d->qvel, r->m->nv);
    // VectorXd torque = Kp*q_error + Kd*(-qvel);
    std::cout << "Jp*qvel: " << (Jp*qvel).format(CommaInitFmt) << std::endl;
    std::cout << "ee_vel: " << ee_vel.format(CommaInitFmt) << std::endl;
    VectorXd torque = Kp*q_error + Kd*(-qvel);

    return torque;
}

// Given a desired taskspace position, uses inverse kinematics to find the corresponding joint position.
// Uses simple iterative Jacobian transpose method. Takes in as input a desired taskspace position, the maximum
// number of iterations to run, a step size, and a mjr robot to run on. Returns a vector containing the computed joint postions
VectorXd inv_kin_iter(mjr_t *r, Vector3d des_pos, int iters, double tol, double step_size) {
    // Make copy of mjData for Jacobian computations and joint position updates
    mjData* d_copy = mj_makeData(r->m);
    mj_copyData(d_copy, r->m, r->d);
    Vector3d ee_pos;
    int ee_id = mj_name2id(r->m, mjOBJ_BODY, "EE");     // Get end-effector body id in mujoco model
    
    MatrixXd_rowMaj Jp(3, r->m->nv);       // position Jacobian
    MatrixXd_rowMaj Jr(3, r->m->nv);       // rotation Jacobian
    for (int k = 0; k < iters; k++) {
        // Get current end-effector Jacobian
        mj_forward(r->m, d_copy);
        mj_jacBody(r->m, d_copy, Jp.data(), Jr.data(), ee_id);

        // Compute task and joint space position error
        mju_copy(ee_pos.data(), &d_copy->xpos[3*ee_id], 3);
        VectorXd task_pos_error = des_pos - ee_pos;
        VectorXd q_error = Jp.transpose() * task_pos_error;

        // Update joint positions with error and step size
        for (int i = 0; i < r->m->nq; i++) {
            d_copy->qpos[i] += step_size*q_error(i);
        }
        printf("iter %i\t ee pos error: %f\n", k, task_pos_error.norm());
        if (task_pos_error.norm() < tol) {      // Stop if already within tolerance
            break;
        }
    }

    // Output final computed joint position
    VectorXd final_qpos(r->m->nq);
    for (int i = 0; i < r->m->nq; i++) {
        final_qpos(i) = d_copy->qpos[i];
    }

    return final_qpos;
}

// Computes torques using taskspace inverse dynamics
std::tuple<VectorXd, VectorXd, MatrixXd_rowMaj> get_torques(mjr_t *r, VectorXd desired_state, VectorXd prev_vel_error, MatrixXd_rowMaj prev_J, double Kp, double Ki, double Kd) {
    // Get current jointspace inertia matrix
    mj_crb(r->m, r->d);      // Call mj_crb first to compute qM
    MatrixXd_rowMaj fullM(r->m->nv, r->m->nv);
    mj_fullM(r->m, fullM.data(), r->d->qM);
    // std::cout << "jointspace inertia M: \n" << fullM << "\n";
    // Get current end-effector Jacobian
    MatrixXd_rowMaj Jp(3, r->m->nv);       // position Jacobian
    MatrixXd_rowMaj Jr(3, r->m->nv);       // rotation Jacobian
    int ee_id = mj_name2id(r->m, mjOBJ_BODY, "EE");     // Get end-effector body id in mujoco model
    mj_jacBody(r->m, r->d, Jp.data(), Jr.data(), ee_id);
    MatrixXd_rowMaj Jdot = (1/r->m->opt.timestep) * (prev_J - Jp);
    mj_comVel(r->m, r->d);
    Vector3d ee_pos;
    Vector3d ee_vel;
    mju_copy(ee_pos.data(), &r->d->xpos[3*ee_id], 3);
    mju_copy(ee_vel.data(), &r->d->cvel[6*ee_id+3], 3);
    // std::cout << "jacobian : \n" << Jp << "\n";
    // Compute Jacobian inverse
    CompleteOrthogonalDecomposition<MatrixXd_rowMaj> cod(Jp);
    MatrixXd_rowMaj Jp_pinv = cod.pseudoInverse();
    // std::cout << "Jacobian pInv: \n" << Jp_pinv << "\n";
    // Compute taskpace inertia matrix
    // lambda = (J*Mq*JT).inverse();
    // MatrixXd_rowMaj cond_mat = 0.001*MatrixXd_rowMaj::Identity(3, 3);
    // MatrixXd_rowMaj taskM = (Jp*fullM*Jp.transpose()).inverse();// + cond_mat).inverse();
    MatrixXd_rowMaj taskM = Jp_pinv.transpose() * fullM * Jp_pinv;
    // std::cout << "taskspace inertia M: \n" << taskM << "\n";
    // Get jointspace h matrix
    MatrixXd_rowMaj h(r->m->nv, 1);
    mj_rne(r->m, r->d, 1, h.data());
    // Compute taskspace h matrix
    VectorXd qdot = VectorXd::Zero(r->m->nv);
    for (int i = 0; i < r->m->nv; i++) {
        qdot(i) = r->d->qvel[i];
    }
    // std::cout << "qdot:\n" << qdot << std::endl;
    // std::cout << "J inv xdot:\n" << Jp_pinv*ee_vel << std::endl;
    MatrixXd_rowMaj eta = Jp_pinv.transpose() * h;// - taskM*Jdot*qdot;     // Ignore Jdot term
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
MatrixXd_rowMaj get_task_accel(mjr_t *r, VectorXd desired_state, VectorXd prev_vel_error, double Kv, double Kp, double Ki) {
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
MatrixXd_rowMaj get_task_accel_PID(mjr_t *r, VectorXd desired_state, VectorXd prev_pos_error, double Kv, double Kp, double Ki) {
    // Given x, dx, ddx (commands), y, dy (current state)
    // Get end effector position and velocity
    Vector3d ee_pos;
    Vector3d ee_vel;
    int ee_id = mj_name2id(r->m, mjOBJ_BODY, "EE");     // Get end-effector body id in mujoco model
    mj_comVel(r->m, r->d);
    mju_copy(ee_pos.data(), &r->d->xpos[3*ee_id], 3);
    mju_copy(ee_vel.data(), &r->d->cvel[6*ee_id + 3], 3);

    VectorXd d_error = (desired_state.segment(3, 3) - ee_vel);
    VectorXd pos_error = (desired_state.head(3) - ee_pos);
    VectorXd int_error = (prev_pos_error + pos_error);
    // std::cout << "pos error: " << pos_error.format(CommaInitFmt) << std::endl;
    // std::cout << "prev pos error: " << prev_pos_error.format(CommaInitFmt) << std::endl;
    // std::cout << "int  pos error: " << int_error.format(CommaInitFmt) << std::endl;
    VectorXd acc_command = desired_state.tail(3) + Kp*pos_error + Ki*int_error + Kv*d_error;
    Matrix<double, 3, 2> output;
    output << acc_command, int_error;

    return output;

}

// Use jacobian to transform taskspace acceleration into jointspace acceleration
std::tuple <VectorXd, MatrixXd_rowMaj> task2joint_accel(mjr_t *r, VectorXd task_accel, MatrixXd_rowMaj prev_J) {
    // Get current end-effector Jacobian
    MatrixXd_rowMaj Jp(3, r->m->nv);       // position Jacobian
    MatrixXd_rowMaj Jr(3, r->m->nv);       // rotation Jacobian
    int ee_id = mj_name2id(r->m, mjOBJ_BODY, "EE");     // Get end-effector body id in mujoco model
    mj_jacBody(r->m, r->d, Jp.data(), Jr.data(), ee_id);
    // std::cout << "Jacobian\n" << Jp << std::endl;
    // Compute Jacobian pseudo-inverse
    // CompleteOrthogonalDecomposition<MatrixXd_rowMaj> cod(Jp);
    // MatrixXd_rowMaj Jp_pinv = cod.pseudoInverse();
    // MatrixXd_rowMaj Jp_pinv = Jp.colPivHouseholderQr().solve();
    // Compute Jdot
    // MatrixXd_rowMaj Jdot = (1/r->m->opt.timestep) * (Jp - prev_J);
    // Compute jointspace accel
    // VectorXd qdot = VectorXd::Zero(r->m->nv);
    // for (int i = 0; i < r->m->nv; i++) {
        // qdot(i) = r->d->qvel[i];
    // }
    // mju_copy(qdot.data(), r->d->qvel, r->m->nv);
    // printf("copied qdot\n");
    // VectorXd qddot = Jp_pinv * (task_accel);// - Jdot*qdot);     // Ignore Jdot term for now
    VectorXd qddot = Jp.colPivHouseholderQr().solve(task_accel);
    return {qddot, Jp};
}

// Use inverse dynamics to solve for joint torques given the desired joint accelerations
VectorXd inv_dyn(mjr_t *r, VectorXd joint_accel) {
    // Get current jointspace inertia matrix
    mj_crb(r->m, r->d);      // Call mj_crb first to compute qM
    MatrixXd_rowMaj fullM(r->m->nv, r->m->nv);
    mj_fullM(r->m, fullM.data(), r->d->qM);
    // Get jointspace h matrix
    MatrixXd_rowMaj h(r->m->nv, 1);
    mj_rne(r->m, r->d, 0, h.data());
    // Compute torque
    VectorXd torque = fullM * joint_accel;// + h;
    return torque;
}

int test_joint_PID_single(mjr_t* robot, int argc, const char** argv) {
    if (argc < 5) {
        printf("Error: requires 4 arguments of Kp, Ki, Kd gains, and which joint (0-nq) to test.\n");
        return 0;
    }
    double Kp = std::stod(argv[1]);
    double Ki = std::stod(argv[2]);
    double Kv = std::stod(argv[3]);
    int test_ind = std::stoi(argv[4]);
    printf("Testing joint %i with gains Kp=%f, Ki=%f, Kv=%f\n", test_ind, Kp, Ki, Kv);
    // Good gains seem to be 2, 0.0, 5
    // Initial state is current state with zero vel and accel
    int nq = robot->m->nq;
    int nv = robot->m->nv;
    VectorXd init_pos(nq);
    VectorXd init_vel = VectorXd::Zero(nv);
    VectorXd init_accel = VectorXd::Zero(nv);
    mju_copy(init_pos.data(), robot->d->qpos, nq);
    VectorXd start(nq+nv+nv);
    start << init_pos, init_vel, init_accel;
    std::cout << "start pos:\n" << start << "\n";

    // Desired state is all joint angles zero with single joint at pi/4 with vel and accel zero
    VectorXd des_pos = VectorXd::Zero(nq);
    des_pos(test_ind) = -M_PI / 4;
    VectorXd end(nq+nv+nv);
    end << des_pos, init_vel, init_accel;
    std::cout << "end pos:\n" << end << std::endl;

    double total_t = 2.0;
    bool render_state = mjr_render(robot);
    VectorXd int_qpos_error = VectorXd::Zero(nq);

    while(robot->d->time < total_t and render_state) {
        if (!robot->paused) {          

            VectorXd des_state = poly_traj5(init_pos, des_pos, total_t , robot->d->time);
            // VectorXd des_state;
            // des_state = end.replicate(1, 1);
            VectorXd torque;
            std::tie(torque, int_qpos_error) = joint_PID(robot, des_state.head(nq), des_state.segment(nq, nv), des_state.tail(nv), int_qpos_error, Kp, Ki, Kv);
            mju_copy(robot->d->ctrl, torque.data(), robot->m->nu);
            std::cout << "torque\n" << torque << std::endl;
            mj_step(robot->m, robot->d);
            printf("qpos: ");
            for (int i = 0; i < nq; i++) {
                printf("%f", robot->d->qpos[i]);
                if (i != robot->m->nq-1) {
                    printf(", ");
                }
            }
            printf("\n");
        }
        render_state = mjr_render(robot);
    }

    while(render_state) {
        render_state = mjr_render(robot);
    }

    return 1;
}

int test_joint_PID_all(mjr_t* robot, int argc, const char** argv) {
    if (argc < 4) {
        printf("Error: requires 3 arguments of Kp, Ki, Kd gains.\n");
        return 0;
    }
    double Kp = std::stod(argv[1]);
    double Ki = std::stod(argv[2]);
    double Kv = std::stod(argv[3]);
    // Decent gains seem to be 2, 0.0, 5
    // Initial state is current state with zero vel and accel
    int nq = robot->m->nq;
    int nv = robot->m->nv;
    VectorXd init_pos(nq);
    VectorXd init_vel = VectorXd::Zero(nv);
    VectorXd init_accel = VectorXd::Zero(nv);
    mju_copy(init_pos.data(), robot->d->qpos, nq);
    VectorXd start(nq+nv+nv);
    start << init_pos, init_vel, init_accel;
    std::cout << "start pos:\n" << start << "\n";

    // Desired state is all joint angles zero with single joint at pi/4 with vel and accel zero
    VectorXd des_pos = VectorXd::Zero(nq);
    for (int i = 0; i < nq; i++) {
        des_pos(i) = M_PI / 4;
    }
    VectorXd end(nq+nv+nv);
    end << des_pos, init_vel, init_accel;
    std::cout << "end pos:\n" << end << std::endl;

    double total_t = 2.0;
    bool render_state = mjr_render(robot);
    VectorXd int_qpos_error = VectorXd::Zero(nq);

    while(robot->d->time < total_t and render_state) {
        if (!robot->paused) {          

            VectorXd des_state = poly_traj5(init_pos, des_pos, total_t , robot->d->time);
            // VectorXd des_state;
            // des_state = end.replicate(1, 1);
            VectorXd torque;
            std::tie(torque, int_qpos_error) = joint_PID(robot, des_state.head(nq), des_state.segment(nq, nv), des_state.tail(nv), int_qpos_error, Kp, Ki, Kv);
            mju_copy(robot->d->ctrl, torque.data(), robot->m->nu);
            mj_step(robot->m, robot->d);
            printf("qpos: ");
            for (int i = 0; i < robot->m->nq; i++) {
                printf("%f", robot->d->qpos[i]);
                if (i != robot->m->nq-1) {
                    printf(", ");
                }
            }
            printf("\n");
        }
        render_state = mjr_render(robot);
    }

    while(render_state) {
        render_state = mjr_render(robot);
    }

    return 1;
}

int inv_kin_iter_test(mjr_t* robot, int argc, const char** argv) {
    if (argc < 3) {
        printf("Error: requires 2 arguments of number of iters and step size.\n");
        return 0;
    }
    double num_iters = std::stod(argv[1]);
    double damping = std::stod(argv[2]);

    // for (int i = 0; i < nq; i++) {
    //     robot->d->qpos[i] = 1;
    // }
    // mj_forward(robot->m, robot->d);

    bool render_state = mjr_render(robot);
    bool done = false;
    Vector3d end_ee(-0.5, -0.5, 0.0);

    while(render_state) {
        if (!robot->paused and !done) {   
            VectorXd final_qpos = inv_kin_iter(robot, end_ee, num_iters, 1e-5, damping);
            mju_copy(robot->d->qpos, final_qpos.data(), robot->m->nq);
            done = true;
        }
        render_state = mjr_render(robot);
    }
    return 1;
}

int test_taskspace_inv_kin(mjr_t* robot, int argc, const char** argv) {
    if (argc < 3) {
        printf("Error: requires 2 arguments of Kp, Kv gains.\n");
        return 0;
    }
    double Kp = std::stod(argv[1]);
    double Kv = std::stod(argv[2]);
    int nq = robot->m->nq;
    int nv = robot->m->nv;

    // robot->d->qpos[0] = .1;
    for (int i = 0; i < nq; i++) {
        robot->d->qpos[i] = 1;
    }
    mj_forward(robot->m, robot->d);
    // Initial state is current state with zero vel and accel
    int ee_id = mj_name2id(robot->m, mjOBJ_SITE, "EE");     // Get end-effector body id in mujoco model
    VectorXd ee_pos_init(3);
    mju_copy(ee_pos_init.data(), &robot->d->site_xpos[3*ee_id], 3);
    VectorXd end_ee(3);
    // end_ee << 0.0, 0.0, 1.6;
    end_ee << 0.0, 0.0, 1.5;
    std::cout << "init ee pos: " << ee_pos_init.format(CommaInitFmt) << std::endl;
    std::cout << "desired ee pos: " << end_ee.format(CommaInitFmt) << std::endl;

    double total_t = 20.0;
    bool render_state = mjr_render(robot);
    VectorXd int_qpos_error = VectorXd::Zero(nq);

    while(robot->d->time < total_t and render_state) {
        if (!robot->paused) {          

            // Vector3d des_state(-0.6, 0.0, 0.0);
            // VectorXd des_state = poly_traj5(ee_pos_init, end_ee, total_t, robot->d->time);
            // std::cout << "Des state\n" << des_state << std::endl;
            VectorXd torque = inv_kin_control(robot, end_ee, Vector3d::Zero(3), Kp, Kv);
            // VectorXd torque = inv_kin(robot, end_ee, Vector3d::Zero(3), Kp, Kv);
            mju_copy(robot->d->ctrl, torque.data(), robot->m->nu);
            std::cout << "torque" << torque << std::endl;
            mj_step(robot->m, robot->d);
            printf("ee pos: %f, %f, %f\n", robot->d->site_xpos[3*ee_id], robot->d->site_xpos[3*ee_id+1], robot->d->site_xpos[3*ee_id+2]);
        }
        render_state = mjr_render(robot);
    }

    while(render_state) {
        render_state = mjr_render(robot);
    }

    return 1;
}

// Tests the taskspace inverse dynamics method
int test_taskspace_inv_dyn(mjr_t* robot, int argc, const char** argv) {
    if (argc < 4) {
        printf("Error: requires 2 arguments of Kp, Ki, Kv gains.\n");
        return 0;
    }
    double Kp = std::stod(argv[1]);
    double Ki = std::stod(argv[2]);
    double Kv = std::stod(argv[3]);
    int nq = robot->m->nq;
    int nv = robot->m->nv;

    for (int i = 0; i < nq; i++) {
        robot->d->qpos[i] = 1;
    }
    mj_forward(robot->m, robot->d);
    // Initial state is current state with zero vel and accel
    int ee_id = mj_name2id(robot->m, mjOBJ_BODY, "EE");     // Get end-effector body id in mujoco model
    VectorXd ee_pos_init(3);
    mju_copy(ee_pos_init.data(), &robot->d->xpos[3*ee_id], 3);
    VectorXd end_ee(3);
    end_ee << -0.5, -0.5, 0.0;
    std::cout << "init ee pos: " << ee_pos_init.format(CommaInitFmt) << std::endl;
    std::cout << "desired ee pos: " << end_ee.format(CommaInitFmt) << std::endl;

    double total_t = 2.0;
    Vector3d int_vel_error = Vector3d::Zero(3);
     // Get current end-effector Jacobian
    MatrixXd_rowMaj prev_Jp(3, robot->m->nv);       // position Jacobian
    MatrixXd_rowMaj Jr(3, robot->m->nv);       // rotation Jacobian
    mj_jacBody(robot->m, robot->d, prev_Jp.data(), Jr.data(), ee_id);
    bool render_state = mjr_render(robot);
    VectorXd int_qpos_error = VectorXd::Zero(nq);

    while(robot->d->time < total_t and render_state) {
        if (!robot->paused) {          

            // VectorXd desired_state = poly_traj5(ee_pos_init, end_ee, total_t , robot->d->time);
            VectorXd desired_state(9);
            desired_state << -0.5, -0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            std::cout << "desired state: " << desired_state.format(CommaInitFmt) << std::endl;
            VectorXd torque, new_int_vel_error;
            MatrixXd_rowMaj new_Jp;
            std::tie(torque, new_int_vel_error, new_Jp) = get_torques(robot, desired_state, int_vel_error, prev_Jp, Kp, Ki, Kv);
            prev_Jp = new_Jp.replicate(1,1);
            int_vel_error = new_int_vel_error.replicate(1, 1);
            mju_copy(robot->d->ctrl, torque.data(), robot->m->nu);
            mj_step(robot->m, robot->d);
            // std::cout << "torques: \n" << torque << "\n";
            printf("ee pos: %f, %f, %f\n", robot->d->xpos[3*ee_id], robot->d->xpos[3*ee_id+1], robot->d->xpos[3*ee_id+2]);
        }
        render_state = mjr_render(robot);
    }

    while(render_state) {
        render_state = mjr_render(robot);
    }

    return 1;
}

// Tests the taskspace acceleration method. Do PID control in taskspace to get a desired taskspace acceleration command,
// then convert it to a desired jointspace acceleration command using Jacobian inverse and then solve the inverse dynamics
// to get the corresponding torques.
int test_taskspace_accel(mjr_t* robot, int argc, const char** argv) {
    if (argc < 4) {
        printf("Error: requires 2 arguments of Kp, Ki, Kv gains.\n");
        return 0;
    }
    double Kp = std::stod(argv[1]);
    double Ki = std::stod(argv[2]);
    double Kv = std::stod(argv[3]);
    int nq = robot->m->nq;
    int nv = robot->m->nv;

    for (int i = 0; i < nq; i++) {
        robot->d->qpos[i] = 1;
    }
    mj_forward(robot->m, robot->d);
    // Initial state is current state with zero vel and accel
    int ee_id = mj_name2id(robot->m, mjOBJ_BODY, "EE");     // Get end-effector body id in mujoco model
    VectorXd ee_pos_init(3);
    mju_copy(ee_pos_init.data(), &robot->d->xpos[3*ee_id], 3);
    VectorXd end_ee(3);
    end_ee << -0.5, -0.5, 0.0;
    std::cout << "init ee pos: " << ee_pos_init.format(CommaInitFmt) << std::endl;
    std::cout << "desired ee pos: " << end_ee.format(CommaInitFmt) << std::endl;

    double total_t = 3.0;
    Vector3d int_vel_error = Vector3d::Zero(3);
     // Get current end-effector Jacobian
    MatrixXd_rowMaj prev_Jp(3, robot->m->nv);       // position Jacobian
    MatrixXd_rowMaj Jr(3, robot->m->nv);       // rotation Jacobian
    mj_jacBody(robot->m, robot->d, prev_Jp.data(), Jr.data(), ee_id);
    bool render_state = mjr_render(robot);

    Vector3d traj_start(-.5, 0, 0);
    Vector3d traj_vel(-.2, 0, 0);

    while(robot->d->time < total_t and render_state) {
        if (!robot->paused) {          
            Vector3d desired_pos = traj_start + robot->d->time*traj_vel;
            // Vector3d desired_pos(-.5, 0.0, 0);
            VectorXd desired_state(9);
            desired_state << desired_pos, traj_vel, Vector3d::Zero(3);
            std::cout << "desired state:\n" << desired_state.format(CommaInitFmt) << std::endl;
            // MatrixXd task_accel = get_task_accel(robot, desired_state, int_vel_error, Kv, Kp, Ki);
            MatrixXd task_accel = get_task_accel_PID(robot, desired_state, int_vel_error, Kv, Kp, Ki);
            std::cout << "task accel col 1" << task_accel.col(1).format(CommaInitFmt) << std::endl;
            int_vel_error += task_accel.col(1);
            // printf("got task accel\n");
            auto [joint_accel, curr_Jp] = task2joint_accel(robot, task_accel.col(0), prev_Jp);
            // printf("converted to joint accel\n");
            VectorXd torque = inv_dyn(robot, joint_accel);
            // std::cout << "torque\n" << torque << std::endl;
            mju_copy(robot->d->ctrl, torque.data(), robot->m->nu);

            mj_step(robot->m, robot->d);
            prev_Jp = curr_Jp.replicate(1,1);
            // std::cout << "prev Jp\n" << prev_Jp << std::endl;
            // std::cout << "curr Jp\n" << curr_Jp << std::endl;
            printf("ee pos: %f, %f, %f\n", robot->d->xpos[3*ee_id], robot->d->xpos[3*ee_id+1], robot->d->xpos[3*ee_id+2]);
        }
        render_state = mjr_render(robot);
    }

    while(render_state) {
        render_state = mjr_render(robot);
    }

    return 1;
}

int letter_test(mjr_t* robot, int argc, const char** argv) {
    if (argc < 4) {
        printf("Error: requires 2 arguments of Kp, Ki, Kv gains.\n");
        return 0;
    }
    double Kp = std::stod(argv[1]);
    double Ki = std::stod(argv[2]);
    double Kv = std::stod(argv[3]);
    bool debug = false;
    bool viz = false;
    bool save_data = true;
    FILE *fp = fopen("./letter_test.csv", "w+");

    int nq = robot->m->nq;
    int ee_id = mj_name2id(robot->m, mjOBJ_BODY, "EE");     // Get end-effector body id in mujoco model

    for (int i = 0; i < nq; i++) {
        robot->d->qpos[i] = 1.5;
    }
    robot->d->qpos[3] = 1.0;
    robot->d->qpos[4] = 1.2;
    mj_forward(robot->m, robot->d);
     // Get current end-effector Jacobian
    MatrixXd_rowMaj prev_Jp(3, robot->m->nv);       // position Jacobian
    mj_jacBody(robot->m, robot->d, prev_Jp.data(), NULL, ee_id);
    bool render_state = true;
    if (viz) {
        render_state = mjr_render(robot);
    } else{
        mjr_close_render(robot);
    }
    Vector3d int_pos_error = Vector3d::Zero(3);

    
    std::ifstream letter_file("./trajectory_gen/cap_a_traj.csv");
    if (!letter_file.is_open()) throw std::runtime_error("Could not open file");

    std::string line, col_value;
    // std::getline(letter_file, line);
    // std::stringstream init_ss(line);
    // while (std::getline(init_ss, col_value, ',')) {
    //     printf("%f ", std::stod(col_value));
    // }
    // printf("\n");
    // std::getline(letter_file, line);
    // std::stringstream next_ss(line);
    // while (std::getline(next_ss, col_value, ',')) {
    //     printf("%f ", std::stod(col_value));
    // }
    // printf("\n");
    // return 1;
    
    // Give robot total_t to reach first desired state
    float total_t = 4.0;
    std::getline(letter_file, line);    // Read next line of traj file 
    // Extract desired state from string
    std::stringstream init_ss(line);
    VectorXd desired_state(9);
    for (int i = 0; i < 3; i++) {
        std::getline(init_ss, col_value, ',');
        desired_state(i) = std::stod(col_value);
        std::getline(init_ss, col_value, ',');
        desired_state(i+1) = std::stod(col_value);
        desired_state(i+2) = 0.0;
    }
    int int_count = 0;
    int int_max = 20;
    int save_count = 0;
    
    while(robot->d->time < total_t and render_state) {
        if (!robot->paused || !viz) {        
            if (int_count >= int_max) {
                int_count = 0;
                int_pos_error = Vector3d::Zero(3);
            }
            
            // MatrixXd task_accel = get_task_accel(robot, desired_state, int_vel_error, Kv, Kp, Ki);
            MatrixXd task_accel = get_task_accel_PID(robot, desired_state, int_pos_error, Kv, Kp, Ki);
            int_pos_error = task_accel.col(1);
            // std::cout << "int pos error" << int_pos_error.format(CommaInitFmt) << std::endl;
            // printf("got task accel\n");
            auto [joint_accel, curr_Jp] = task2joint_accel(robot, task_accel.col(0), prev_Jp);
            // printf("converted to joint accel\n");
            VectorXd torque = inv_dyn(robot, joint_accel);
            // std::cout << "torque\n" << torque << std::endl;
            mju_copy(robot->d->ctrl, torque.data(), robot->m->nu);

            mj_step(robot->m, robot->d);
            prev_Jp = curr_Jp.replicate(1,1);
            // std::cout << "prev Jp\n" << prev_Jp << std::endl;
            // std::cout << "curr Jp\n" << curr_Jp << std::endl;
            if (debug) {
                std::cout << "desired state:" << desired_state.format(CommaInitFmt) << std::endl;
                printf("ee pos: %f, %f, %f\n", robot->d->xpos[3*ee_id], robot->d->xpos[3*ee_id+1], robot->d->xpos[3*ee_id+2]);
            }
            int_count += 1;
        }
        if (viz) {
            render_state = mjr_render(robot);
        }
    }

    int_pos_error = Vector3d::Zero(3);
    int step_count = 60;     // How many steps to take before updating desired state
    int curr_step = 0;
    // Kp += 20;
    // Ki = 0.1;
    // Kv -= 2;
    int_max = 100;
    fprintf(fp, "%f, %f, %f\n", robot->d->xpos[3*ee_id], robot->d->xpos[3*ee_id+1], robot->d->xpos[3*ee_id+2]);
    while(!letter_file.eof() and render_state) {
        if (!robot->paused || !viz) {   
            if (int_count >= int_max) {
                int_count = 0;
                int_pos_error = Vector3d::Zero(3);
            }

            if (curr_step == step_count) { 
                if (letter_file.eof()) {
                    break;
                }
                std::getline(letter_file, line);    // Read next line of traj file 
                // Extract desired state from string
                std::stringstream ss(line);
                // desired_state = Vector3d::Zero(6);
                // std::cout << "line: " << line << std::endl;
                for (int i = 0; i < 2; i++) {
                    std::getline(ss, col_value, ',');
                    // std::cout << "col value: " << col_value << std::endl;
                    desired_state(i) = std::max(-4.5, std::min(std::stod(col_value), 4.5));
                    // desired_state(3*i) = std::max(-4.5, std::min(std::stod(col_value), 4.5));
                    // std::getline(ss, col_value, ',');
                    // desired_state(3*i+1) = std::max(-4.5, std::min(std::stod(col_value), 4.5));
                    // desired_state(3*i+2) = 0.0;
                }
                curr_step = 0;
                if (save_data) {
                    fprintf(fp, "%f, %f, %f\n", robot->d->xpos[3*ee_id], robot->d->xpos[3*ee_id+1], robot->d->xpos[3*ee_id+2]);
                    save_count += 1;
                    if (save_count % 50 == 0) {
                        printf("Saved %i states\n", save_count);
                        printf("sim time: %f", robot->d->time);
                    }
                }
            }
            
            // MatrixXd task_accel = get_task_accel(robot, desired_state, int_vel_error, Kv, Kp, Ki);
            MatrixXd task_accel = get_task_accel_PID(robot, desired_state, int_pos_error, Kv, Kp, Ki);
            int_pos_error = task_accel.col(1);
            // printf("got task accel\n");
            auto [joint_accel, curr_Jp] = task2joint_accel(robot, task_accel.col(0), prev_Jp);
            // printf("converted to joint accel\n");
            VectorXd torque = inv_dyn(robot, joint_accel);
            // std::cout << "torque\n" << torque << std::endl;
            mju_copy(robot->d->ctrl, torque.data(), robot->m->nu);

            mj_step(robot->m, robot->d);
            prev_Jp = curr_Jp.replicate(1,1);
            // std::cout << "prev Jp\n" << prev_Jp << std::endl;
            // std::cout << "curr Jp\n" << curr_Jp << std::endl;
            if (debug) {
                std::cout << "desired state:" << desired_state.format(CommaInitFmt) << std::endl;
                printf("ee pos: %f, %f, %f\n", robot->d->xpos[3*ee_id], robot->d->xpos[3*ee_id+1], robot->d->xpos[3*ee_id+2]);
                printf("sim time: %f", robot->d->time);
            }
            curr_step += 1;
            int_count += 1;
        }
        if (viz) {
            render_state = mjr_render(robot);
        }
    }
    printf("sim time: %f", robot->d->time);

    // while (std::getline(letter_file, line)) {
    //     std::stringstream ss(line);
    //     VectorXd desired_state(6);
    //     for (int i = 0; i < 6; i++) {
    //         std::getline(ss, col_value, ',');
    //         desired_state(i) = std::stod(col_value);
    //         // printf("%f ", std::stod(col_value));
    //     }
    //     std::cout << desired_state << std::endl;
    //     // return 1;

    // }
    return 1;

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
    mjr_t* robot = mjr_init("./model/plane_arm_big.xml");

    // test_joint_PID_single(robot, argc, argv);
    // test_joint_PID_all(robot, argc, argv);
    // inv_kin_iter_test(robot, argc, argv);
    // test_taskspace_inv_kin(robot, argc, argv);
    // test_taskspace_inv_dyn(robot, argc, argv);
    // test_taskspace_accel(robot, argc, argv);
    letter_test(robot, argc, argv);
    mjr_free(robot);
 
    return 1;
}