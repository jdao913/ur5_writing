#include "mjxmacro.h"
#include "uitools.h"
#include "stdio.h"
#include "string.h"
#include "mujoco.h"
#include "mj_robot.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>
#include <tuple>
#include <math.h>

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
    // std::cout << "qpos: " << qpos.format(CommaInitFmt) << std::endl;

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