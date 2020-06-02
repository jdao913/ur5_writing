#include "mjxmacro.h"
#include "uitools.h"
#include "stdio.h"
#include "string.h"
#include "mujoco.h"
#include "mj_robot.h"
#include "control.h"
#include "joint_PID.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>
#include <tuple>
#include <math.h>

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
        if (task_pos_error.norm() < tol) {      // Stop if already within tolerance
            break;
        }
    }

    // Output final computed joint position
    VectorXd final_qpos(r->m->nq);
    for (int i = 0; i < r->m->nq; i++) {
        final_qpos(i) = d_copy->qpos[i];
    }

    mj_deleteData(d_copy);

    return final_qpos;
}

// Given a desired taskspace state, returns torques. Computes torques by using inverse kinematics to transform 
// desired taskspace state into desired jointspace state and then does a PID controller to get desired joint 
// acceleration command, then does inverse dynamics to get torque. Returns torque and updated integrated position error.
std::tuple<VectorXd, VectorXd> IK_joint_PID(mjr_t *r, Vector3d desired_state, VectorXd prev_pos_error, double Kp, double Ki, double Kd) {
    VectorXd des_joint = inv_kin_iter(r, desired_state, 200, 1e-5, .7);
    // std::cout << "des joint: " << des_joint.format(CommaInitFmt) << std::endl;
    // printf("finished IK\n");
    VectorXd torque, new_int_error;
    VectorXd zero_vel = VectorXd::Zero(r->m->nv);
    // printf("finished declare\n");
    std::tie(torque, new_int_error) = joint_PID(r, des_joint, zero_vel, zero_vel, prev_pos_error, Kp, Ki, Kd);
    // printf("computed torque\n");

    return {torque, new_int_error};
            
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
    robot->d->qpos[0] = 1.57;
    robot->d->qpos[1] = 1.41;
    robot->d->qpos[2] = 1.26;
    robot->d->qpos[3] = 1.11;
    robot->d->qpos[4] = 1.48;

    bool render_state = mjr_render(robot);
    bool done = false;
    Vector3d end_ee(0.0, 0.0, 0.0);

    while(render_state) {
        if (!robot->paused and !done) {   
            VectorXd final_qpos = inv_kin_iter(robot, end_ee, num_iters, 1e-3, damping);
            mju_copy(robot->d->qpos, final_qpos.data(), robot->m->nq);
            done = true;
        }
        render_state = mjr_render(robot);
    }
    return 1;
}