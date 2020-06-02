#include "mjxmacro.h"
#include "uitools.h"
#include "stdio.h"
#include "string.h"
#include "mujoco.h"
#include "mj_robot.h"
#include "joint_PID.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>
#include <tuple>
#include <math.h>

// Computes torques using taskspace inverse dynamics
std::tuple<VectorXd, Vector3d> task_inv_dyn(mjr_t *r, VectorXd desired_state, Vector3d prev_pos_error, double Kp, double Ki, double Kd) {
    // Get current jointspace inertia matrix
    mj_crb(r->m, r->d);      // Call mj_crb first to compute qM
    MatrixXd_rowMaj fullM(r->m->nv, r->m->nv);
    mj_fullM(r->m, fullM.data(), r->d->qM);

    // Get current end-effector Jacobian
    MatrixXd_rowMaj Jp(3, r->m->nv);       // position Jacobian
    int ee_id = mj_name2id(r->m, mjOBJ_BODY, "EE");     // Get end-effector body id in mujoco model
    mj_jacBody(r->m, r->d, Jp.data(), NULL, ee_id);
    mj_comVel(r->m, r->d);
    Vector3d ee_pos;
    mju_copy(ee_pos.data(), &r->d->xpos[3*ee_id], 3);
    VectorXd qvel = VectorXd::Zero(r->m->nv);
    mju_copy(qvel.data(), r->d->qvel, r->m->nv);
    Vector3d ee_vel = Jp*qvel;

    // Compute Jacobian inverse
    CompleteOrthogonalDecomposition<MatrixXd_rowMaj> cod(Jp);
    MatrixXd_rowMaj Jp_pinv = cod.pseudoInverse();

    // Compute taskpace inertia matrix
    MatrixXd_rowMaj taskM = Jp_pinv.transpose() * fullM * Jp_pinv;

    // Get jointspace h matrix
    MatrixXd_rowMaj h(r->m->nv, 1);
    mj_rne(r->m, r->d, 1, h.data());

    // Compute taskspace h matrix
    MatrixXd_rowMaj eta = Jp_pinv.transpose() * h;     // Ignore Jdot term
   
    // Compute torques
    VectorXd pos_error = desired_state.head(3) - ee_pos;
    VectorXd d_error = desired_state.segment(3, 3) - ee_vel;
    prev_pos_error += pos_error;

    VectorXd torque = Jp.transpose() * (taskM * (desired_state.tail(3)+Kp*pos_error+Kd*d_error+Ki*prev_pos_error) + eta);

    return {torque, prev_pos_error};
}

// Tests the taskspace inverse dynamics method
int test_taskspace_inv_dyn(mjr_t* robot, int argc, const char** argv) {
    IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
    if (argc < 4) {
        printf("Error: requires 3 arguments of Kp, Ki, Kv gains.\n");
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
    end_ee << -1.0, 0.0, 0.0;
    std::cout << "init ee pos: " << ee_pos_init.format(CommaInitFmt) << std::endl;
    std::cout << "desired ee pos: " << end_ee.format(CommaInitFmt) << std::endl;

    double total_t = 20.0;
    Vector3d int_pos_error = Vector3d::Zero(3);
    
    bool render_state = mjr_render(robot);
    VectorXd int_qpos_error = VectorXd::Zero(nq);

    int int_max = 50;
    int int_count = 0;
    while(robot->d->time < total_t and render_state) {
        if (!robot->paused) {          
            if (int_count >= int_max) {
                int_pos_error = Vector3d::Zero(3);
                int_count = 0;
            }
            // VectorXd desired_state = poly_traj5(ee_pos_init, end_ee, total_t , robot->d->time);
            VectorXd desired_state(9);
            desired_state << -1.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            std::cout << "desired state: " << desired_state.format(CommaInitFmt) << std::endl;
            VectorXd torque;
            std::tie(torque, int_pos_error) = task_inv_dyn(robot, desired_state, int_pos_error, Kp, Ki, Kv);
            mju_copy(robot->d->ctrl, torque.data(), robot->m->nu);
            mj_step(robot->m, robot->d);

            printf("ee pos: %f, %f, %f\n", robot->d->xpos[3*ee_id], robot->d->xpos[3*ee_id+1], robot->d->xpos[3*ee_id+2]);
        }
        render_state = mjr_render(robot);
    }

    while(render_state) {
        render_state = mjr_render(robot);
    }

    return 1;
}