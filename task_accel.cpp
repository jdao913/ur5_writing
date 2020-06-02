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


// Given the inputted desired taskspace state (pos, vel, accel), the current state of the robot r,
// and the PID control gains, returns the corresponding taskspace acceleration command for a PID
// controller with a feedforward component. Also requires the position error from the previous timestep.
// Returns a (3, 2) Matrix, where the first column is the acceleration command and the second column
// is the current integrated position error
std::tuple <VectorXd, VectorXd> get_task_accel_PID(mjr_t *r, VectorXd desired_state, VectorXd prev_pos_error, double Kp, double Ki, double Kd) {
    // Get end effector position and velocity
    Vector3d ee_pos;
    Vector3d ee_vel;
    int ee_id = mj_name2id(r->m, mjOBJ_BODY, "EE");     // Get end-effector body id in mujoco model
    mj_comVel(r->m, r->d);
    mju_copy(ee_pos.data(), &r->d->xpos[3*ee_id], 3);
    mju_copy(ee_vel.data(), &r->d->cvel[6*ee_id + 3], 3);

    // PID Controller
    VectorXd d_error = (desired_state.segment(3, 3) - ee_vel);
    VectorXd pos_error = (desired_state.head(3) - ee_pos);
    VectorXd int_error = (prev_pos_error + pos_error);
    VectorXd acc_command = desired_state.tail(3) + Kp*pos_error + Ki*int_error + Kd*d_error;

    return {acc_command, int_error};

}

// Use jacobian to transform taskspace acceleration into jointspace acceleration
VectorXd task2joint_accel(mjr_t *r, VectorXd task_accel) {
    // Get current end-effector Jacobian
    MatrixXd_rowMaj Jp(3, r->m->nv);       // position Jacobian
    MatrixXd_rowMaj Jr(3, r->m->nv);       // rotation Jacobian
    int ee_id = mj_name2id(r->m, mjOBJ_BODY, "EE");     // Get end-effector body id in mujoco model
    mj_jacBody(r->m, r->d, Jp.data(), Jr.data(), ee_id);
    
    // Solve for joint accel using Jacobian
    VectorXd qddot = Jp.colPivHouseholderQr().solve(task_accel);    // Ignore Jdot term
    return qddot;
}

// Tests the taskspace acceleration method. Do PID control in taskspace to get a desired taskspace acceleration command,
// then convert it to a desired jointspace acceleration command using Jacobian inverse and then solve the inverse dynamics
// to get the corresponding torques.
int test_taskspace_accel(mjr_t* robot, int argc, const char** argv) {
    IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");
    if (argc < 4) {
        printf("Error: requires 2 arguments of Kp, Ki, Kv gains.\n");
        return 0;
    }
    double Kp = std::stod(argv[1]);
    double Ki = std::stod(argv[2]);
    double Kd = std::stod(argv[3]);
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
    Vector3d int_pos_error = Vector3d::Zero(3);
   
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
            VectorXd task_accel;
            std::tie(task_accel, int_pos_error) = get_task_accel_PID(robot, desired_state, int_pos_error, Kp, Ki, Kd);
            VectorXd joint_accel = task2joint_accel(robot, task_accel);
            VectorXd torque = inv_dyn(robot, joint_accel);
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