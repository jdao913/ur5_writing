#include "mjxmacro.h"
#include "uitools.h"
#include "stdio.h"
#include "string.h"
#include "mujoco.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
// #include "mj_robot.h"

std::tuple <VectorXd, VectorXd> get_task_accel_PID(mjr_t *r, VectorXd desired_state, VectorXd prev_pos_error, double Kp, double Ki, double Kd);
VectorXd task2joint_accel(mjr_t *r, VectorXd task_accel);
int test_taskspace_accel(mjr_t* robot, int argc, const char** argv);