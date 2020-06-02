#include "mjxmacro.h"
#include "uitools.h"
#include "stdio.h"
#include "string.h"
#include "mujoco.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
// #include "mj_robot.h"

std::tuple<VectorXd, Vector3d> task_inv_dyn(mjr_t *r, VectorXd desired_state, Vector3d prev_pos_error, double Kp, double Ki, double Kd);
int test_taskspace_inv_dyn(mjr_t* robot, int argc, const char** argv);