#include "mjxmacro.h"
#include "uitools.h"
#include "stdio.h"
#include "string.h"
#include "mujoco.h"
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace Eigen;

VectorXd inv_kin_iter(mjr_t *r, Vector3d des_pos, int iters, double tol, double step_size);
std::tuple<VectorXd, VectorXd> IK_joint_PID(mjr_t *r, Vector3d desired_state, VectorXd prev_pos_error, double Kp, double Ki, double Kd);
int inv_kin_iter_test(mjr_t* robot, int argc, const char** argv);
