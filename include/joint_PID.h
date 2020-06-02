#include "mjxmacro.h"
#include "uitools.h"
#include "stdio.h"
#include "string.h"
#include "mujoco.h"
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace Eigen;

VectorXd poly_traj5(VectorXd start, VectorXd end, double total_t, double curr_t);
std::tuple<VectorXd, VectorXd> joint_PID(mjr_t *r, VectorXd des_pos, VectorXd des_vel, VectorXd des_accel, VectorXd prev_error, 
                                        double Kp, double Ki, double Kd);
VectorXd inv_dyn(mjr_t *r, VectorXd joint_accel);
int test_joint_PID_single(mjr_t* robot, int argc, const char** argv);
int test_joint_PID_all(mjr_t* robot, int argc, const char** argv);