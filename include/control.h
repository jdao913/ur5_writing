#include "mjxmacro.h"
#include "uitools.h"
#include "stdio.h"
#include "string.h"
#include "mujoco.h"
// #include "ur5.h"
#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace Eigen;

VectorXd poly_traj5(Vector3d start, Vector3d end, double total_t, double curr_t);