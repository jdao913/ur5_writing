#include "mjxmacro.h"
#include "uitools.h"
#include "stdio.h"
#include "string.h"
#include "mujoco.h"
#include "mj_robot.h"
#include "control.h"
#include "inv_kin.h"
#include "task_accel.h"
#include "joint_PID.h"
#include "task_inv_dyn.h"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <iostream>
#include <vector>
#include <tuple>
#include <math.h>
#include <fstream>
#include <sstream>

using namespace Eigen;

IOFormat CommaInitFmt(StreamPrecision, DontAlignCols, ", ", ", ", "", "", " << ", ";");

int letter_test(mjr_t* robot, int argc, const char** argv) {
    if (argc < 4) {
        printf("Error: requires 4 arguments of Kp, Ki, Kv gains and step count.\n");
        return 0;
    }
    double Kp = std::stod(argv[1]);
    double Ki = std::stod(argv[2]);
    double Kd = std::stod(argv[3]);
    // Good gains are 15 0.0 4 and step count of 20
    int step_count = std::stoi(argv[4]);     // How many steps to take before updating desired state
    bool debug = false;
    bool viz = true;
    bool save_data = false;
    FILE *fp = fopen("./robot_test.csv", "w+");

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

    
    std::ifstream letter_file("./trajectory_gen/robot_traj_resize.csv");
    if (!letter_file.is_open()) throw std::runtime_error("Could not open file");

    std::string line, col_value;
    
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
    int vis_count = 100;
    int curr_vis = 0;
    
    while(robot->d->time < total_t and render_state) {
        if (!robot->paused || !viz) {        
            if (int_count >= int_max) {
                int_count = 0;
                int_pos_error = Vector3d::Zero(3);
            }
            
            // MatrixXd task_accel = get_task_accel_PID(robot, desired_state, int_pos_error, Kv, Kp, Ki);
            VectorXd task_accel;
            std::tie(task_accel, int_pos_error) = get_task_accel_PID(robot, desired_state, int_pos_error, Kp, Ki, Kd);
            // int_pos_error = task_accel.col(1);
            // std::cout << "int pos error" << int_pos_error.format(CommaInitFmt) << std::endl;
            // printf("got task accel\n");
            // auto [joint_accel, curr_Jp] = task2joint_accel(robot, task_accel.col(0), prev_Jp);
            VectorXd joint_accel = task2joint_accel(robot, task_accel);
            // printf("converted to joint accel\n");
            VectorXd torque = inv_dyn(robot, joint_accel);
            // std::cout << "torque\n" << torque << std::endl;
            mju_copy(robot->d->ctrl, torque.data(), robot->m->nu);

            mj_step(robot->m, robot->d);
            // prev_Jp = curr_Jp.replicate(1,1);
            // std::cout << "prev Jp\n" << prev_Jp << std::endl;
            // std::cout << "curr Jp\n" << curr_Jp << std::endl;
            if (debug) {
                std::cout << "desired state:" << desired_state.format(CommaInitFmt) << std::endl;
                printf("ee pos: %f, %f, %f\n", robot->d->xpos[3*ee_id], robot->d->xpos[3*ee_id+1], robot->d->xpos[3*ee_id+2]);
            }
            int_count += 1;
            curr_vis += 1;
            
        } 
        if (viz) {
            render_state = mjr_render(robot);
        }
        
    }

    int_pos_error = Vector3d::Zero(3);
    
    int curr_step = 0;
    // Kp += 20;
    // Ki = 0.1;
    // Kv -= 2;
    int_max = 100;
    curr_vis = 0;
    if (save_data) {
        fprintf(fp, "%f, %f, %f\n", robot->d->xpos[3*ee_id], robot->d->xpos[3*ee_id+1], robot->d->xpos[3*ee_id+2]);
    }
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
                if (line.empty()) {
                    break;
                }
                // Extract desired state from string
                std::stringstream ss(line);
                // desired_state = Vector3d::Zero(6);
                for (int i = 0; i < 2; i++) {
                    std::getline(ss, col_value, ',');
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
                    if (save_count % 100 == 0) {
                        printf("Saved %i states\n", save_count);
                        printf("sim time: %f\n", robot->d->time);
                    }
                }
            }
            
            // MatrixXd task_accel = get_task_accel(robot, desired_state, int_vel_error, Kv, Kp, Ki);
            // MatrixXd task_accel = get_task_accel_PID(robot, desired_state, int_pos_error, Kv, Kp, Ki);
            VectorXd task_accel;
            std::tie(task_accel, int_pos_error) = get_task_accel_PID(robot, desired_state, int_pos_error, Kp, Ki, Kd);
            // int_pos_error = task_accel.col(1);
            // printf("got task accel\n");
            // auto [joint_accel, curr_Jp] = task2joint_accel(robot, task_accel.col(0), prev_Jp);
            VectorXd joint_accel = task2joint_accel(robot, task_accel);
            // printf("converted to joint accel\n");
            VectorXd torque = inv_dyn(robot, joint_accel);
            // std::cout << "torque\n" << torque << std::endl;
            mju_copy(robot->d->ctrl, torque.data(), robot->m->nu);

            mj_step(robot->m, robot->d);
            // prev_Jp = curr_Jp.replicate(1,1);
            // std::cout << "prev Jp\n" << prev_Jp << std::endl;
            // std::cout << "curr Jp\n" << curr_Jp << std::endl;
            if (debug) {
                std::cout << "desired state:" << desired_state.format(CommaInitFmt) << std::endl;
                printf("ee pos: %f, %f, %f\n", robot->d->xpos[3*ee_id], robot->d->xpos[3*ee_id+1], robot->d->xpos[3*ee_id+2]);
                printf("sim time: %f\n", robot->d->time);
            }
            curr_step += 1;
            int_count += 1;
            curr_vis += 1;
            if (viz && curr_vis >= vis_count) {
                render_state = mjr_render(robot);
                curr_vis = 0;
            }
        } else {
            if (viz) {
                render_state = mjr_render(robot);
            }
        }
    }
    printf("sim time: %f\n", robot->d->time);
    printf("done\n");

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

int letter_test_IK(mjr_t* robot, int argc, const char** argv) {
    if (argc < 4) {
        printf("Error: requires 4 arguments of Kp, Ki, Kv gains and step count.\n");
        return 0;
    }
    double Kp = std::stod(argv[1]);
    double Ki = std::stod(argv[2]);
    double Kd = std::stod(argv[3]);
    int step_count = std::stoi(argv[4]);     // How many steps to take before updating desired state
    bool debug = false;
    bool viz = false;
    bool save_data = true;
    FILE *fp = fopen("./robot_test_ik.csv", "w+");

    int nq = robot->m->nq;
    int ee_id = mj_name2id(robot->m, mjOBJ_BODY, "EE");     // Get end-effector body id in mujoco model

    // for (int i = 0; i < nq; i++) {
    //     robot->d->qpos[i] = 1.5;
    // }
    robot->d->qpos[0] = 1.57;
    robot->d->qpos[1] = 1.41;
    robot->d->qpos[2] = 1.26;
    robot->d->qpos[3] = 1.11;
    robot->d->qpos[4] = 1.48;
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
    VectorXd int_pos_error = VectorXd::Zero(nq);

    
    std::ifstream letter_file("./trajectory_gen/robot_traj_resize.csv");
    if (!letter_file.is_open()) throw std::runtime_error("Could not open file");

    std::string line, col_value;
    
    // Give robot total_t to reach first desired state
    float total_t = 4.0;
    std::getline(letter_file, line);    // Read next line of traj file 
    // Extract desired state from string
    std::stringstream init_ss(line);
    VectorXd desired_state(3);
    for (int i = 0; i < 2; i++) {
        std::getline(init_ss, col_value, ',');
        desired_state(i) = std::stod(col_value);
    }
    desired_state(2) = 0.0;
    int int_count = 0;
    int int_max = 20;
    int save_count = 0;
    int vis_count = 100;
    int curr_vis = 0;
    
    // while(robot->d->time < total_t and render_state) {
    //     if (!robot->paused || !viz) {        
    //         if (int_count >= int_max) {
    //             int_count = 0;
    //             int_pos_error = VectorXd::Zero(nq);
    //         }
            
    //         VectorXd torque; 
    //         std::tie(torque, int_pos_error) = IK_joint_PID(robot, desired_state.head(3), int_pos_error, Kp, Ki, Kd);
    //         // std::cout << "torque\n" << torque << std::endl;
    //         mju_copy(robot->d->ctrl, torque.data(), robot->m->nu);
    //         mj_step(robot->m, robot->d);

    //         if (debug) {
    //             std::cout << "desired state:" << desired_state.format(CommaInitFmt) << std::endl;
    //             printf("ee pos: %f, %f, %f\n", robot->d->xpos[3*ee_id], robot->d->xpos[3*ee_id+1], robot->d->xpos[3*ee_id+2]);
    //         }
    //         int_count += 1;
    //         curr_vis += 1;
            
    //     } 
    //     if (viz) {
    //         render_state = mjr_render(robot);
    //     }
        
    // }

    int_pos_error = VectorXd::Zero(nq);
    
    int curr_step = 0;
    int count = 0;
    // Kp += 20;
    // Ki = 0.1;
    // Kv -= 2;
    int_max = 100;
    curr_vis = 0;
    if (save_data) {
        fprintf(fp, "%f, %f, %f\n", robot->d->xpos[3*ee_id], robot->d->xpos[3*ee_id+1], robot->d->xpos[3*ee_id+2]);
    }
    while(!letter_file.eof() and render_state) {
        if (!robot->paused || !viz) {   
            if (int_count >= int_max) {
                int_count = 0;
                int_pos_error = VectorXd::Zero(nq);
            }

            if (curr_step == step_count) { 
                std::getline(letter_file, line);    // Read next line of traj file 
                if (line.empty()) {
                    break;
                }
                // Extract desired state from string
                std::stringstream ss(line);
                // desired_state = Vector3d::Zero(6);
                // std::cout << "line: " << line << std::endl;
                for (int i = 0; i < 2; i++) {
                    std::getline(ss, col_value, ',');
                    // std::cout << "col value: " << col_value << std::endl;
                    desired_state(i) = std::max(-4.5, std::min(std::stod(col_value), 4.5));
                }
                desired_state(2) = 0.0;
                curr_step = 0;
                if (save_data) {
                    fprintf(fp, "%f, %f, %f\n", robot->d->xpos[3*ee_id], robot->d->xpos[3*ee_id+1], robot->d->xpos[3*ee_id+2]);
                    save_count += 1;
                    if (save_count % 100 == 0) {
                        printf("Saved %i states\n", save_count);
                        printf("sim time: %f\n", robot->d->time);
                    }
                }
            }
            
            VectorXd torque; 
            std::tie(torque, int_pos_error) = IK_joint_PID(robot, desired_state.head(3), int_pos_error, Kp, Ki, Kd);
            // std::cout << "torque\n" << torque << std::endl;
            mju_copy(robot->d->ctrl, torque.data(), robot->m->nu);
            mj_step(robot->m, robot->d);
           
            if (debug) {
                std::cout << "desired state:" << desired_state.format(CommaInitFmt) << std::endl;
                printf("ee pos: %f, %f, %f\n", robot->d->xpos[3*ee_id], robot->d->xpos[3*ee_id+1], robot->d->xpos[3*ee_id+2]);
                printf("sim time: %f\n", robot->d->time);
            }
            curr_step += 1;
            int_count += 1;
            curr_vis += 1;
            if (viz && curr_vis >= vis_count) {
                render_state = mjr_render(robot);
                curr_vis = 0;
            }
        } else {
            if (viz) {
                render_state = mjr_render(robot);
            }
        }
        count += 1;
    }
    printf("sim time: %f\n", robot->d->time);

    return 1;

}

int letter_test_inv_dyn(mjr_t* robot, int argc, const char** argv) {
    if (argc < 4) {
        printf("Error: requires 4 arguments of Kp, Ki, Kv gains and step count.\n");
        return 0;
    }
    double Kp = std::stod(argv[1]);
    double Ki = std::stod(argv[2]);
    double Kv = std::stod(argv[3]);
    // Good gains seem to be 0.05 0.0 0.5 with step count of 40
    int step_count = std::stoi(argv[4]);     // How many steps to take before updating desired state
    bool debug = false;
    bool viz = false;
    bool save_data = true;
    FILE *fp = fopen("./robot_test_inv_dyn.csv", "w+");

    int nq = robot->m->nq;
    int ee_id = mj_name2id(robot->m, mjOBJ_BODY, "EE");     // Get end-effector body id in mujoco model

    // for (int i = 0; i < nq; i++) {
    //     robot->d->qpos[i] = 1.5;
    // }
    robot->d->qpos[0] = 1.57;
    robot->d->qpos[1] = 1.41;
    robot->d->qpos[2] = 1.26;
    robot->d->qpos[3] = 1.11;
    robot->d->qpos[4] = 1.48;
    mj_forward(robot->m, robot->d);
    
    bool render_state = true;
    if (viz) {
        render_state = mjr_render(robot);
    } else{
        mjr_close_render(robot);
    }
    VectorXd int_pos_error = VectorXd::Zero(3);

    
    std::ifstream letter_file("./trajectory_gen/robot_traj_resize.csv");
    if (!letter_file.is_open()) throw std::runtime_error("Could not open file");

    std::string line, col_value;
    
    // Give robot total_t to reach first desired state
    float total_t = 4.0;
    std::getline(letter_file, line);    // Read next line of traj file 
    // Extract desired state from string
    std::stringstream init_ss(line);
    VectorXd desired_state = VectorXd::Zero(9);
    for (int i = 0; i < 2; i++) {
        std::getline(init_ss, col_value, ',');
        desired_state(i) = std::stod(col_value);
    }
    desired_state(2) = 0.0;
    std::cout << "desired state:" << desired_state.format(CommaInitFmt) << std::endl;
    int int_count = 0;
    int int_max = 20;
    int save_count = 0;
    int vis_count = 100;
    int curr_vis = 0;
    
    // while(robot->d->time < total_t and render_state) {
    //     if (!robot->paused || !viz) {        
    //         if (int_count >= int_max) {
    //             int_count = 0;
    //             int_pos_error = VectorXd::Zero(nq);
    //         }
            
    //         VectorXd torque; 
    //         std::tie(torque, int_pos_error) = IK_joint_PID(robot, desired_state.head(3), int_pos_error, Kp, Ki, Kd);
    //         // std::cout << "torque\n" << torque << std::endl;
    //         mju_copy(robot->d->ctrl, torque.data(), robot->m->nu);
    //         mj_step(robot->m, robot->d);

    //         if (debug) {
    //             std::cout << "desired state:" << desired_state.format(CommaInitFmt) << std::endl;
    //             printf("ee pos: %f, %f, %f\n", robot->d->xpos[3*ee_id], robot->d->xpos[3*ee_id+1], robot->d->xpos[3*ee_id+2]);
    //         }
    //         int_count += 1;
    //         curr_vis += 1;
            
    //     } 
    //     if (viz) {
    //         render_state = mjr_render(robot);
    //     }
        
    // }

    int_pos_error = VectorXd::Zero(3);
    
    int curr_step = 0;
    int count = 0;
    // Kp += 20;
    // Ki = 0.1;
    // Kv -= 2;
    int_max = 100;
    curr_vis = 0;
    if (save_data) {
        fprintf(fp, "%f, %f, %f\n", robot->d->xpos[3*ee_id], robot->d->xpos[3*ee_id+1], robot->d->xpos[3*ee_id+2]);
    }
    while(!letter_file.eof() and render_state) {
        if (!robot->paused || !viz) {   
            if (int_count >= int_max) {
                int_count = 0;
                int_pos_error = VectorXd::Zero(3);
            }

            if (curr_step == step_count) { 
                std::getline(letter_file, line);    // Read next line of traj file 
                if (line.empty()) {
                    break;
                }
                // Extract desired state from string
                std::stringstream ss(line);
                // desired_state = Vector3d::Zero(6);
                // std::cout << "line: " << line << std::endl;
                for (int i = 0; i < 2; i++) {
                    std::getline(ss, col_value, ',');
                    // std::cout << "col value: " << col_value << std::endl;
                    desired_state(i) = std::max(-4.5, std::min(std::stod(col_value), 4.5));
                }
                desired_state(2) = 0.0;
                curr_step = 0;
                if (save_data) {
                    fprintf(fp, "%f, %f, %f\n", robot->d->xpos[3*ee_id], robot->d->xpos[3*ee_id+1], robot->d->xpos[3*ee_id+2]);
                    save_count += 1;
                    if (save_count % 100 == 0) {
                        printf("Saved %i states\n", save_count);
                        printf("sim time: %f\n", robot->d->time);
                    }
                }
            }
            
            VectorXd torque;
            std::tie(torque, int_pos_error) = task_inv_dyn(robot, desired_state, int_pos_error, Kp, Ki, Kv);
            mju_copy(robot->d->ctrl, torque.data(), robot->m->nu);
            mj_step(robot->m, robot->d);
           
            if (debug) {
                std::cout << "desired state:" << desired_state.format(CommaInitFmt) << std::endl;
                printf("ee pos: %f, %f, %f\n", robot->d->xpos[3*ee_id], robot->d->xpos[3*ee_id+1], robot->d->xpos[3*ee_id+2]);
                printf("sim time: %f\n", robot->d->time);
            }
            curr_step += 1;
            int_count += 1;
            curr_vis += 1;
            if (viz && curr_vis >= vis_count) {
                render_state = mjr_render(robot);
                curr_vis = 0;
            }
        } else {
            if (viz) {
                render_state = mjr_render(robot);
            }
        }
        count += 1;
    }
    printf("sim time: %f\n", robot->d->time);

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
    mjr_t* robot = mjr_init("./model/plane_arm_big_waypoints.xml");

    // test_joint_PID_single(robot, argc, argv);
    // test_joint_PID_all(robot, argc, argv);
    // inv_kin_iter_test(robot, argc, argv);
    // test_taskspace_inv_kin(robot, argc, argv);
    // test_taskspace_inv_dyn(robot, argc, argv);
    // test_taskspace_accel(robot, argc, argv);
    // letter_test(robot, argc, argv);
    // letter_test_IK(robot, argc, argv);
    letter_test_inv_dyn(robot, argc, argv);
    mjr_free(robot);
 
    return 1;
}