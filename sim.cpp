#include "mujoco.h"
// #include "ur5.h"
#include "mj_robot.h"
#include "stdio.h"
#include "string.h"
#include "mjxmacro.h"
#include "uitools.h"
#include <iostream>

// main function
int main(int argc, const char** argv)
{

    // activate software
    const char* key_buf = getenv("MUJOCO_KEY_PATH");
    mj_activate(key_buf);

    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // ur5_t* robot = ur5_init();
    mjr_t* robot = mjr_init("./model/plane_arm_big_waypoints.xml");
    int ee_id = mj_name2id(robot->m, mjOBJ_BODY, "EE");     // Get end-effector body id in mujoco model

    // bool render_state = ur5_render(robot);
    bool render_state = mjr_render(robot);
    double render_time = 60.0;
    // run main loop, target real-time simulation and 60 fps rendering
    while( render_state == 1) {
        
        if (robot->slowmotion) {
            render_time = 600.0;
        } else {
            render_time = 60.0;
        }
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        if (!robot->paused) {
            mjtNum simstart = robot->d->time;
            while( robot->d->time - simstart < 1.0/render_time ) {
                mj_step(robot->m, robot->d);
            }
            printf("ee pos: %f, %f, %f\n", robot->d->xpos[3*ee_id], robot->d->xpos[3*ee_id+1], robot->d->xpos[3*ee_id+2]);
        }

        // render_state = ur5_render(robot);
        render_state = mjr_render(robot);
    }

    // ur5_free(robot);
    mjr_free(robot);

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}