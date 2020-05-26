#include "mujoco.h"
#include "ur5.h"

// main function
int main(int argc, const char** argv)
{

    // activate software
    const char* key_buf = getenv("MUJOCO_KEY_PATH");
    mj_activate(key_buf);

    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    ur5_t* robot = ur5_init();

    bool render_state = ur5_render(robot);
    // run main loop, target real-time simulation and 60 fps rendering
    while( render_state ) {
        
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        if (!robot->paused) {
            mjtNum simstart = robot->d->time;
            while( robot->d->time - simstart < 1.0/60.0 )
                mj_step(robot->m, robot->d);
        }

        render_state = ur5_render(robot);
    }

    ur5_free(robot);

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}