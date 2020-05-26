#include "mjxmacro.h"
#include "uitools.h"
#include "stdio.h"
#include "string.h"
#include "mujoco.h"
#include "glfw3.h"
#include "ur5.h"

#include <chrono>

// help strings
const char help_content[] = 
        "Alt mouse button\n"
        "UI right hold\n"
        "UI title double-click\n"
        "Space\n"
        "Esc\n"
        "Right arrow\n"
        "Left arrow\n"
        "Down arrow\n"
        "Up arrow\n"
        "Page Up\n"
        "Double-click\n"
        "Right double-click\n"
        "Ctrl Right double-click\n"
        "Scroll, middle drag\n"
        "Left drag\n"
        "[Shift] right drag\n"
        "Ctrl [Shift] drag\n"
        "Ctrl [Shift] right drag";

const char help_title[] = 
        "Swap left-right\n"
        "Show UI shortcuts\n"
        "Expand/collapse all  \n"
        "Pause\n"
        "Free camera\n"
        "Step forward\n"
        "Step back\n"
        "Step forward 100\n"
        "Step back 100\n"
        "Select parent\n"
        "Select\n"
        "Center\n"
        "Track camera\n"
        "Zoom\n"
        "View rotate\n"
        "View translate\n"
        "Object rotate\n"
        "Object translate";

static int fontscale = mjFONTSCALE_200;

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    ur5_t* r = (ur5_t*) glfwGetWindowUserPointer(window);

    if (act == GLFW_RELEASE) {
        return;
    } else if (act == GLFW_PRESS) {
        // control keys
        if (mods == GLFW_MOD_CONTROL) {
            if (key == GLFW_KEY_A) {
                memcpy(r->cam.lookat, r->m->stat.center, sizeof(r->cam.lookat));
                r->cam.distance = 1.5*r->m->stat.extent;
                // set to free camera
                r->cam.type = mjCAMERA_FREE;
            } else if (key == GLFW_KEY_P) {
                printf("qpos: ");
                for (int i = 0; i < r->m->nq; i++) {
                    printf("%f", r->d->qpos[i]);
                    if (i != r->m->nq-1) {
                        printf(", ");
                    }
                }
                printf("\n");
                // mju_printMat_fp(v->d->qpos, v->m->nq, 1);
            } else if (key == GLFW_KEY_Q) {
                glfwSetWindowShouldClose(window, true);
            }
        }
        // toggle visualiztion flag
        for (int i=0; i < mjNVISFLAG; i++) {
            if (key == mjVISSTRING[i][2][0]) {
                mjtByte flags[mjNVISFLAG];
                memcpy(flags, r->opt.flags, sizeof(flags));
                flags[i] = flags[i] == 0 ? 1 : 0;
                memcpy(r->opt.flags, flags, sizeof(r->opt.flags));
                return;
            }
        }
        // toggle rendering flag
        for (int i=0; i < mjNRNDFLAG; i++) {
            if (key == mjRNDSTRING[i][2][0]) {
                mjtByte flags[mjNRNDFLAG];
                memcpy(flags, r->scn.flags, sizeof(flags));
                flags[i] = flags[i] == 0 ? 1 : 0;
                memcpy(r->scn.flags, flags, sizeof(r->scn.flags));
                return;
            }
        }
        // toggle geom/site group
        for (int i=0; i < mjNGROUP; i++) {
            if (key == i + 48) {    // Int('0') = 48
                if (mods && GLFW_MOD_SHIFT == true) {
                    mjtByte sitegroup[mjNGROUP];
                    memcpy(sitegroup, r->opt.sitegroup, sizeof(sitegroup));
                    sitegroup[i] = sitegroup[i] > 0 ? 0 : 1;
                    // memcpy(v->opt.sitegroup = sitegroup
                    r->opt.sitegroup[i] = sitegroup[i];
                    return;
                } else {
                    mjtByte geomgroup[mjNGROUP];
                    memcpy(geomgroup, r->opt.geomgroup, sizeof(geomgroup));
                    geomgroup[i] = geomgroup[i] > 0 ? 0 : 1;
                    memcpy(r->opt.geomgroup, geomgroup, sizeof(r->opt.geomgroup));
                    return;
                }
            }
        }
        switch (key) {
            case GLFW_KEY_F1: {     // help
                r->showhelp += 1;
                if (r->showhelp > 1) {
                    r->showhelp = 0;
                }
            } break;
            case GLFW_KEY_F2: {     // option
                r->showoption = !r->showoption;
            } break;
            case GLFW_KEY_F3: {     // info
                r->showinfo = !r->showinfo;
            } break;
            case GLFW_KEY_F5: {     // toggle fullscreen
                r->showfullscreen = !r->showfullscreen;
                r->showfullscreen ? glfwMaximizeWindow(window) : glfwRestoreWindow(window);
            } break;
            // case GLFW_KEY_F7: {     // sensor figure
            //     v->showsensor = !v->showsensor;
            // } break;
            case GLFW_KEY_ENTER: {  // slow motion
                r->slowmotion = !r->slowmotion;
                // r->slowmotion ? printf("Slow Motion Mode!\n") : printf("Normal Speed Mode!\n");
            } break;
            case GLFW_KEY_SPACE: {  // pause
                r->paused = !r->paused;
                // r->paused ? printf("Paused\n") : printf("Running\n");
            } break;
            case GLFW_KEY_BACKSPACE: {  // reset
                mj_resetData(r->m, r->d);
                mj_forward(r->m, r->d);
            } break;
            case GLFW_KEY_RIGHT: {      // step forward
                if (r->paused) {
                    mj_step(r->m, r->d);
                }
            } break;
            case GLFW_KEY_LEFT: {       // step backward
                if (r->paused) {
                    double dt = r->m->opt.timestep;
                    r->m->opt.timestep = -dt;
                    mj_step(r->m, r->d);
                    r->m->opt.timestep = dt;
                }
            } break;
            case GLFW_KEY_DOWN: {      // step forward 100
                if (r->paused) {
                    for (int i = 0; i < 100; i++) {
                        mj_step(r->m, r->d);
                    }
                }
            } break;
            case GLFW_KEY_UP: {       // step back 100
                if (r->paused) {
                    double dt = r->m->opt.timestep;
                    r->m->opt.timestep = -dt;
                    for (int i = 0; i < 100; i++) {
                        mj_step(r->m, r->d);
                    }
                    r->m->opt.timestep = dt;
                }
            } break;
            case GLFW_KEY_ESCAPE: {     // free camera
                r->cam.type = mjCAMERA_FREE;
            } break;
            case GLFW_KEY_EQUAL: {      // bigger font
                if (fontscale < 200) {
                    fontscale += 50;
                    mjr_makeContext(r->m, &r->con, fontscale);
                }
            } break;
            case GLFW_KEY_MINUS: {      // smaller font
                if (fontscale > 100) {
                    fontscale -= 50;
                    mjr_makeContext(r->m, &r->con, fontscale);
                }
            } break;
            case GLFW_KEY_LEFT_BRACKET: {  // '[' previous fixed camera or free
                int fixedcam = r->cam.type;
                if (r->m->ncam > 0 && fixedcam == mjCAMERA_FIXED) {
                    int fixedcamid = r->cam.fixedcamid;
                    if (fixedcamid  > 0) {
                        r->cam.fixedcamid = fixedcamid-1;
                    } else {
                        r->cam.type = mjCAMERA_FREE;
                    }
                }
            } break;
            case GLFW_KEY_RIGHT_BRACKET: {  // ']' next fixed camera
                if (r->m->ncam > 0) {
                    int fixedcam = r->cam.type;
                    int fixedcamid = r->cam.fixedcamid;
                    if (fixedcam != mjCAMERA_FIXED) {
                        r->cam.type = mjCAMERA_FIXED;
                    } else if (fixedcamid < r->m->ncam - 1) {
                        r->cam.fixedcamid = fixedcamid+1;
                    }
                }
            } break;
        }
    }

}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    ur5_t* r = (ur5_t*) glfwGetWindowUserPointer(window);
    // update button state
    r->button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    r->button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    r->button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &r->lastx, &r->lasty);

    // set perturbation
    int newperturb = 0;
    if (mods == GLFW_MOD_CONTROL && r->pert.select > 0) {
        if (act == GLFW_PRESS) {
            // right: translate;  left: rotate
            if (r->button_right) {
                newperturb = mjPERT_TRANSLATE;
            } else if (r->button_left) {
                newperturb = mjPERT_ROTATE;
            }
            // perturbation onset: reset reference
            if (newperturb > 0 && r->pert.active == 0) {
                mjv_initPerturb(r->m, r->d, &r->scn, &r->pert);
            }
        }
    }
    r->pert.active = newperturb;

    // detect double-click (250 msec)
    time_t curr_time = time(0);
    if (act == GLFW_PRESS && (curr_time - r->lastclicktm < 0.25) && (button == r->lastbutton)) {
        // determine selection mode
        int selmode = 2;    // Right Click
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            selmode = 1;
        } else if (mods == GLFW_MOD_CONTROL) {
            selmode = 3; // CTRL + Right Click
        }
        // get current window size
        int width, height;
        glfwGetWindowSize(window, &width, &height);
        // find geom and 3D click point, get corresponding body
        mjtNum selpnt[3];

        int selgeom = 0;
        int selskin = 0;
        mjtNum aspectratio = (mjtNum) width / height;
        mjtNum relx = (mjtNum) r->lastx / width;
        mjtNum rely = (mjtNum) (height - r->lasty) / height;

        int selbody = mjv_select(r->m, r->d, &r->opt,
                            aspectratio, relx,
                            rely, 
                            &r->scn, selpnt, &selgeom, &selskin);
        // set lookat point, start tracking is requested
        if (selmode == 2 || selmode == 3) {
            // copy selpnt if geom clicked
            if (selbody >= 0) {
                memcpy(r->cam.lookat, selpnt, sizeof(r->cam.lookat));
            }

            // switch to tracking camera
            if (selmode == 3 && selbody >= 0) {
                r->cam.type = mjCAMERA_TRACKING;
                r->cam.trackbodyid = selbody;
                r->cam.fixedcamid = -1;
            }
        } else { // set body selection
            if (selbody >= 0) {
                // compute localpos
                mjtNum tmp[3];
                mju_sub3(tmp, selpnt, r->d->qpos+3*selbody);
                mju_mulMatTVec(r->pert.localpos, r->d->xmat+9*selbody, tmp, 3, 3);

                // record selection
                r->pert.select = selbody;
                r->pert.skinselect = selskin;
            } else {
                r->pert.select = 0;
                r->pert.skinselect = -1;
            }
        }

        // stop perturbation on select
        r->pert.active = 0;
    }
    // save info
    if (act == GLFW_PRESS) {
        r->lastbutton = button;
        r->lastclicktm = time(0);
    }

}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    ur5_t* r = (ur5_t*) glfwGetWindowUserPointer(window);
    // no buttons down: nothing to do
    if( !r->button_left && !r->button_middle && !r->button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - r->lastx;
    double dy = ypos - r->lasty;
    r->lastx = xpos;
    r->lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( r->button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( r->button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    // mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);

    // move perturb or camera
    mjtNum xchange = dx / height;
    mjtNum ychange = dy / height;
    if (r->pert.active != 0) {
        mjv_movePerturb(r->m, r->d, action, xchange, ychange, &r->scn, &r->pert);
    } else {
        mjv_moveCamera(r->m, action, xchange, ychange, &r->scn, &r->cam);
    }
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    ur5_t* r = (ur5_t*) glfwGetWindowUserPointer(window);
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(r->m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &r->scn, &r->cam);
}

ur5_t* ur5_init() {
    ur5_t *robot = (ur5_t*) calloc(1, sizeof (ur5_t));

    // load and compile model
    const char* modelfile = "./model/ur5.xml";
    char error[1000] = "Could not load binary model";
    mjModel *m = mj_loadXML(modelfile, 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);
    robot->m = mj_copyModel(NULL, m);

    // make data
    robot->d = mj_makeData(robot->m);

    robot->lastx = 0.0;
    robot->lasty = 0.0;
    robot->button_left = false;
    robot->button_middle = false;
    robot->button_right = false;
    robot->lastbutton = GLFW_MOUSE_BUTTON_1;
    robot->lastclicktm = 0.0;
    robot->refreshrate = glfwGetVideoMode(glfwGetPrimaryMonitor())->refreshRate;
    robot->showhelp = 0;
    robot->showoption = false;
    robot->showfullscreen = false;
    robot->showsensor = false;
    robot->slowmotion = false;
    robot->showinfo = true;
    robot->paused = true;
    robot->framenum = 0;
    robot->lastframenum = 0;

    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    robot->window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(robot->window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&robot->cam);
    mjv_defaultOption(&robot->opt);
    mjv_defaultScene(&robot->scn);
    mjr_defaultContext(&robot->con);

    // create scene and context
    mjv_makeScene(robot->m, &robot->scn, 2000);
    mjr_makeContext(robot->m, &robot->con, mjFONTSCALE_150);

    glfwSetWindowUserPointer(robot->window, robot);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(robot->window, keyboard);
    glfwSetCursorPosCallback(robot->window, mouse_move);
    glfwSetMouseButtonCallback(robot->window, mouse_button);
    glfwSetScrollCallback(robot->window, scroll);

    return robot;
}

void ur5_free(ur5_t* r) {
    // Free mujoco objects
    mjv_freeScene(&r->scn);
    mjr_freeContext(&r->con);
    mj_deleteData(r->d);
    mj_deleteModel(r->m);
    mj_deactivate();

    // Close window
    glfwDestroyWindow(r->window);

    free(r);

}

bool ur5_render(ur5_t *r) {
    // Return early if window is closed
    if (!r || !r->window)
        return false;

    // Check if window should be closed
    if (glfwWindowShouldClose(r->window)) {
        return false;
    }

    // clear old perturbations, apply new
    mju_zero(r->d->xfrc_applied, 6 * r->m->nbody);
    if (r->pert.select > 0) {
       mjv_applyPerturbPose(r->m, r->d, &r->pert, 0); // move mocap bodies only
       mjv_applyPerturbForce(r->m, r->d, &r->pert);
    }
    
    mj_forward(r->m, r->d);
    // Set up for rendering
    glfwMakeContextCurrent(r->window);
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(r->window, &viewport.width, &viewport.height);
    // mjrRect smallrect = viewport;
    // Render scene

    mjv_updateScene(r->m, r->d, &r->opt, &r->pert, &r->cam, mjCAT_ALL, &r->scn);
    mjr_render(viewport, &r->scn, &r->con);
    // if (r->showsensor) {
    //     if (!r->paused) {
    //         sensorupdate(v);
    //     }
    //     sensorshow(v, smallrect);
    // }
    if (r->showhelp) {
        mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, help_title, help_content, &r->con);
    }
    if (r->showinfo) {
        char buf[1024];
        char str_slow[20];
        if (r->slowmotion) {
            strcpy(str_slow, "(10x slowdown)");
        } else {
            strcpy(str_slow, "");
        }
        char str_paused[50];
        if(r->paused) {
            strcpy(str_paused, "\nPaused");
        } else {
            strcpy(str_paused, "\nRunning");
        }
        strcat(str_paused, "\nTime:");
        char status[50];
        sprintf(status, "\n\n%.2f", r->d->time);
        strcpy(buf, str_slow);
        strcat(buf, status);
        // status = str_slow * status

        mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport,
                    str_paused,
                    buf, &r->con);
    }
    // Show updated scene
    glfwSwapBuffers(r->window);
    glfwPollEvents();

    return true;
}