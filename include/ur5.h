#include "mjxmacro.h"
#include "uitools.h"
#include "stdio.h"
#include "string.h"
#include "mujoco.h"
#include "glfw3.h"

struct ur5 {
    // MuJoCo stuff
    mjModel *m;
    mjData *d;
    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;
    mjvPerturb pert;
    mjvFigure figsensor;
    mjvFigure figGRF;

     //visual interaction controls
    double lastx;
    double lasty;
    bool button_left;
    bool button_middle;
    bool button_right;

    int lastbutton;
    double lastclicktm;

    int refreshrate;

    int showhelp;
    bool showoption;
    bool showfullscreen;
    bool showsensor;
    bool slowmotion;

    bool showinfo;
    bool paused;

    int framenum;
    int lastframenum;
    
    // GLFW  handle
    GLFWwindow *window;
};

typedef struct ur5 ur5_t;

ur5_t *ur5_init();
void ur5_free(ur5_t* r);

bool ur5_render(ur5_t *r);

// GLFW Callbacks
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
void mouse_button(GLFWwindow* window, int button, int act, int mods);
void mouse_move(GLFWwindow* window, double xpos, double ypos);
void scroll(GLFWwindow* window, double xoffset, double yoffset);

