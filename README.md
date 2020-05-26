# ur5-writing

A project to produce handwritten characters with a UR5 manipulator robot.

## Installation and Setup

This project requires [MuJoCo](http://www.mujoco.org/index.html) which can be downloaded [here](https://www.roboti.us/index.html). A license file for MuJoCo (mjkey.txt) is required to run the simulation. A trial or student license can be obtained for free [here](https://www.roboti.us/license.html). Once you have MuJoCo downloaded and a valid `mjkey.txt`, it is suggested that you place both in a hidden `.mujoco` directory in your home directory. You must then set the `MUJOCO_KEY_PATH` environment variable to be where your MuJoCo license key is located (i.e. `~/.mujoco/mjkey.txt`). This can be done by adding `export MUJOCO_KEY_PATH = <path to mjkey.txt>` to your `.bashrc`.
You should then be able to compile and run simulation by running `make sim`

## Visualization Features
The F1 key will display a help screen that lists the most useful of the following functions.
* Mouse Interactivity:
  * The camera can be rotated by holding left mouse button and can be moved by holding the right mouse button.
  * Zoom the camera in and out using the scroll wheel.
  * To have the camera track a certain body, double control right-click on the desired body. To go back to free camera mode press the esc key.
  * Double left-click to select a body to apply a force on.
  * Once a body is selected, control right-click and drag to apply a force in a direction. The magnitude of the force is proportional to the distance from the cursor the body.
  * Control left-click allows you to apply a rotational force.
* Key Commands:
  * Simulation can be paused and restarted using the spacebar key.
  * The backspace key resets the model to the initial neutral qpos (all joint angles are zero).
  * When paused, the model can be simulated step by step using the right arrow key. To step forward in increments of 100, use the down arrow key. The left and up arrow keys similar step the simulation backwards by 1 and 100 steps, though this can be unstable especially if too many backward steps are taken in a row.
  * The enter key will enable/disable slowmotion, which will make the simulation run 10x slower than normal.
  * The equal ('=') and minus ('-') keys can be used to increase and decrease the font size of the info screen respectively.
  * Ctrl+a will recenter the camera to the center of the platform
  * Ctrl+p will print the current qpos to the terminal
  * Ctrl+q will close the visualization window.
  * Toggle visualization flags:
    * 'j': Highlight the joint locations
    * 'u': Highlight the actuator locations
    * 'n': Highlight the constraint locations
    * 'c': Highlight points of contact
    * 'f': Show contact force arrows
    * 't': Make the model transparent
    * 'm': Show the center of mass of the entire model
  * F3 will hide/show the info window in the bottom left corner that displays simulation time and whether the simulation is running or paused.
