// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <thread>
#include <optional>
#include <sstream>
#include <chrono>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;


// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  // backspace: reset simulation
  if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
    mj_resetData(m, d);
    mj_forward(m, d);
  } else if (act==GLFW_PRESS && key==GLFW_KEY_ESCAPE){
      glfwSetWindowShouldClose(window, GLFW_TRUE);
  }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  // update button state
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}


// main function
int main(int argc, const char** argv) {
  // check command-line arguments
  if (argc!=2) {
    std::printf(" USAGE:  basic modelfile\n");
    return EXIT_FAILURE;
  }

  // load and compile model
  char error[1000] = "Could not load binary model";
  if (std::strlen(argv[1])>4 && !std::strcmp(argv[1]+std::strlen(argv[1])-4, ".mjb")) {
    m = mj_loadModel(argv[1], 0);
  } else {
    m = mj_loadXML(argv[1], 0, error, 1000);
  }
  if (!m) {
    mju_error("Load model error: %s", error);
  }

  // set timestep
  const int sim_step_ms = 1;
  m->opt.timestep = sim_step_ms / 1000.0;
  
  // make data
  d = mj_makeData(m);

  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  // create window, request v-sync
  GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  // make OpenGL context current (open) in order to use the OpenGL API
  glfwMakeContextCurrent(window);
  // How many displayed screen frames to wait before updating/swapping the
  // buffers on a call to glfwSwapBuffers(). Setting this to zero will make
  // buffer updates occur immediately, leading to frame tearing (buffer update
  // while display is not complete). This acts on the current context.
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);  

  const int control_step_ms = 10;
  if (control_step_ms % sim_step_ms != 0){
      mju_error("control step is not a multiple of the sim step");
  }
  // display framerate
  const int vis_fps = 50;
  const int frame_step_ms = 1.0 / vis_fps * 1000;

  std::optional<std::chrono::time_point<std::chrono::steady_clock>> prev_now;
  std::optional<double> prev_vis_sim_time;
  const auto try_disp_frame = [&](const double sim_time){
      // wait until desired sim duration has passed before attempting to display
      if (prev_vis_sim_time){
          const int sim_dur_since_vis_ms = (sim_time - *prev_vis_sim_time)*1000;
          if (sim_dur_since_vis_ms < frame_step_ms){
              return;
          }
      }
      prev_vis_sim_time = sim_time;
      
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);
    
    if (prev_now) {
        // wait to reach target display frame rate
        std::this_thread::sleep_until(*prev_now + std::chrono::milliseconds(frame_step_ms));

        // measure and display frame rate
        std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();
        std::chrono::duration<double> dur_s = now - *prev_now;

        const double fps = 1 / dur_s.count();
        std::ostringstream os;
        os << fps;
        mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport,
                    "FPS", os.str().c_str(), &con);
    }
    prev_now = std::chrono::steady_clock::now();

    // swap OpenGL buffers (blocking call due to v-sync)
    // GLFW windows use double buffering. One buffer for display and the second
    // for rendering. After the render frame is update, then it should be
    // swapped with the display buffer to actually display it.
    glfwSwapBuffers(window);
  };
  
  // run main loop, target real-time simulation and 60 fps rendering
  while (!glfwWindowShouldClose(window)) {
    // todo: get next control input
    
    // simulate over a control step
    for (int i{}; i<control_step_ms/sim_step_ms; ++i){
        try_disp_frame(d->time);
        mj_step(m, d);
    }
    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }

  //free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);

  // free MuJoCo model and data
  mj_deleteData(d);
  mj_deleteModel(m);

  glfwDestroyWindow(window);

  // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif

  return EXIT_SUCCESS;
}
