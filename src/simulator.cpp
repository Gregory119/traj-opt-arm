#include "simulator.hpp"

#include <iostream>

bool Simulator::button_left = false;
bool Simulator::button_middle = false;
bool Simulator::button_right = false;
double Simulator::lastx = 0;
double Simulator::lasty = 0;

std::shared_ptr<Simulator> Simulator::getInstance()
{
    static std::shared_ptr<Simulator> sim(new Simulator("../model/so101.xml"));
    return sim;
}

Simulator::Simulator(const std::string &model_path,
                     const int control_step_ms,
                     const int vis_fps,
                     const int sim_step_ms)
    : m_control_step_ms{control_step_ms}
    , m_frame_step_ms{static_cast<int>(1.0 / vis_fps * 1000.0)}
    , m_sim_step_ms{sim_step_ms}
    , m_vis_timer{PeriodicSimTimer(m_frame_step_ms / 1000.0,
                                   [this](PeriodicSimTimer &) { dispFrame(); })}
    // frame rate timer
    , m_control_timer{PeriodicSimTimer(
          m_control_step_ms / 1000.0,
          [this](PeriodicSimTimer &) { updateControl(); },
          false  // disable
          )}     // control timer
    , m_start_control_timer{PeriodicSimTimer(
          1.0,  // delay before control starts
          [this](PeriodicSimTimer &this_timer) {
              // std::cout << "start control sim time: " << m_data->time <<
              // std::endl;
              m_control_timer.reset(true);
              this_timer.reset(false);  // make this timer only trigger once
          })}
    , m_sim_timers{{&m_vis_timer, &m_control_timer, &m_start_control_timer}}
{
    if (m_control_step_ms % m_sim_step_ms != 0) {
        mju_error("trajectory sample step is not a multiple of the sim step");
    }

    // load and compile model
    char error [1000] = "Could not load binary model";
    // load mjb file
    if ((model_path.size() > 4)
        && (model_path.substr(model_path.size() - 4) == ".mjb")) {
        m_model = mj_loadModel(model_path.c_str(), 0);
    } else {
        // load other file
        m_model = mj_loadXML(model_path.c_str(), 0, error, 1000);
    }
    if (!m_model) {
        mju_error("Load model error: %s", error);
    }

    // set timestep
    m_model->opt.timestep = m_sim_step_ms / 1000.0;

    // make data
    m_data = mj_makeData(m_model);

    initVis();
}

Simulator::~Simulator()
{
    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data
    mj_deleteData(m_data);
    mj_deleteModel(m_model);

    glfwDestroyWindow(m_window);

    // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif
}

void Simulator::run()
{
    while (!glfwWindowShouldClose(m_window)) {
        // update timers
        for (PeriodicSimTimer *timer : m_sim_timers) {
            timer->update(m_data->time);
        }

        // step simulation
        mj_step(m_model, m_data);
    }
}

void Simulator::initVis()
{
    // init GLFW
    if (!glfwInit()) {
        mju_error("Could not initialize GLFW");
    }

    // create window, request v-sync
    m_window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    // make OpenGL context current (open) in order to use the OpenGL API
    glfwMakeContextCurrent(m_window);
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
    mjv_makeScene(m_model, &scn, 2000);
    mjr_makeContext(m_model, &con, mjFONTSCALE_150);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(m_window, keyboard);
    glfwSetCursorPosCallback(m_window, mouse_move);
    glfwSetMouseButtonCallback(m_window, mouse_button);
    glfwSetScrollCallback(m_window, scroll);
}

// keyboard callback
void Simulator::keyboard(GLFWwindow *window,
                         int key,
                         int scancode,
                         int act,
                         int mods)
{
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE) {
        Simulator::getInstance()->reset();
    } else if (act == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
}

void Simulator::reset()
{
    mj_resetData(m_model, m_data);
    mj_forward(m_model, m_data);
    prev_now.reset();
    for (PeriodicSimTimer *timer : m_sim_timers) {
        timer->reset();
    }
}

// mouse button callback
void Simulator::mouse_button(GLFWwindow *window, int button, int act, int mods)
{
    // update button state
    button_left
        = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle
        = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right
        = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void Simulator::mouse_move(GLFWwindow *window, double xpos, double ypos)
{
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
    bool mod_shift
        = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS
           || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

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
    mjv_moveCamera(Simulator::getInstance()->m_model,
                   action,
                   dx / height,
                   dy / height,
                   &(Simulator::getInstance()->scn),
                   &(Simulator::getInstance()->cam));
}

// scroll callback
void Simulator::scroll(GLFWwindow *window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(Simulator::getInstance()->m_model,
                   mjMOUSE_ZOOM,
                   0,
                   -0.05 * yoffset,
                   &(Simulator::getInstance()->scn),
                   &(Simulator::getInstance()->cam));
}

void Simulator::dispFrame()
{
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(m_window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m_model, m_data, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    if (prev_now) {
        // wait to reach target display frame rate
        std::this_thread::sleep_until(
            *prev_now + std::chrono::milliseconds(m_frame_step_ms));

        // measure and display frame rate
        std::chrono::time_point<std::chrono::steady_clock> now
            = std::chrono::steady_clock::now();
        std::chrono::duration<double> dur_s = now - *prev_now;

        const double fps = 1 / dur_s.count();
        std::ostringstream os;
        os << fps;
        mjr_overlay(mjFONT_NORMAL,
                    mjGRID_BOTTOMLEFT,
                    viewport,
                    "FPS",
                    os.str().c_str(),
                    &con);
    }
    prev_now = std::chrono::steady_clock::now();

    // swap OpenGL buffers (blocking call due to v-sync)
    // GLFW windows use double buffering. One buffer for display and the second
    // for rendering. After the render frame is update, then it should be
    // swapped with the display buffer to actually display it.
    glfwSwapBuffers(m_window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
}

void Simulator::updateControl()
{
    // std::cout << "updateControl()" << std::endl;
    //  todo: update trajectory sample
    m_data->ctrl [0] = 1.0;
}
