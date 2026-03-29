#include "simulator.hpp"

#include <iostream>
#include <save_trajectory.hpp>

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
                     const int sim_step_ms,
                     const std::string &record_filename_csv)
    : m_control_step_ms{control_step_ms}
    , m_frame_step_ms{static_cast<int>(1.0 / vis_fps * 1000.0)}
    , m_sim_step_ms{sim_step_ms}
    , m_timers{{// display frame rate timer
                {TimerId::Display,
                 PeriodicSimTimer(
                     m_frame_step_ms / 1000.0,
                     [this](PeriodicSimTimer &, const double /*time*/) {
                         dispFrame();
                     })},
                // control timer
                {TimerId::Control,
                 PeriodicSimTimer(
                     m_control_step_ms / 1000.0,
                     [this](PeriodicSimTimer &, const double /*time*/) {
                         updateControl();
                     })},
                // record data timer
                {TimerId::Record,
                 PeriodicSimTimer(
                     m_sim_step_ms / 1000.0,
                     [this](PeriodicSimTimer &, const double /*time*/) {
                         record();
                     })}}}
    , m_record_filename_csv{record_filename_csv}
{
    if (m_control_step_ms % m_sim_step_ms != 0) {
        mju_error("trajectory sample step is not a multiple of the sim step");
    }

    // load and compile model
    char error[1000] = "Could not load binary model";
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
    mjv_freeScene(&m_scn);
    mjr_freeContext(&m_con);

    // free MuJoCo model and data
    mj_deleteData(m_data);
    mj_deleteModel(m_model);

    glfwDestroyWindow(m_window);

    // terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif
}

void Simulator::setTrajectory(DiscreteJointStateTraj target_traj)
{
    // todo: validate size of state
    m_target_traj_orig = std::move(target_traj);
    m_target_traj = m_target_traj_orig;
}

void Simulator::setCsvRecordFileName(const std::string& name)
{
    m_record_filename_csv = name;
}

void Simulator::run()
{
    while (!glfwWindowShouldClose(m_window)) {
        // update timers
        for (auto &[_, timer] : m_timers) {
            timer.update(m_data->time);
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
    mjv_defaultCamera(&m_cam);
    m_cam.distance = 1.5;
    mjv_defaultOption(&m_opt);
    mjv_defaultScene(&m_scn);
    mjr_defaultContext(&m_con);

    // create scene and context
    mjv_makeScene(m_model, &m_scn, 2000);
    mjr_makeContext(m_model, &m_con, mjFONTSCALE_150);

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
    for (auto &[_, timer] : m_timers) {
        timer.reset();
    }
    m_target_traj = m_target_traj_orig;
    m_traj_record.clear();
    m_timers.find(TimerId::Record)->second.reset(true);
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
                   &(Simulator::getInstance()->m_scn),
                   &(Simulator::getInstance()->m_cam));
}

// scroll callback
void Simulator::scroll(GLFWwindow *window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(Simulator::getInstance()->m_model,
                   mjMOUSE_ZOOM,
                   0,
                   -0.05 * yoffset,
                   &(Simulator::getInstance()->m_scn),
                   &(Simulator::getInstance()->m_cam));
}

void Simulator::dispFrame()
{
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(m_window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m_model, m_data, &m_opt, NULL, &m_cam, mjCAT_ALL, &m_scn);
    mjr_render(viewport, &m_scn, &m_con);

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
                    &m_con);
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
    // std::cout << "updateControl() time: " << m_data->time << std::endl;

    // get the last control input up to the current time
    std::optional<JointState> e;
    while (!m_target_traj.empty()
           && (m_target_traj.front().time < m_data->time)) {
        e = m_target_traj.front();
        m_target_traj.pop_front();
    }
    if (!e) {
        // Useable control not found. This can occur if sim time has not past
        // the first trajectory element. This can also occur if the trajectory
        // has not been set.
        return;
    }
    // Put the found trajectory element back on the front of the queue in case
    // it needs to be used for the next update.  This means that once the time
    // has past through the entire trajectory, the last element will always be
    // used for the control input.
    m_target_traj.push_front(*e);

    for (size_t i{}; i < e->q.size(); ++i) {
        m_data->ctrl[i] = e->q(i);
    }
}

void Simulator::record()
{
    Eigen::VectorXd q = Eigen::VectorXd::Zero(m_model->nq);
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(m_model->nv);
    Eigen::VectorXd ddq = Eigen::VectorXd::Zero(m_model->nv);

    assert(m_model->nq == m_model->nv);
    for (int i{}; i < m_model->nq; ++i) {
        q[i] = m_data->qpos[i];
        dq[i] = m_data->qvel[i];
        ddq[i] = m_data->qacc[i];
    }
    m_traj_record.push_back(JointState{.time = m_data->time,
                                       .q = std::move(q),
                                       .dq = std::move(dq),
                                       .ddq = std::move(ddq)});

    // save the recording if the end of the target trajectory is reached
    if ((m_target_traj.size() == 1) && !m_traj_record.empty()) {
        std::cout << "saving recording" << std::endl;
        saveDiscreteJointStateTrajCsv(m_record_filename_csv, m_traj_record);
        // stop recording by disabling the record timer
        m_traj_record.clear();
        m_timers.find(TimerId::Record)->second.reset(false);
    }
}
