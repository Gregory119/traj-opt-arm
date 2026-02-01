#pragma once

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <memory>
#include <optional>
#include <sstream>
#include <thread>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "periodic_sim_timer.hpp"

struct TrajElement
{
    double time;
    std::vector<double> val;
};

class Simulator final
{
public:
    static std::shared_ptr<Simulator> getInstance();

    ~Simulator();

    void setTrajectory(std::deque<TrajElement> ctrl_traj);

    void run();

private:
    Simulator(const std::string &model_path,
              const int control_step_ms = 10,
              const int vis_fps = 50,
              const int sim_step_ms = 1);

    void reset();

    // initialize visualization
    void initVis();

    // UI callbacks
    static void keyboard(GLFWwindow *window,
                         int key,
                         int scancode,
                         int act,
                         int mods);
    static void mouse_button(GLFWwindow *window, int button, int act, int mods);
    static void mouse_move(GLFWwindow *window, double xpos, double ypos);
    static void scroll(GLFWwindow *window, double xoffset, double yoffset);

    void dispFrame();

    void updateControl();

    const int m_control_step_ms;
    const int m_frame_step_ms;
    const int m_sim_step_ms;

    // MuJoCo data structures
    mjModel *m_model = nullptr;  // MuJoCo model
    mjData *m_data = nullptr;    // MuJoCo data

    // Mujoco visualization data structures
    mjvCamera cam;   // abstract camera
    mjvOption opt;   // visualization options
    mjvScene scn;    // abstract scene
    mjrContext con;  // custom GPU context

    // GLFW
    GLFWwindow *m_window = nullptr;

    // mouse interaction
    static bool button_left;
    static bool button_middle;
    static bool button_right;
    static double lastx;
    static double lasty;

    // frame rate timer
    PeriodicSimTimer m_vis_timer;
    // control sample timer
    PeriodicSimTimer m_control_timer;
    const std::vector<PeriodicSimTimer *> m_sim_timers;
    std::optional<std::chrono::time_point<std::chrono::steady_clock>> prev_now;

    std::deque<TrajElement> m_ctrl_traj;
};
