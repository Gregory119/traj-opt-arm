#pragma once

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <thread>
#include <traj_element.hpp>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include "periodic_sim_timer.hpp"

class Simulator final
{
public:
    static std::shared_ptr<Simulator> getInstance();

    ~Simulator();

    void setTrajectory(DiscreteJointStateTraj target_traj);

    void run();

private:
    Simulator(const std::string &model_path,
              const int control_step_ms = 10,
              const int vis_fps = 50,
              const int sim_step_ms = 1,
              const std::string &record_filename_csv = "sim-state-traj.csv");

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
    void record();

    const int m_control_step_ms;
    const int m_frame_step_ms;
    const int m_sim_step_ms;

    // MuJoCo data structures
    mjModel *m_model = nullptr;  // MuJoCo model
    mjData *m_data = nullptr;    // MuJoCo data

    // Mujoco visualization data structures
    mjvCamera m_cam;   // abstract camera
    mjvOption m_opt;   // visualization options
    mjvScene m_scn;    // abstract scene
    mjrContext m_con;  // custom GPU context

    // GLFW
    GLFWwindow *m_window = nullptr;

    // mouse interaction
    static bool button_left;
    static bool button_middle;
    static bool button_right;
    static double lastx;
    static double lasty;

    enum class TimerId
    {
        Display,
        Control,
        Record
    };

    std::map<TimerId, PeriodicSimTimer>
        m_timers;
    std::optional<std::chrono::time_point<std::chrono::steady_clock>> prev_now;

    // keep a copy of the original trajectory to enable resetting the simulation
    DiscreteJointStateTraj m_target_traj_orig;
    // target trajectory that gets popped during sim
    DiscreteJointStateTraj m_target_traj;
    // recorded actual trajectory
    DiscreteJointStateTraj m_traj_record;

    const std::string m_record_filename_csv;
};
