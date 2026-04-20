#pragma once

#include <string>

#include <ifopt/composite.h>

#include "traj_element.hpp"

// save state trajectory data to a csv file
void saveDiscreteJointStateTrajCsv(const std::string &filename,
                                   const DiscreteJointStateTraj &sample_traj);

// used to save control trajectory data to a csv file
void saveDiscreteJointDataTrajCsv(const std::string &filename,
                                  const DiscreteJointDataTraj &sample_traj);

/* Save the discrete bounds on a collocation trajectory used for solving the
 * optimization problem.
 *
 * @param n Length of the vector value being bound (eg. For state bounds this is
 * the length of the state vector).
 * @param start_time Start time of the trajectory.
 * @param dur Trajectory duration.
 */
void saveColBoundsCsv(const std::string &filename,
                      const ifopt::Component::VecBound &bounds,
                      const int n,
                      const double start_time,
                      const double dur);
