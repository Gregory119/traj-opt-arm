#pragma once

#include <string>

#include "traj_element.hpp"

// save state trajectory data to a csv file
void saveDiscreteJointStateTrajCsv(const std::string &filename,
                                   const DiscreteJointStateTraj &sample_traj);

// used to save control trajectory data to a csv file
void saveDiscreteJointDataTrajCsv(const std::string &filename,
                                  const DiscreteJointDataTraj &sample_traj);
