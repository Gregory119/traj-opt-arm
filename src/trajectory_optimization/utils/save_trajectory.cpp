#include "save_trajectory.hpp"

#include <rapidcsv.h>

void saveDiscreteJointStateTrajCsv(const std::string &filename,
                                   const DiscreteJointStateTraj &sample_traj)
{
    // time | q(0) | ... | q(n-1) | dq(0) | ... | dq(n-1) | ddq(0) | ... |
    // ddq(n-1)
    rapidcsv::Document doc{};

    // create header
    doc.InsertColumn(0, std::vector<double>(), "time");
    std::array<std::string, 3> prefixes = {"q", "dq", "ddq"};
    for (int i{}; i < prefixes.size(); ++i) {
        for (int j{}; j < sample_traj[0].q.size(); ++j) {
            std::ostringstream os;
            os << prefixes[i] << j;
            doc.InsertColumn(i * sample_traj[0].q.size() + j + 1,
                             std::vector<double>(),
                             os.str());
        }
    }

    // fill in data in rows
    for (int i{}; i < sample_traj.size(); ++i) {
        std::vector<double> row_data;
        const JointState &e = sample_traj.at(i);
        row_data.push_back(e.time);
        row_data.insert(row_data.cend(), e.q.cbegin(), e.q.cend());
        row_data.insert(row_data.cend(), e.dq.cbegin(), e.dq.cend());
        row_data.insert(row_data.cend(), e.ddq.cbegin(), e.ddq.cend());
        doc.InsertRow(i, row_data);
    }
    doc.Save(filename);
}

void saveDiscreteJointDataTrajCsv(const std::string &filename,
                                  const DiscreteJointDataTraj &sample_traj)
{
    // time | data(0) | data(1) | ... | data(n-1)
    rapidcsv::Document doc{};

    // create header
    doc.InsertColumn(0, std::vector<double>(), "time");
    for (int i{}; i < sample_traj[0].data.size(); ++i) {
        std::ostringstream os;
        os << "data" << i;
        doc.InsertColumn(i + 1, std::vector<double>(), os.str());
    }

    // fill in data in rows
    for (int i{}; i < sample_traj.size(); ++i) {
        std::vector<double> row_data;
        const JointData &e = sample_traj.at(i);
        row_data.push_back(e.time);
        row_data.insert(row_data.cend(), e.data.cbegin(), e.data.cend());
        doc.InsertRow(i, row_data);
    }
    doc.Save(filename);
}

void saveColBoundsCsv(const std::string &filename,
                      const ifopt::Component::VecBound &bounds,
                      const int n,
                      const double start_time,
                      const double dur)
{
    assert(bounds.size() % n == 0);

    // time | x(0)_low | x(0)_high | ... | x(n-1)_low | x(n-1)_high |
    rapidcsv::Document doc{};

    // create header
    doc.InsertColumn(0, std::vector<double>(), "time");
    for (int i{}; i < n; ++i) {
        std::ostringstream os;
        os << "x" << i << "_low";
        doc.InsertColumn(i * 2 + 1, std::vector<double>(), os.str());
        os.str("");
        os << "x" << i << "_high";
        doc.InsertColumn(i * 2 + 2, std::vector<double>(), os.str());
    }

    // fill in data in rows
    const int num_col_pts = bounds.size() / n;
    for (int i{}; i < num_col_pts; ++i) {
        std::vector<double> row_data;
        // insert time
        const double alpha = static_cast<double>(i) / (num_col_pts-1);
        const double t = alpha * dur + start_time;
        row_data.push_back(t);
        // insert bounds
        for (int j{}; j < n; ++j) {
            row_data.push_back(bounds[i * n + j].lower_);
            row_data.push_back(bounds[i * n + j].upper_);
        }
        doc.InsertRow(i, row_data);
    }
    doc.Save(filename);
}
