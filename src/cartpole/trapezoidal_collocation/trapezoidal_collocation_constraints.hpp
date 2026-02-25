#pragma once

#include <ifopt/constraint_set.h>

class TrapezoidalCollocationConstraints final : public ifopt::ConstraintSet
{
public:
    using DynFn = std::function<Eigen::VectorXd(const Eigen::VectorXd &state,
                                                const Eigen::VectorXd &control,
                                                const double time)>;

    TrapezoidalCollocationConstraints(const int num_constraints,
                                      const std::string &state_var_set_name,
                                      const int state_len,
                                      const std::string &control_var_set_name,
                                      const int control_len,
                                      const double segment_dt,
                                      const DynFn &dyn_fn)
        : ConstraintSet(num_constraints, "trap_col_constraints")
        , m_state_var_set_name{state_var_set_name}
        , m_state_len{state_len}
        , m_control_var_set_name{control_var_set_name}
        , m_control_len{control_len}
        , m_segment_dt{segment_dt}
        , m_dyn_fn{dyn_fn}
    {
        const Eigen::VectorXd state_vars
            = GetVariables()->GetComponent(m_state_var_set_name)->GetValues();
        const int num_knot_pts = state_var.size() / m_state_len;
        const int num_segments = num_knot_pts - 1;
        m_num_constraints = num_segments * m_state_len;
    }

    Eigen::VectorXd GetValues() const override
    {
        const Eigen::VectorXd state_vars
            = GetVariables()->GetComponent(m_state_var_set_name)->GetValues();
        const Eigen::VectorXd control_vars
            = GetVariables()->GetComponent(m_control_var_set_name)->GetValues();

        assert(state_vars.size() % m_state_len == 0);
        assert(control_vars.size() % m_control_len == 0);
        assert(control_vars.size() / m_control_len
               == state_vars.size() / m_state_len);

        // fill in defect constraint values
        auto defect_constraints = Eigen::VectorXd::Zero(m_num_constraints);

        // make sure that k+1 can be accessed for the last iteration
        const int k_max = num_segments;
        for (int k{}; k < k_max; ++k) {
            // get state and control k
            auto state_view_k
                = state_vars(Eigen::seqN(k * m_state_len, m_state_len));
            auto control_view_k
                = control_vars(Eigen::seqN(k * m_control_len, m_control_len));
            // time relative to start time of zero
            const double tk = k * m_segment_dt;
            const Eigen::VectorXd fk
                = m_dyn_fn(state_view_k, control_view_k, tk);
            assert(fk.size() == m_state_len);

            // get state and control k+1
            auto state_view_k1
                = state_vars(Eigen::seqN((k + 1) * m_state_len, m_state_len));
            auto control_view_k1 = control_vars(
                Eigen::seqN((k + 1) * m_control_len, m_control_len));
            // time relative to start time of zero
            const double tk1 = (k + 1) * m_segment_dt;
            const Eigen::VectorXd fk1
                = m_dyn_fn(state_view_k1, control_view_k1, tk1);
            assert(fk1.size() == m_state_len);

            const Eigen::VectorXd defectk = state_view_k1 - state_view_k
                                            - m_segment_dt / 2.0 * (fk + fk1);
            defect_constraints(Eigen::seqN(k * m_state_len, m_state_len))
                = defectk;
        }

        return defect_constraints;
    }

    ifopt::Component::VecBound GetBounds() const override
    {
        // defects should all be zero
        ifopt::Component::VecBound bounds(m_num_constraints, {0.0, 0.0});
        return bounds;
    }

    void FillJacobianBlock(std::string var_set,
                           Jacobian &jac_block) const override
    {
        if (var_set == "var_set") {
            const Eigen::VectorXd x
                = GetVariables()->GetComponent("var_set")->GetValues();

            // At this point the nonzero elements of the jacobian have to be set
            // (they don't exist yet). Fill in along the major order - ifopt
            // uses row major order for the jacobian sparse matrix by default so
            // fill row by row. This could be optimized by reserving memory for
            // each major axis (see
            // https://libeigen.gitlab.io/eigen/docs-nightly/group__TutorialSparse.html).
            jac_block.insert(0, 0) = x[1] * x[2] * x[3];
            jac_block.insert(0, 1) = x[0] * x[2] * x[3];
            jac_block.insert(0, 2) = x[0] * x[1] * x[3];
            jac_block.insert(0, 3) = x[0] * x[1] * x[2];
            jac_block.insert(1, 0) = 2 * x[0];
            jac_block.insert(1, 1) = 2 * x[1];
            jac_block.insert(1, 2) = 2 * x[2];
            jac_block.insert(1, 3) = 2 * x[3];
        }
    }

private:
    const std::string m_state_var_set_name;
    const int m_state_len;
    const std::string m_control_var_set_name;
    const int m_control_len;
    const double m_segment_dt;
    const DynFn m_dyn_fn;
    int m_num_constraints;
    Eigen::Vector4d m_x;
};
