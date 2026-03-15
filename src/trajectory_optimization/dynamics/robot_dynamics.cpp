#include "robot_dynamics.hpp"

namespace pin = pinocchio;

Eigen::VectorXd dyn(const Eigen::VectorXd &state,
                    const Eigen::VectorXd &control,
                    const double /*time*/,
                    const pin::Model &model)
{
    // data required by algorithm
    pin::Data data(model);

    // state = x = [q, dq] = [q1, q2, dq1, dq2]
    assert(state.size() == 4);
    // control = [u]
    assert(control.size() == 1);

    // Map control inputs to torque. This assumes control elements are in order
    // of joint torques, up to the size of the control vector.
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);
    tau(Eigen::seqN(0, control.size())) = control;

    // Get the joint configuration.
    const auto q = state(Eigen::seqN(0, state.size() / 2));
    // Get the generalized joint velocity.
    const auto v = state(Eigen::seqN(state.size() / 2, state.size() / 2));

    // calculate the forward dynamics = [dq, ddq]
    pin::aba(model, data, q, v, tau);
    Eigen::VectorXd dx = Eigen::VectorXd::Zero(2 * model.nv);
    dx << v, data.ddq;  // concatenate
    // std::cout << "state = \n" << state << std::endl;
    // std::cout << "control = \n" << control << std::endl;
    // std::cout << "tau = \n" << tau << std::endl;
    // std::cout << "q = \n" << q << std::endl;
    // std::cout << "v = \n" << v << std::endl;
    // std::cout << "data.ddq = \n" << data.ddq << std::endl;
    return dx;
}

Jacobian jacDynWrtState(const Eigen::VectorXd &state,
                        const Eigen::VectorXd &control,
                        const double /*time*/,
                        const pin::Model &model)
{
    // data required by algorithm
    pin::Data data(model);

    // state = x = [q, dq] = [q1, q2, dq1, dq2]
    assert(state.size() == 4);
    // control = [u]
    assert(control.size() == 1);

    // Map control inputs to torque. This assumes control elements are in order
    // of joint torques, up to the size of the control vector.
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);
    tau(Eigen::seqN(0, control.size())) = control;

    // Get the joint configuration. Note that pinocchio has an additional
    // universe joint at index 0.
    const auto q = state(Eigen::seqN(0, state.size() / 2));
    // Get the generalized joint velocity.
    const auto v = state(Eigen::seqN(state.size() / 2, state.size() / 2));

    // calculate the partial derivative of generalized joint acceleration w.r.t
    // the generalized joint configuration, joint velocity, and joint torque
    Eigen::MatrixXd ddq_dq = Eigen::MatrixXd::Zero(model.nv, model.nv);
    Eigen::MatrixXd ddq_dv = Eigen::MatrixXd::Zero(model.nv, model.nv);
    Eigen::MatrixXd ddq_dtau = Eigen::MatrixXd::Zero(model.nv, model.nv);
    pin::computeABADerivatives(model,
                               data,
                               q,
                               v,
                               tau,
                               ddq_dq,
                               ddq_dv,
                               ddq_dtau);

    /*
      Jacobian of the forward dynamics function f w.r.t the state x=[q v] is:
      df/dx = [df/dq df/dv] =
      [dv/dq dv/dv;
       da/dq da/dv] =
      [0 I;
       da/dq da/dv]
      where v = dq/dt, a = dv / dt

      Note that the zero submatrix and identity submatrix each have
      size=(state_len/2 x state_len/2)
     */

    const int state_len = state.size();
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(state_len / 2 + state_len / 2 * state_len);
    // fill in dv/dv=I
    for (int i{}; i < state_len / 2; ++i) {
        for (int j{}; j < state_len / 2; ++j) {
            if (i == j) {
                triplets.push_back({i, j + state_len / 2, 1.0});
            }
        }
    }
    // fill in da/dq
    for (int i{}; i < ddq_dq.rows(); ++i) {
        for (int j{}; j < ddq_dq.cols(); ++j) {
            triplets.push_back({i + state_len / 2, j, ddq_dq(i, j)});
        }
    }
    // fill in da/dv
    for (int i{}; i < ddq_dv.rows(); ++i) {
        for (int j{}; j < ddq_dv.cols(); ++j) {
            triplets.push_back(
                {i + state_len / 2, j + state_len / 2, ddq_dv(i, j)});
        }
    }
    Jacobian jac(state_len, state_len);
    jac.setFromTriplets(triplets.cbegin(), triplets.cend());
    return jac;
}

// todo: currently pin::computeABADerivatives() gets called twice for the same
// computation in jacDynWrtState() and jacDynWrtControl(). This
// is redundant because pin::computeABADerivatives() calculates the derivates
// required for the jacobian w.r.t the state and w.r.t the control. An
// optimization would be to refactor the code so that
// pin::computeABADerivatives() only gets called once between these two
// functions.
Jacobian jacDynWrtControl(const Eigen::VectorXd &state,
                          const Eigen::VectorXd &control,
                          const double /*time*/,
                          const pin::Model &model)
{
    // data required by algorithm
    pin::Data data(model);

    // state = x = [q, dq] = [q1, q2, dq1, dq2]
    assert(state.size() == 4);
    // control = [u]
    assert(control.size() == 1);

    // Map control inputs to torque. This assumes control elements are in order
    // of joint torques, up to the size of the control vector.
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(model.nv);
    tau(Eigen::seqN(0, control.size())) = control;

    // Get the joint configuration. Note that pinocchio has an additional
    // universe joint at index 0.
    const auto q = state(Eigen::seqN(0, state.size() / 2));
    // Get the generalized joint velocity.
    const auto v = state(Eigen::seqN(state.size() / 2, state.size() / 2));

    // calculate the partial derivative of generalized joint acceleration w.r.t
    // the generalized joint configuration, joint velocity, and joint torque
    Eigen::MatrixXd ddq_dq = Eigen::MatrixXd::Zero(model.nv, model.nv);
    Eigen::MatrixXd ddq_dv = Eigen::MatrixXd::Zero(model.nv, model.nv);
    Eigen::MatrixXd ddq_dtau = Eigen::MatrixXd::Zero(model.nv, model.nv);
    pin::computeABADerivatives(model,
                               data,
                               q,
                               v,
                               tau,
                               ddq_dq,
                               ddq_dv,
                               ddq_dtau);

    /*
      Jacobian of the forward dynamics function f w.r.t the control u=[tau(0)]
      is:
      df/du = df/dtau(0) =
      [dv/dtau(0);
       da/dtau(0)] =
      [0;
       da/dtau(0)]
      where v = dq/dt, a = dv / dt
     */

    const int state_len = state.size();
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(state_len / 2);
    // fill da/dtau(0), which is the first column of da/dtau, where tau is the
    // generalized joint vector.
    // std::cout << "ddq_dtau = \n" << ddq_dtau << std::endl;
    for (int i{}; i < state_len / 2; ++i) {
        triplets.push_back({i + state_len / 2, 0, ddq_dtau(i, 0)});
    }
    Jacobian jac(state_len, 1);
    jac.setFromTriplets(triplets.cbegin(), triplets.cend());
    return jac;
}
