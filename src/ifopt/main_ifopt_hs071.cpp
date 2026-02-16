#include "hs071_variables.hpp"
#include "hs071_constraints.hpp"
#include "hs071_cost.hpp"

#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>

#include <IpIpoptApplication.hpp>

#include <iostream>


int main(int /*argc*/, char ** /*argv*/)
{
    // define problem
    ifopt::Problem nlp;
    nlp.AddVariableSet(std::make_shared<HS071Variables>());
    nlp.AddConstraintSet(std::make_shared<HS071Constraints>());
    nlp.AddCostSet(std::make_shared<HS071Cost>());
    nlp.PrintCurrent();

    // choose solver and options
    ifopt::IpoptSolver ipopt;
    ipopt.SetOption("tol", 3.82e-6);
    ipopt.SetOption("mu_strategy", "adaptive");
    ipopt.SetOption("output_file", "ipopt.out");

    // solve
    ipopt.Solve(nlp);
    Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
    std::cout << x.transpose() << std::endl;

    return 0;
}

