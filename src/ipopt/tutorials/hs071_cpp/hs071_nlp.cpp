#include "hs071_nlp.hpp"

#include <array>
#include <cassert>
#include <iostream>

bool HS071_NLP::get_nlp_info(Ipopt::Index &n,
                             Ipopt::Index &m,
                             Ipopt::Index &nnz_jac_g,
                             Ipopt::Index &nnz_h_lag,
                             Ipopt::TNLP::IndexStyleEnum &index_style)
{
    // number of decision variables
    n = 4;
    // the problem has two constraint equations in g(x)
    m = 2;
    // number of nonzero entries in the Jacobian (dense)
    nnz_jac_g = 8;
    // the Hessian is dense but it is symmetric so only the lower triangle is
    // needed
    nnz_h_lag = 10;
    // 0 index arrays
    index_style = C_STYLE;
    return true;
}

bool HS071_NLP::get_bounds_info(Ipopt::Index n,
                                Ipopt::Number *x_l,
                                Ipopt::Number *x_u,
                                Ipopt::Index m,
                                Ipopt::Number *g_l,
                                Ipopt::Number *g_u)
{
    assert(n == 4);
    assert(m = 2);

    // set lower and upper bounds of decision variables
    for (size_t i{}; i < n; ++i) {
        x_l[i] = 1.0;
        x_u[i] = 5.0;
    }
    // set upper and lower constraints
    // note that default +inf=1e19
    std::array<Ipopt::Number, 2> constraints_l = {25.0, 40.0};
    std::array<Ipopt::Number, 2> constraints_u = {2e19, 40.0};
    for (size_t i{}; i < m; ++i) {
        g_l[i] = constraints_l[i];
        g_u[i] = constraints_u[i];
    }
    return true;
}

bool HS071_NLP::get_starting_point(Ipopt::Index n,
                                   bool init_x,
                                   Ipopt::Number *x,
                                   bool init_z,
                                   Ipopt::Number *z_L,
                                   Ipopt::Number *z_U,
                                   Ipopt::Index m,
                                   bool init_lambda,
                                   Ipopt::Number *lambda)
{
    assert(n == 4);
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);
    std::array<Ipopt::Number, 4> x_vals = {1, 5, 5, 1};
    std::copy(x_vals.cbegin(), x_vals.cend(), x);
    return true;
}

bool HS071_NLP::eval_f(Ipopt::Index n,
                       const Ipopt::Number *x,
                       bool new_x,
                       Ipopt::Number &obj_value)
{
    obj_value = x[0] * x[3] * (x[0] + x[1] + x[2]) + x[2];
    return true;
}

bool HS071_NLP::eval_grad_f(Ipopt::Index n,
                            const Ipopt::Number *x,
                            bool new_x,
                            Ipopt::Number *grad_f)
{
    assert(n == 4);
    grad_f[0] = x[0] * x[3] + x[3] * (x[0] + x[1] + x[2]);
    grad_f[1] = x[0] * x[3];
    grad_f[2] = x[1] * x[3] + 1;
    grad_f[3] = x[0] * (x[0] + x[1] + x[2]);
    return true;
}

bool HS071_NLP::eval_g(Ipopt::Index n,
                       const Ipopt::Number *x,
                       bool new_x,
                       Ipopt::Index m,
                       Ipopt::Number *g)
{
    assert(m == 2);
    g[0] = x[0] * x[1] * x[2] * x[3];
    g[1] = x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3];
    return true;
}

bool HS071_NLP::eval_jac_g(Ipopt::Index n,
                           const Ipopt::Number *x,
                           bool new_x,
                           Ipopt::Index m,
                           Ipopt::Index nele_jac,
                           Ipopt::Index *iRow,
                           Ipopt::Index *jCol,
                           Ipopt::Number *values)
{
    if (values == nullptr) {
        iRow[0] = 0;
        jCol[0] = 0;
        iRow[1] = 0;
        jCol[1] = 1;
        iRow[2] = 0;
        jCol[2] = 2;
        iRow[3] = 0;
        jCol[3] = 3;
        iRow[4] = 1;
        jCol[4] = 0;
        iRow[5] = 1;
        jCol[5] = 1;
        iRow[6] = 1;
        jCol[6] = 2;
        iRow[7] = 1;
        jCol[7] = 3;
        return true;
    }
    values[0] = x[1] * x[2] * x[3];
    values[1] = x[0] * x[2] * x[3];
    values[2] = x[0] * x[1] * x[3];
    values[3] = x[0] * x[1] * x[2];
    values[4] = 2 * x[0];
    values[5] = 2 * x[1];
    values[6] = 2 * x[2];
    values[7] = 2 * x[3];
    return true;
}

bool HS071_NLP::eval_h(Ipopt::Index n,
                       const Ipopt::Number *x,
                       bool new_x,
                       Ipopt::Number obj_factor,
                       Ipopt::Index m,
                       const Ipopt::Number *lambda,
                       bool new_lambda,
                       Ipopt::Index nele_hess,
                       Ipopt::Index *iRow,
                       Ipopt::Index *jCol,
                       Ipopt::Number *values)
{
    assert(n == 4);
    assert(m == 2);

    if (values == NULL) {
        // return the structure. This is a symmetric matrix, fill the lower left
        // triangle only.

        // the hessian for this problem is actually dense
        Ipopt::Index idx = 0;
        for (Ipopt::Index row = 0; row < 4; row++) {
            for (Ipopt::Index col = 0; col <= row; col++) {
                iRow[idx] = row;
                jCol[idx] = col;
                idx++;
            }
        }

        assert(idx == nele_hess);
    } else {
        // return the values. This is a symmetric matrix, fill the lower left
        // triangle only

        // fill the objective portion
        values[0] = obj_factor * (2 * x[3]);  // 0,0

        values[1] = obj_factor * (x[3]);  // 1,0
        values[2] = 0.;                   // 1,1

        values[3] = obj_factor * (x[3]);  // 2,0
        values[4] = 0.;                   // 2,1
        values[5] = 0.;                   // 2,2

        values[6] = obj_factor * (2 * x[0] + x[1] + x[2]);  // 3,0
        values[7] = obj_factor * (x[0]);                    // 3,1
        values[8] = obj_factor * (x[0]);                    // 3,2
        values[9] = 0.;                                     // 3,3

        // add the portion for the first constraint
        values[1] += lambda[0] * (x[2] * x[3]);  // 1,0

        values[3] += lambda[0] * (x[1] * x[3]);  // 2,0
        values[4] += lambda[0] * (x[0] * x[3]);  // 2,1

        values[6] += lambda[0] * (x[1] * x[2]);  // 3,0
        values[7] += lambda[0] * (x[0] * x[2]);  // 3,1
        values[8] += lambda[0] * (x[0] * x[1]);  // 3,2

        // add the portion for the second constraint
        values[0] += lambda[1] * 2;  // 0,0

        values[2] += lambda[1] * 2;  // 1,1

        values[5] += lambda[1] * 2;  // 2,2

        values[9] += lambda[1] * 2;  // 3,3
    }

    return true;
}

void HS071_NLP::finalize_solution(Ipopt::SolverReturn status,
                                  Ipopt::Index n,
                                  const Ipopt::Number *x,
                                  const Ipopt::Number *z_L,
                                  const Ipopt::Number *z_U,
                                  Ipopt::Index m,
                                  const Ipopt::Number *g,
                                  const Ipopt::Number *lambda,
                                  Ipopt::Number obj_value,
                                  const Ipopt::IpoptData *ip_data,
                                  Ipopt::IpoptCalculatedQuantities *ip_cq)
{
    // here is where we would store the solution to variables, or write to a
    // file, etc so we could use the solution.

    // For this example, we write the solution to the console
    std::cout << std::endl
              << std::endl
              << "Solution of the primal variables, x" << std::endl;
    for (Ipopt::Index i = 0; i < n; i++) {
        std::cout << "x[" << i << "] = " << x[i] << std::endl;
    }

    std::cout << std::endl
              << std::endl
              << "Solution of the bound multipliers, z_L and z_U" << std::endl;
    for (Ipopt::Index i = 0; i < n; i++) {
        std::cout << "z_L[" << i << "] = " << z_L[i] << std::endl;
    }
    for (Ipopt::Index i = 0; i < n; i++) {
        std::cout << "z_U[" << i << "] = " << z_U[i] << std::endl;
    }

    std::cout << std::endl << std::endl << "Objective value" << std::endl;
    std::cout << "f(x*) = " << obj_value << std::endl;

    std::cout << std::endl << "Final value of the constraints:" << std::endl;
    for (Ipopt::Index i = 0; i < m; i++) {
        std::cout << "g(" << i << ") = " << g[i] << std::endl;
    }
}
