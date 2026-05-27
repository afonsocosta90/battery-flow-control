#include "btm/solver/gradient_descent.hpp"

#include <algorithm>
#include <limits>

namespace btm::solver {

GradientDescentSolver::GradientDescentSolver(int max_iterations,
                                             double step_size,
                                             double tol,
                                             double fd_epsilon)
    : max_iterations_(max_iterations),
      step_size_(step_size),
      tol_(tol),
      fd_epsilon_(fd_epsilon) {}

Solution GradientDescentSolver::solve(const Problem& problem,
                                      const std::vector<core::MassFlowRate>& initial_guess) const {
    Solution sol;
    sol.u_opt = initial_guess;
    problem.project(sol.u_opt);

    double cost = problem.cost(sol.u_opt);
    sol.final_cost = cost;

    const int N = static_cast<int>(sol.u_opt.size());

    for (int iter = 0; iter < max_iterations_; ++iter) {
        // Finite-difference gradient: central differences on each input
        std::vector<double> grad(N, 0.0);
        for (int i = 0; i < N; ++i) {
            auto u_plus  = sol.u_opt;
            auto u_minus = sol.u_opt;

            u_plus[i].value  += fd_epsilon_;
            u_minus[i].value -= fd_epsilon_;

            // Project to keep perturbations feasible
            problem.project(u_plus);
            problem.project(u_minus);

            const double c_plus  = problem.cost(u_plus);
            const double c_minus = problem.cost(u_minus);

            grad[i] = (c_plus - c_minus) / (2.0 * fd_epsilon_);
        }

        // Projected gradient descent step
        auto u_new = sol.u_opt;
        for (int i = 0; i < N; ++i)
            u_new[i].value -= step_size_ * grad[i];
        problem.project(u_new);

        const double new_cost = problem.cost(u_new);

        // Accept if the step reduced the cost (no line search — fixed step size)
        if (new_cost < cost) {
            const double improvement = cost - new_cost;
            cost       = new_cost;
            sol.u_opt  = u_new;
            sol.final_cost = cost;
            sol.iterations = iter + 1;

            // Converged: cost improvement below absolute tolerance
            if (improvement < tol_) {
                sol.converged = true;
                break;
            }
        } else {
            // Cost did not decrease; gradient is not making progress (possibly at boundary
            // or step size is too large for this region).  Accept and continue; the
            // iteration cap provides the hard safety budget.
            break;
        }
    }

    return sol;
}

} // namespace btm::solver
