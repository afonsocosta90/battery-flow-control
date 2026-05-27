#pragma once

#include "btm/core/types.hpp"

#include <functional>
#include <vector>

namespace btm::solver {

struct Problem {
    // Cost function: (input_sequence) -> cost
    std::function<double(const std::vector<core::MassFlowRate>&)> cost;

    // Projection onto feasible set (e.g. [min, max] for each input)
    std::function<void(std::vector<core::MassFlowRate>&)> project;

    // Finite-difference gradient callback (optional, can be computed internally)
    // For now we compute finite differences inside the solver.
};

struct Solution {
    std::vector<core::MassFlowRate> u_opt;   // optimal input sequence
    double final_cost{0.0};
    int iterations{0};
    bool converged{false};
};

class GradientDescentSolver {
public:
    explicit GradientDescentSolver(int max_iterations,
                                   double step_size,
                                   double tol,
                                   double fd_epsilon);

    Solution solve(const Problem& problem,
                   const std::vector<core::MassFlowRate>& initial_guess) const;

private:
    int max_iterations_;
    double step_size_;
    double tol_;
    double fd_epsilon_;
};

} // namespace btm::solver
