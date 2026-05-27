#include "btm/solver/gradient_descent.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

using namespace btm::solver;
using namespace btm::core;

// ============================================================================
// Helper — build a simple unconstrained or box-constrained quadratic problem.
//
// cost(u) = Σ_i w_i · (u_i − u_star_i)²
//
// Minimum is at u* = u_star_i, clipped to [lo, hi] when bounds are present.
// ============================================================================

struct QuadraticProblem {
    std::vector<double> u_star;   // unconstrained minimum
    std::vector<double> weights;
    double lo;
    double hi;
};

static Problem make_quadratic(const QuadraticProblem& q) {
    Problem p;
    p.cost = [q](const std::vector<MassFlowRate>& u) -> double {
        double c = 0.0;
        for (std::size_t i = 0; i < u.size(); ++i) {
            const double e = u[i].value - q.u_star[i];
            c += q.weights[i] * e * e;
        }
        return c;
    };
    p.project = [q](std::vector<MassFlowRate>& u) {
        for (auto& x : u) {
            if (x.value < q.lo) x.value = q.lo;
            if (x.value > q.hi) x.value = q.hi;
        }
    };
    return p;
}

// ============================================================================
// Tests
// ============================================================================

// Non-negotiable: solver converges to the known optimum on an unconstrained
// quadratic (bounds are wide enough that the optimum is interior).
TEST(MpcSolver, ConvergesToInteriorMinimum) {
    QuadraticProblem q;
    q.u_star  = {1.0, 0.5, -0.3};
    q.weights = {2.0, 1.0, 3.0};
    q.lo = -10.0;
    q.hi = 10.0;

    GradientDescentSolver solver(200, 0.05, 1e-6, 1e-4);
    auto prob = make_quadratic(q);

    std::vector<MassFlowRate> init = {MassFlowRate{0.0}, MassFlowRate{0.0}, MassFlowRate{0.0}};
    auto sol = solver.solve(prob, init);

    for (std::size_t i = 0; i < q.u_star.size(); ++i) {
        EXPECT_NEAR(sol.u_opt[i].value, q.u_star[i], 0.05)
            << "Component " << i << " did not converge to known optimum";
    }
    EXPECT_LT(sol.final_cost, 1e-3) << "Final cost should be near zero at minimum";
}

// Solver must respect hard input bounds via projection.
TEST(MpcSolver, RespectsBoxConstraints) {
    QuadraticProblem q;
    q.u_star  = {3.0, -1.0};   // unconstrained optima outside [0, 2]
    q.weights = {1.0, 1.0};
    q.lo = 0.0;
    q.hi = 2.0;

    GradientDescentSolver solver(100, 0.1, 1e-6, 1e-4);
    auto prob = make_quadratic(q);

    std::vector<MassFlowRate> init = {MassFlowRate{1.0}, MassFlowRate{1.0}};
    auto sol = solver.solve(prob, init);

    // With u_star outside bounds, the constrained optimum is at the boundary.
    EXPECT_NEAR(sol.u_opt[0].value, 2.0, 0.05) << "u[0] should be clipped to upper bound";
    EXPECT_NEAR(sol.u_opt[1].value, 0.0, 0.05) << "u[1] should be clipped to lower bound";

    for (const auto& u : sol.u_opt) {
        EXPECT_GE(u.value, q.lo - 1e-9);
        EXPECT_LE(u.value, q.hi + 1e-9);
    }
}

// Warm-start: starting closer to the optimum should require fewer iterations.
TEST(MpcSolver, WarmStartReducesIterations) {
    QuadraticProblem q;
    q.u_star  = {1.5, 0.5};
    q.weights = {1.0, 1.0};
    q.lo = 0.0;
    q.hi = 2.0;

    GradientDescentSolver solver(500, 0.05, 1e-6, 1e-4);
    auto prob = make_quadratic(q);

    // Cold start: far from optimum
    std::vector<MassFlowRate> cold = {MassFlowRate{0.0}, MassFlowRate{0.0}};
    auto sol_cold = solver.solve(prob, cold);

    // Warm start: close to optimum
    std::vector<MassFlowRate> warm = {MassFlowRate{1.4}, MassFlowRate{0.6}};
    auto sol_warm = solver.solve(prob, warm);

    EXPECT_LE(sol_warm.iterations, sol_cold.iterations)
        << "Warm start should not require more iterations than cold start";
}

// Scalar 1D case: u* = 0.7 within [0, 1].
TEST(MpcSolver, ScalarConvergence) {
    QuadraticProblem q;
    q.u_star  = {0.7};
    q.weights = {5.0};
    q.lo = 0.0;
    q.hi = 1.0;

    GradientDescentSolver solver(300, 0.02, 1e-7, 1e-4);
    auto prob = make_quadratic(q);

    std::vector<MassFlowRate> init = {MassFlowRate{0.0}};
    auto sol = solver.solve(prob, init);

    EXPECT_NEAR(sol.u_opt[0].value, 0.7, 0.02);
}

// All components equal: diagonal quadratic, single minimum, well-conditioned.
TEST(MpcSolver, LargerDimension) {
    constexpr int N = 10;
    QuadraticProblem q;
    q.u_star.resize(N);
    q.weights.resize(N, 1.0);
    for (int i = 0; i < N; ++i) q.u_star[i] = 0.5 + 0.05 * i;
    q.lo = 0.0;
    q.hi = 1.5;

    GradientDescentSolver solver(500, 0.05, 1e-6, 1e-4);
    auto prob = make_quadratic(q);

    std::vector<MassFlowRate> init(N, MassFlowRate{0.0});
    auto sol = solver.solve(prob, init);

    for (int i = 0; i < N; ++i) {
        EXPECT_NEAR(sol.u_opt[i].value, q.u_star[i], 0.1)
            << "Component " << i << " failed for N=10 case";
    }
}
