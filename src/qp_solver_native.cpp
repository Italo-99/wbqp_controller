#include "wbqp_controller/qp_solver_native.hpp"
#include <algorithm>
#include <cstdio>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace wbqp
{
    namespace
    {
        bool isFiniteSparse(const Eigen::SparseMatrix<double>& M)
        {
            for (int c = 0; c < M.outerSize(); ++c)
            {
                for (Eigen::SparseMatrix<double>::InnerIterator it(M, c); it; ++it)
                {
                    if (!std::isfinite(it.value()))
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        Eigen::Matrix<double,6,6> taskWeightMatrix(const NativeQpInput& in)
        {
            Eigen::Matrix<double,6,6> m = Eigen::Matrix<double,6,6>::Zero();
            for (int i = 0; i < 3; ++i) { m(i,i) = in.w_lin; }
            for (int i = 3; i < 6; ++i) { m(i,i) = in.w_ang; }
            return m;
        }

        Eigen::Matrix<double,9,9> regularizationContribution(const NativeQpInput& in)
        {
            Eigen::Matrix<double,9,9> R = Eigen::Matrix<double,9,9>::Zero();
            for (int i = 0; i < 6; ++i) { R(i,i) = in.beta_arm; }
            R(6,6) = in.alpha_xy;
            R(7,7) = in.alpha_xy;
            R(8,8) = in.alpha_yaw;
            return 2.0 * (R + in.nu * Eigen::Matrix<double,9,9>::Identity());
        }

        Eigen::Matrix<double,9,1> baseLowerBounds(const NativeQpInput& in)
        {
            Eigen::Matrix<double,9,1> v;
            v << -in.max_dotq, -in.max_dotq, -in.max_dotq,
                 -in.max_dotq, -in.max_dotq, -in.max_dotq,
                 -in.max_V, -in.max_V, -in.max_Omegaz;
            return v;
        }

        Eigen::Matrix<double,9,1> baseUpperBounds(const NativeQpInput& in)
        {
            Eigen::Matrix<double,9,1> v;
            v << in.max_dotq, in.max_dotq, in.max_dotq,
                 in.max_dotq, in.max_dotq, in.max_dotq,
                 in.max_V, in.max_V, in.max_Omegaz;
            return v;
        }
    } // namespace

    Eigen::Matrix<double,6,9> NativeQpSolver::mapJ(const std::array<double,54>& Jc)
    {
        Eigen::Matrix<double,6,9> J;
        for (int j=0;j<9;++j) for (int r=0;r<6;++r) J(r,j) = Jc[r + 6*j];
        return J;
    }

    void NativeQpSolver::buildTask(const Eigen::Matrix<double,6,9>& J,
                                   const Eigen::Matrix<double,6,1>& u,
                                   const Eigen::Matrix<double,9,1>& dqprev,
                                   const NativeQpInput& in,
                                   Eigen::Matrix<double,9,9>& H,
                                   Eigen::Matrix<double,9,1>& f)
    {
        const Eigen::Matrix<double,6,6> Wt = taskWeightMatrix(in);

        // 0.5 z^T H z + f^T z with 2x scaling to match LS normal equations convention
        H = 2.0 * (J.transpose() * Wt * J);
        f = -2.0 * (J.transpose() * Wt * u + in.nu * dqprev);
    }

    void NativeQpSolver::buildReg(Eigen::Matrix<double,9,9>& H,
                                  const NativeQpInput& in)
    {
        H += regularizationContribution(in);
    }

    void NativeQpSolver::buildVarBounds(const NativeQpInput& in,
                                        Eigen::Matrix<double,9,1> &lb,
                                        Eigen::Matrix<double,9,1> &ub)
    {
        lb = baseLowerBounds(in);
        ub = baseUpperBounds(in);

        // Feasibility margin used to repair transient inconsistent bounds.
        const double m = 2.0 * in.max_dotq * in.dt;

        // Joints: velocity box AND position window this step: q + dq*dt ∈ [qmin,qmax]
        for (int i = 0; i < 6; ++i) {
            const double lb_pos = (in.qmin[i] - in.q[i]) / in.dt;
            const double ub_pos = (in.qmax[i] - in.q[i]) / in.dt;

            double lb_i = std::max(lb[i], lb_pos);
            double ub_i = std::min(ub[i], ub_pos);

            // Repair inconsistent bounds for this cycle:
            // 1) preferred direction: shift lower bound from upper by margin
            // 2) reversed fallback if still inconsistent
            // 3) final collapse to a feasible point in the velocity box
            if (lb_i > ub_i) {
                lb_i = ub_i - m;
            }
            if (lb_i > ub_i) {
                ub_i = lb_i + m;
            }
            if (lb_i > ub_i) {
                const double v_mid = 0.5 * (lb_i + ub_i);
                const double v_sat = std::max(baseLowerBounds(in)[i], std::min(baseUpperBounds(in)[i], v_mid));
                lb_i = v_sat;
                ub_i = v_sat;
            }

            lb[i] = lb_i;
            ub[i] = ub_i;
        }
    }

    void NativeQpSolver::buildAccelConstraints(const NativeQpInput& in,
                                               Eigen::SparseMatrix<double>& A,
                                               Eigen::VectorXd &lA, Eigen::VectorXd &uA)
    {
        // Rate box: |z - zprev| ≤ a_max * dt  -> [-I; I] z ≤ [zprev + d; -zprev + d]
        const int rows = 18; // 2 per var
        A.resize(rows, 9);
        std::vector<Eigen::Triplet<double>> trips; trips.reserve(2*9);
        for (int i=0;i<9;++i) {
            trips.emplace_back(i,   i, -1.0); // -I
            trips.emplace_back(i+9, i,  1.0); //  I
        }
        A.setFromTriplets(trips.begin(), trips.end());

        lA = Eigen::VectorXd::Constant(rows, -1e20); // -inf
        uA = Eigen::VectorXd(rows);

        auto dcap = [&](int idx)->double{
            if (idx<6) return in.qddot_max * in.dt;
            if (idx<8) return in.a_lin_max * in.dt; // 6,7
            return in.alpha_max * in.dt;            // 8
        };

        for (int i=0;i<9;++i) {
            const double d = dcap(i);
            uA[i]   = -in.dotq_prev[i] + d; // (-I)z ≤ -zprev + d
            uA[i+9] =  in.dotq_prev[i] + d; // ( I)z ≤  zprev + d
        }
    }

    bool NativeQpSolver::solveOsqp(const Eigen::SparseMatrix<double>& Hs,
                                   const Eigen::Matrix<double,9,1>& f_in,
                                   const Eigen::SparseMatrix<double>& Aall,
                                   const Eigen::VectorXd& l_in,
                                   const Eigen::VectorXd& u_in,
                                   const Eigen::Matrix<double,9,1>& xwarm_in,
                                   NativeQpOutput &out)
    {
        OsqpEigen::Solver solver;
        solver.settings()->setWarmStart(true);
        solver.settings()->setVerbosity(false);
        solver.settings()->setAlpha(1.6);
        solver.settings()->setPolish(true);
        solver.settings()->setTimeLimit(0.01);

        // ---- critical: local, non-const, dynamic vectors ----
        Eigen::VectorXd f  = f_in;   // dynamic copy (non-const)
        Eigen::VectorXd lb = l_in;   // non-const
        Eigen::VectorXd ub = u_in;   // non-const

        solver.data()->setNumberOfVariables(9);
        solver.data()->setNumberOfConstraints(static_cast<int>(Aall.rows()));
        solver.data()->setHessianMatrix(Hs);
        solver.data()->setGradient(f);                    // binds to Ref<VectorXd>
        solver.data()->setLinearConstraintsMatrix(Aall);
        solver.data()->setLowerBound(lb);                 // binds to Ref<VectorXd>
        solver.data()->setUpperBound(ub);                 // binds to Ref<VectorXd>

        if (!solver.initSolver()) return false;

        // --- Warm start with previous velocity (project to box bounds to be safe) ---
        Eigen::VectorXd x0 = xwarm_in;
        for (int i = 0; i < 9; ++i) {
            // project onto simple variable bounds if present in u/l blocks
            // ub segment at rows [r0, r0+9) holds upper bounds; lb isn't explicit
            // but we have box bounds separately in the caller: clamp with +/-inf guard
            double lo = -std::numeric_limits<double>::infinity();
            double hi =  std::numeric_limits<double>::infinity();
            // If you keep explicit lb/ub arrays around, use them here instead:
            // lo = lb_vars[i]; hi = ub_vars[i];
            // For now just guard against NaNs.
            if (!std::isfinite(x0[i])) x0[i] = 0.0;
            // (Optional) clamp to known joint speed limits if you have them cached
        }
        solver.setPrimalVariable(x0);   // only primal is fine; dual not required. :contentReference[oaicite:1]{index=1}

        const auto status = solver.solveProblem();
        if (status != OsqpEigen::ErrorExitFlag::NoError) return false;

        const Eigen::VectorXd z = solver.getSolution();
        for (int i = 0; i < 9 && i < z.size(); ++i) { out[i] = z[i]; }
        return true;
    }

    void NativeQpSolver::solveFallback(const Eigen::Matrix<double,9,9>& H,
                                    const Eigen::Matrix<double,9,1>& f,
                                    const Eigen::Matrix<double,9,1>& lb,
                                    const Eigen::Matrix<double,9,1>& ub,
                                    NativeQpOutput &out)
    {
        Eigen::Matrix<double,9,1> z = H.ldlt().solve(-f);
        for (int i=0;i<9;++i) {
            double zi = z[i];
            zi = std::max(lb[i], std::min(ub[i], zi));
            out[i] = zi;
        }
    }

    NativeQpOutput NativeQpSolver::solve(const NativeQpInput &in)
    {
        if (!std::isfinite(in.dt) || in.dt <= 1.0e-9)
        {
            throw std::invalid_argument("NativeQpSolver: dt must be finite and > 1e-9");
        }

        for (int i = 0; i < 54; ++i)
        {
            if (!std::isfinite(in.J6x9_colmajor[i]))
            {
                throw std::invalid_argument("NativeQpSolver: non-finite Jacobian entry");
            }
        }
        for (int i = 0; i < 6; ++i)
        {
            if (!std::isfinite(in.u_star[i]) ||
                !std::isfinite(in.q[i]) ||
                !std::isfinite(in.qmin[i]) ||
                !std::isfinite(in.qmax[i]))
            {
                throw std::invalid_argument("NativeQpSolver: non-finite u_star/q/qmin/qmax");
            }
            if (in.qmin[i] > in.qmax[i])
            {
                throw std::invalid_argument("NativeQpSolver: qmin > qmax");
            }
        }
        for (int i = 0; i < 9; ++i)
        {
            if (!std::isfinite(in.dotq_prev[i]))
            {
                throw std::invalid_argument("NativeQpSolver: non-finite dotq_prev");
            }
        }

        // Map inputs
        const Eigen::Matrix<double,6,9> J = mapJ(in.J6x9_colmajor);
        Eigen::Matrix<double,6,1> u; for (int i=0;i<6;++i) u[i] = in.u_star[i];
        Eigen::Matrix<double,9,1> dqprev; for (int i=0;i<9;++i) dqprev[i] = in.dotq_prev[i];
        
        // Build quadratic objective
        Eigen::Matrix<double,9,9> H; Eigen::Matrix<double,9,1> f;
        buildTask(J, u, dqprev, in, H, f);
        buildReg(H, in);
        if (!H.allFinite() || !f.allFinite())
        {
            throw std::invalid_argument("NativeQpSolver: objective has non-finite values");
        }

        // Bounds and linear constraints
        Eigen::Matrix<double,9,1> lb, ub; buildVarBounds(in, lb, ub);
        if (!lb.allFinite() || !ub.allFinite())
        {
            throw std::invalid_argument("NativeQpSolver: non-finite variable bounds");
        }
        for (int i = 0; i < 9; ++i)
        {
            if (lb[i] > ub[i])
            {
                if (i < 6)
                {
                    char msg[512];
                    std::snprintf(
                        msg,
                        sizeof(msg),
                        "NativeQpSolver: inconsistent variable bounds (lb > ub) at joint[%d]: "
                        "q=%.9f rad, qmin=%.9f rad, qmax=%.9f rad, lb=%.9f rad/s, ub=%.9f rad/s, "
                        "dt=%.9f s, max_dotq=%.9f rad/s",
                        i,
                        in.q[i],
                        in.qmin[i],
                        in.qmax[i],
                        lb[i],
                        ub[i],
                        in.dt,
                        in.max_dotq);
                    throw std::invalid_argument(msg);
                }
                else
                {
                    char msg[384];
                    std::snprintf(
                        msg,
                        sizeof(msg),
                        "NativeQpSolver: inconsistent variable bounds (lb > ub) at var[%d]: "
                        "lb=%.9f, ub=%.9f",
                        i,
                        lb[i],
                        ub[i]);
                    throw std::invalid_argument(msg);
                }
            }
        }

        Eigen::SparseMatrix<double> Aacc; Eigen::VectorXd lA, uA;
        buildAccelConstraints(in, Aacc, lA, uA);
        if (!lA.allFinite() || !uA.allFinite() || !isFiniteSparse(Aacc))
        {
            throw std::invalid_argument("NativeQpSolver: non-finite acceleration constraints");
        }

        const std::size_t n_extra = in.extra_A_rows.size();
        if (in.extra_l.size() != n_extra || in.extra_u.size() != n_extra)
        {
            throw std::invalid_argument("NativeQpInput extra constraints size mismatch");
        }
        for (std::size_t r = 0; r < n_extra; ++r)
        {
            if (!std::isfinite(in.extra_l[r]) || !std::isfinite(in.extra_u[r]))
            {
                throw std::invalid_argument("NativeQpSolver: non-finite extra row bounds");
            }
            if (in.extra_l[r] > in.extra_u[r])
            {
                throw std::invalid_argument("NativeQpSolver: inconsistent extra row bounds (l > u)");
            }
            for (int c = 0; c < 9; ++c)
            {
                if (!std::isfinite(in.extra_A_rows[r][c]))
                {
                    throw std::invalid_argument("NativeQpSolver: non-finite extra constraint row entry");
                }
            }
        }

        // Add variable bounds as linear rows: [I; -I] z ≤ [ub; -lb]
        const int r_acc = static_cast<int>(Aacc.rows());
        const int r_box = 2 * 9;
        const int r_extra = static_cast<int>(n_extra);
        const int r_total = r_acc + r_box + r_extra;

        Eigen::SparseMatrix<double> Ab(r_total, 9);
        std::vector<Eigen::Triplet<double>> trips;
        trips.reserve(Aacc.nonZeros() + 2 * 9 + r_extra * 9);
        for (int k=0;k<Aacc.outerSize();++k)
            for (Eigen::SparseMatrix<double>::InnerIterator it(Aacc,k); it; ++it)
            trips.emplace_back(it.row(), it.col(), it.value());
        const int r0 = r_acc;
        for (int i=0;i<9;++i) {
            trips.emplace_back(r0+i,   i,  1.0);
            trips.emplace_back(r0+9+i, i, -1.0);
        }
        const int r1 = r0 + r_box;
        for (int r = 0; r < r_extra; ++r)
        {
            const auto &row = in.extra_A_rows[r];
            for (int c = 0; c < 9; ++c)
            {
                if (std::abs(row[c]) > 1.0e-12)
                {
                    trips.emplace_back(r1 + r, c, row[c]);
                }
            }
        }
        Ab.setFromTriplets(trips.begin(), trips.end());

        Eigen::VectorXd lAll(Ab.rows()), uAll(Ab.rows());
        lAll.head(r0) = lA;                     uAll.head(r0) = uA;
        lAll.segment(r0,9).setConstant(-1e20); uAll.segment(r0,9) = ub;      // I z ≤ ub
        lAll.segment(r0 + 9,9).setConstant(-1e20);
        uAll.segment(r0 + 9,9) = -lb;                                              // -I z ≤ -lb
        if (r_extra > 0)
        {
            for (int r = 0; r < r_extra; ++r)
            {
                lAll[r1 + r] = in.extra_l[r];
                uAll[r1 + r] = in.extra_u[r];
            }
        }
        if (!lAll.allFinite() || !uAll.allFinite() || !isFiniteSparse(Ab))
        {
            throw std::invalid_argument("NativeQpSolver: non-finite assembled constraints");
        }

        // Sparse H for OSQP
        Eigen::SparseMatrix<double> Hs = H.sparseView();
        if (!isFiniteSparse(Hs))
        {
            throw std::invalid_argument("NativeQpSolver: non-finite Hessian");
        }
        NativeQpOutput out{};
        if (solveOsqp(Hs, f, Ab, lAll, uAll, dqprev, out)) return out;

        // Fallback:
        // If extra hard constraints are present, return clamped previous command
        // instead of unconstrained closed-form, to avoid violating hard rows by design.
        if (r_extra > 0)
        {
            NativeQpOutput out_prev{};
            for (int i=0;i<9;++i)
            {
                out_prev[i] = std::max(lb[i], std::min(ub[i], dqprev[i]));
            }
            return out_prev;
        }

        // Fallback (no extra hard rows)
        NativeQpOutput out_f{};
        solveFallback(H, f, lb, ub, out_f);
        return out_f;
    }
}
