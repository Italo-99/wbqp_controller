
#include "wbqp_controller/qp_solver_native.hpp"
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace wbqp
{
    namespace
    {
        const Eigen::Matrix<double,6,6>& taskWeightMatrix()
        {
            static const Eigen::Matrix<double,6,6> Wt = [] {
                Eigen::Matrix<double,6,6> m = Eigen::Matrix<double,6,6>::Zero();
                for (int i = 0; i < 3; ++i) { m(i,i) = NativeQpConfig::W_lin; }
                for (int i = 3; i < 6; ++i) { m(i,i) = NativeQpConfig::W_ang; }
                return m;
            }();
            return Wt;
        }

        const Eigen::Matrix<double,9,9>& regularizationContribution()
        {
            static const Eigen::Matrix<double,9,9> Hreg = [] {
                Eigen::Matrix<double,9,9> R = Eigen::Matrix<double,9,9>::Zero();
                for (int i = 0; i < 6; ++i) { R(i,i) = NativeQpConfig::beta_arm; }
                R(6,6) = NativeQpConfig::alpha_xy;
                R(7,7) = NativeQpConfig::alpha_xy;
                R(8,8) = NativeQpConfig::alpha_yaw;
                return 2.0 * (R + NativeQpConfig::nu * Eigen::Matrix<double,9,9>::Identity());
            }();
            return Hreg;
        }

        const Eigen::Matrix<double,9,1>& baseLowerBounds()
        {
            static const Eigen::Matrix<double,9,1> lb = [] {
                Eigen::Matrix<double,9,1> v;
                v << -NativeQpConfig::max_dotq, -NativeQpConfig::max_dotq, -NativeQpConfig::max_dotq,
                     -NativeQpConfig::max_dotq, -NativeQpConfig::max_dotq, -NativeQpConfig::max_dotq,
                     -NativeQpConfig::max_V, -NativeQpConfig::max_V, -NativeQpConfig::max_Omegaz;
                return v;
            }();
            return lb;
        }

        const Eigen::Matrix<double,9,1>& baseUpperBounds()
        {
            static const Eigen::Matrix<double,9,1> ub = [] {
                Eigen::Matrix<double,9,1> v;
                v << NativeQpConfig::max_dotq, NativeQpConfig::max_dotq, NativeQpConfig::max_dotq,
                     NativeQpConfig::max_dotq, NativeQpConfig::max_dotq, NativeQpConfig::max_dotq,
                     NativeQpConfig::max_V, NativeQpConfig::max_V, NativeQpConfig::max_Omegaz;
                return v;
            }();
            return ub;
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
                                   Eigen::Matrix<double,9,9>& H,
                                   Eigen::Matrix<double,9,1>& f)
    {
        const Eigen::Matrix<double,6,6>& Wt = taskWeightMatrix();

        // 0.5 z^T H z + f^T z with 2x scaling to match LS normal equations convention
        H = 2.0 * (J.transpose() * Wt * J);
        f = -2.0 * (J.transpose() * Wt * u + NativeQpConfig::nu * dqprev);
    }

    void NativeQpSolver::buildReg(Eigen::Matrix<double,9,9>& H)
    {
        H += regularizationContribution();
    }

    void NativeQpSolver::buildVarBounds(const NativeQpInput& in,
                                        Eigen::Matrix<double,9,1> &lb,
                                        Eigen::Matrix<double,9,1> &ub)
    {
        lb = baseLowerBounds();
        ub = baseUpperBounds();

        // Joints: velocity box AND position window this step: q + dq*dt ∈ [qmin,qmax]
        for (int i=0;i<6;++i) {
            const double lb_pos = (in.qmin[i] - in.q[i]) / in.dt;
            const double ub_pos = (in.qmax[i] - in.q[i]) / in.dt;
            const double lb_i = std::max(baseLowerBounds()[i], lb_pos);
            const double ub_i = std::min(baseUpperBounds()[i], ub_pos);
            lb[i] = lb_i; ub[i] = ub_i;
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
            if (idx<6) return NativeQpConfig::qddot_max * in.dt;
            if (idx<8) return NativeQpConfig::a_lin_max * in.dt; // 6,7
            return NativeQpConfig::alpha_max * in.dt;            // 8
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
        // Map inputs
        const Eigen::Matrix<double,6,9> J = mapJ(in.J6x9_colmajor);
        Eigen::Matrix<double,6,1> u; for (int i=0;i<6;++i) u[i] = in.u_star[i];
        Eigen::Matrix<double,9,1> dqprev; for (int i=0;i<9;++i) dqprev[i] = in.dotq_prev[i];
        
        // Build quadratic objective
        Eigen::Matrix<double,9,9> H; Eigen::Matrix<double,9,1> f;
        buildTask(J, u, dqprev, H, f);
        buildReg(H);

        // Bounds and linear constraints
        Eigen::Matrix<double,9,1> lb, ub; buildVarBounds(in, lb, ub);

        Eigen::SparseMatrix<double> Aacc; Eigen::VectorXd lA, uA;
        buildAccelConstraints(in, Aacc, lA, uA);

        // Add variable bounds as linear rows: [I; -I] z ≤ [ub; -lb]
        Eigen::SparseMatrix<double> Ab(Aacc.rows()+2*9, 9);
        std::vector<Eigen::Triplet<double>> trips; trips.reserve(Aacc.nonZeros()+2*9);
        for (int k=0;k<Aacc.outerSize();++k)
            for (Eigen::SparseMatrix<double>::InnerIterator it(Aacc,k); it; ++it)
            trips.emplace_back(it.row(), it.col(), it.value());
        const int r0 = static_cast<int>(Aacc.rows());
        for (int i=0;i<9;++i) {
            trips.emplace_back(r0+i,   i,  1.0);
            trips.emplace_back(r0+9+i, i, -1.0);
        }
        Ab.setFromTriplets(trips.begin(), trips.end());

        Eigen::VectorXd lAll(Ab.rows()),       uAll(Ab.rows());
        lAll.head(r0) = lA;                    uAll.head(r0) = uA;
        lAll.segment(r0,9).setConstant(-1e20); uAll.segment(r0,9) = ub;      // I z ≤ ub
        lAll.tail(9).setConstant(-1e20);       uAll.tail(9)       = -lb;     // -I z ≤ -lb

        // Sparse H for OSQP
        Eigen::SparseMatrix<double> Hs = H.sparseView();
        NativeQpOutput out{};
        if (solveOsqp(Hs, f, Ab, lAll, uAll, dqprev, out)) return out;

        // Fallback
        NativeQpOutput out_f{};
        solveFallback(H, f, lb, ub, out_f);
        return out_f;
    }
}
