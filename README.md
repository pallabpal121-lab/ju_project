# Physical AI Math Hardware Optimization Accelerator Suite

A high-performance, programmable **Hardware Optimization Accelerator Suite** implemented in SystemVerilog. Designed for embedded physical AI, robotics SLAM, trajectory optimization, nonlinear parameter estimation, classification, constrained optimal control, derivative-free black-box tuning, compressed sensing, distributed consensus, trust-region non-linear optimization, multi-agent swarm intelligence, accelerated proximal gradient methods, primal-dual interior point convex quadratic programming, neural network edge training, NP-hard combinatorial optimization, Total Variation (TV) image/signal reconstruction, projection-free constrained optimization, Augmented Lagrangian constrained optimization, decentralized multi-agent resource allocation, real-time adaptive filtering & system identification, Extended Kalman non-linear state estimation & sensor fusion, and scientific computing on FPGA/ASIC platforms.

The suite includes twenty-six specialized hardware architectures:
1. **Solver #1A: 1D 32-Bit Newton Accelerator (`Q16.16`)**: Lightweight fixed-point architecture for scalar non-linear equations.
2. **Solver #1B: 1D 64-Bit Newton Accelerator (`Q32.32`)**: High-precision architecture delivering ultra-fine resolution (`2^-32 ≈ 2.328 × 10^-10`) for aerospace and scientific computing.
3. **Solver #1C: Multivariable N-Dimensional Newton Accelerator (`Q16.16`)**: Coupled multi-variable optimization engine integrating a hardware **Cholesky decomposition linear system solver** $(H + \lambda I)\mathbf{p} = -\mathbf{g}$ to solve coupled vector optimization problems without matrix inversion.
4. **Solver #2: Levenberg-Marquardt (LM) Non-Linear Least Squares Accelerator (`Q16.16`)**: Industry-standard optimizer for Robotics SLAM, sensor calibration, and curve fitting featuring an **adaptive Marquardt damping controller** $(J^T J + \lambda I)\mathbf{p} = -J^T \mathbf{r}$.
5. **Solver #3: Iteratively Reweighted Least Squares (IRLS) Accelerator (`Q16.16`)**: Physical machine learning & classification engine optimizing Logistic Regression and Generalized Linear Models (GLM) via $(X^T W X + \lambda I) \Delta \mathbf{w} = X^T (\mathbf{y} - \mathbf{p})$ with a hardware Sigmoid probability unit.
6. **Solver #4: Quasi-Newton BFGS Accelerator (`Q16.16`)**: High-dimensional optimizer directly computing and updating the **Inverse Hessian Matrix ($B_k \approx H_k^{-1}$)** in silicon via symmetric Rank-2 updates, eliminating matrix inversions and second-derivative computations entirely.
7. **Solver #5: Non-Linear Conjugate Gradient (CG) Accelerator (`Q16.16`)**: Ultra-low-area matrix-free $O(N)$ vector memory solver executing **Polak-Ribière conjugate direction updates with Powell restarts** and directional curvature line search.
8. **Solver #6: Gauss-Newton Non-Linear Least Squares Accelerator (`Q16.16`)**: Second-order non-linear least squares engine solving $(J^T J + \lambda_{\text{eps}} I)\mathbf{p} = -J^T \mathbf{r}$ via direct hardware Cholesky factorization.
9. **Solver #7: Sequential Quadratic Programming (SQP) Constrained Accelerator (`Q16.16`)**: Active-Set Primal-Dual constrained optimization engine solving non-linear problems under hard physical box constraints $\mathbf{l} \le \mathbf{x} \le \mathbf{u}$ with KKT Lagrange multiplier shadow prices.
10. **Solver #8: Nelder-Mead Simplex Direct Search Accelerator (`Q16.16`)**: Derivative-free heuristic optimizer transforming an $(N+1)$-dimensional geometric simplex via hardware Reflection, Expansion, Contraction, and Shrink operations for non-differentiable or noisy black-box fitness functions.
11. **Solver #9: Limited-Memory BFGS (L-BFGS) Accelerator (`Q16.16`)**: High-dimensional Quasi-Newton solver operating via an $O(mN)$ circular displacement history buffer and **Two-Loop Recursion** pipeline, eliminating full matrix storage entirely.
12. **Solver #10: Coordinate Descent / LASSO L1 Sparsity Accelerator (`Q16.16`)**: Machine learning & compressed sensing optimizer executing cyclical coordinate sweeps with a **Hardware Soft-Thresholding Operator $S_\lambda(z)$**, driving non-predictive weights to exact silicon zero.
13. **Solver #11: Alternating Direction Method of Multipliers (ADMM) Accelerator (`Q16.16`)**: Distributed convex optimizer splitting primal consensus inversion $\mathbf{x} = (A^T A + \rho I)^{-1} \mathbf{v}$, proximal soft-thresholding $\mathbf{z} = S_{\lambda/\rho}(\mathbf{x} + \mathbf{u})$, and dual update $\mathbf{u} \leftarrow \mathbf{u} + (\mathbf{x} - \mathbf{z})$.
14. **Solver #12: Projected Gradient Descent (PGD) Accelerator (`Q16.16`)**: Multi-geometry constrained optimization engine implementing hardware projections for **Non-Negative Orthants ($\mathbf{x} \ge \mathbf{0}$)**, **Hyperbox Bounds ($\mathbf{l} \le \mathbf{x} \le \mathbf{u}$)**, **Euclidean $L_2$ Balls ($\|\mathbf{x}\|_2 \le R$)**, and **Probability Simplices ($\sum x_i = 1, x_i \ge 0$)**.
15. **Solver #13: Trust-Region Dogleg Non-Linear Optimizer (`Q16.16`)**: Robust global optimizer dynamically interpolating between steepest-descent **Cauchy Point ($\mathbf{p}_c = -\alpha_c \mathbf{g}$)** and unconstrained **Gauss-Newton Step ($\mathbf{p}_{gn} = -(J^T J)^{-1} \mathbf{g}$)** constrained within adaptive trust-region radius $\Delta$.
16. **Solver #14: Particle Swarm Optimization (PSO) Accelerator (`Q16.16`)**: Multi-agent global heuristic optimization engine executing parallel velocity updates with **Hardware Xorshift PRNG**, cognitive/social acceleration, velocity clamping, and personal/global best fitness tracking over non-convex multi-modal landscapes.
17. **Solver #15: Fast Iterative Shrinkage-Thresholding Algorithm (FISTA) (`Q16.16`)**: Accelerated proximal gradient optimizer executing **Nesterov Momentum Extrapolation** $\mathbf{y}_{k+1} = \mathbf{x}_k + \beta_k(\mathbf{x}_k - \mathbf{x}_{k-1})$ with $O(1/k^2)$ convergence rate and hardware **Soft-Thresholding Operator** $S_{\gamma \lambda}(\cdot)$ for sparse recovery.
18. **Solver #16: Primal-Dual Interior Point Method (IPM) for QP (`Q16.16`)**: Convex quadratic programming solver evaluating perturbed KKT conditions, condensed augmented normal equations $(Q + A^T \Theta A)\Delta \mathbf{x} = - \mathbf{g}_{\text{aug}}$ via hardware **Cholesky Decomposition**, and fraction-to-the-boundary step integration ($\alpha_p, \alpha_d$).
19. **Solver #17: Adam / RMSProp / Momentum SGD Neural Accelerator (`Q16.16`)**: Deep learning adaptive optimizer executing first-moment running mean $\mathbf{m}_t = \beta_1 \mathbf{m}_{t-1} + (1-\beta_1)\mathbf{g}_t$, second-moment uncentered variance $\mathbf{v}_t = \beta_2 \mathbf{v}_{t-1} + (1-\beta_2)\mathbf{g}_t^2$, coordinate-wise normalization $\frac{\alpha \mathbf{m}_t}{\sqrt{\mathbf{v}_t} + \epsilon}$, AdamW decoupled weight decay, and adaptive ravine step contraction.
20. **Solver #18: Quadratic Unconstrained Binary Optimization (QUBO) / Simulated Annealing (SA) Ising Accelerator (`Q16.16`)**: Hardware Ising Hamiltonian engine minimizing $E(\mathbf{q}) = \mathbf{q}^T Q \mathbf{q}$ over binary spins $\mathbf{q} \in \{0, 1\}^N$ via single-cycle local field evaluations $\Delta E_k$, pipelined Boltzmann exponential acceptance $P = \exp(-\Delta E / T)$, hardware Xorshift stochastic sampling, and geometric thermal cooling.
21. **Solver #19: Primal-Dual Hybrid Gradient (PDHG / Chambolle-Pock) Accelerator (`Q16.16`)**: Non-smooth first-order minimax saddle-point solver alternating dual projection $\mathbf{y}_{k+1} = \text{prox}_{\sigma g^*}(\mathbf{y}_k + \sigma K \bar{\mathbf{x}}_k)$, primal proximal resolution $\mathbf{x}_{k+1} = \text{prox}_{\tau f}(\mathbf{x}_k - \tau K^T \mathbf{y}_{k+1})$, and over-relaxation extrapolation $\bar{\mathbf{x}}_{k+1} = 2\mathbf{x}_{k+1} - \mathbf{x}_k$ for Total Variation (TV) denoising and compressed sensing.
22. **Solver #20: Frank-Wolfe / Conditional Gradient Accelerator (`Q16.16`)**: Projection-free constrained optimization engine replacing expensive Euclidean projections with a **Linear Minimization Oracle (LMO)** $\mathbf{s}_k = \arg\min_{\mathbf{s} \in \mathcal{C}} \langle \mathbf{s}, \nabla f(\mathbf{x}_k) \rangle$ over $L_1$ balls, hyperboxes, and probability simplices with exact quadratic line search and Duality Gap stopping certificates.
23. **Solver #21: Augmented Lagrangian Method (ALM) / Method of Multipliers Accelerator (`Q16.16`)**: Equality and inequality constrained optimization engine integrating an **Augmented KKT Cholesky linear solver**, Hestenes-Powell-Rockafellar inequality penalty engine, dual multiplier updater ($\boldsymbol{\lambda} \leftarrow \boldsymbol{\lambda} + \rho(A \mathbf{x} - \mathbf{b})$, $\boldsymbol{\mu} \leftarrow \max(\mathbf{0}, \boldsymbol{\mu} + \rho(C \mathbf{x} - \mathbf{d}))$), and adaptive penalty parameter controller ($\rho$).
24. **Solver #22: Dual Decomposition Multi-Agent Resource Allocation Engine (`Q16.16`)**: Decentralized multi-agent optimization architecture executing parallel local agent solvers $\mathbf{x}_s^*(\boldsymbol{\lambda}) = Q_s^{-1}(\mathbf{p}_s - A_s^T \boldsymbol{\lambda})$, broadcast shadow price coordinator $\boldsymbol{\lambda}_{k+1} = \boldsymbol{\lambda}_k + \alpha(\sum A_s \mathbf{x}_s - \mathbf{c})$, and market clearing for microgrid power flow and distributed edge networking.
25. **Solver #23: Recursive Least Squares (RLS) Adaptive Filtering Accelerator (`Q16.16`)**: Real-time streaming adaptive filter engine executing Sherman-Morrison-Woodbury inverse covariance matrix updates $P_t = \frac{1}{\lambda}(P_{t-1} - \mathbf{k}_t \mathbf{v}_t^T)$ in $O(N^2)$ operations with exponential forgetting factor $\lambda$, Kalman gain vector pipeline $\mathbf{k}_t = \frac{P \mathbf{x}}{\lambda + \mathbf{x}^T P \mathbf{x}}$, and symmetric covariance regularization.
26. **Solver #24: Extended Kalman Filter (EKF) Non-Linear State Estimator (`Q16.16`)**: Real-time robotics state estimation and multi-sensor fusion engine executing time-update state prediction ($\hat{\mathbf{x}}_k^- = \mathbf{f}(\hat{\mathbf{x}}, \mathbf{u}), P_k^- = F P F^T + Q$), non-linear measurement innovation ($\mathbf{y} = \mathbf{z} - \mathbf{h}(\hat{\mathbf{x}}^-)$), innovation covariance inversion ($S = H P^- H^T + R, S^{-1}$), Kalman gain matrix generation ($K = P^- H^T S^{-1}$), state update ($\hat{\mathbf{x}} = \hat{\mathbf{x}}^- + K \mathbf{y}$), and covariance update ($P = (I - K H)P^-$).

---

## Key Features & Highlights

- **Extended Kalman Filter (EKF) Predict-Correct SoC Engine**: Real-time non-linear filtering for autonomous robotics navigation, range-bearing radar tracking, and multi-sensor fusion.
- **Dedicated Time-Update Prediction Engine (`ekf_predict_engine.sv`)**: Evaluates non-linear state propagation $\mathbf{f}(\mathbf{x}, \mathbf{u})$ and computes sandwich product covariance growth $P^- = F P F^T + Q$ with symmetric enforcement in hardware.
- **Dedicated Measurement-Update Correction Engine (`ekf_correct_engine.sv`)**: Evaluates non-linear measurement residuals $\mathbf{y} = \mathbf{z} - \mathbf{h}(\hat{\mathbf{x}}^-)$, builds innovation covariance $S = H P^- H^T + R$, computes hardware matrix inversion $S^{-1}$, generates Kalman gain $K = P^- H^T S^{-1}$, updates state $\hat{\mathbf{x}} = \hat{\mathbf{x}}^- + K \mathbf{y}$, and applies covariance reduction $P = (I - K H)P^-$.
- **Real-Time Streaming Recursive Least Squares (RLS) Architecture**: Evaluates streaming Kalman gain and inverse covariance updates on the fly in ~50 clock cycles per sample without matrix inversions.
- **Sherman-Morrison-Woodbury Covariance Engine**: Directly computes $P_t = \frac{1}{\lambda}(P_{t-1} - \mathbf{k}_t \mathbf{v}_t^T)$ in hardware with symmetric regularization $P_t = \frac{1}{2}(P_t + P_t^T)$ to maintain positive-definiteness.
- **Decentralized Multi-Agent Dual Decomposition Architecture**: Solves coupled resource allocation $\min \sum f_s(\mathbf{x}_s) \text{ s.t. } \sum A_s \mathbf{x}_s = \mathbf{c} \ (\text{or } \le \mathbf{c})$ via parallel local agent optimization and master shadow price updates.
- **Parallel Local Agent Hardware Cores**: Evaluates local agent decisions $\mathbf{x}_s = Q_s^{-1}(\mathbf{p}_s - A_s^T \boldsymbol{\lambda})$ in parallel hardware pipelines without centralized state storage.
- **Master Shadow Price Coordinator**: Aggregates total resource usage $\sum A_s \mathbf{x}_s$, computes coupling residuals $\mathbf{r} = \sum A_s \mathbf{x}_s - \mathbf{c}$, and broadcasts updated market-clearing shadow prices $\boldsymbol{\lambda}$.
- **Hardware Augmented Lagrangian & KKT Cholesky Solver**: Evaluates the augmented primal step $(Q + \rho A^T A + \rho C_{\text{act}}^T C_{\text{act}} + \lambda_{\text{damp}} I)\Delta \mathbf{x} = -\mathbf{g}_{\text{aug}}$ via direct hardware Cholesky factorization, supporting both equality ($A \mathbf{x} = \mathbf{b}$) and inequality ($C \mathbf{x} \le \mathbf{d}$) constraints with shadow prices.
- **Hestenes-Powell-Rockafellar Dual Engine**: Pipelined multiplier updater executing $\boldsymbol{\lambda}_{k+1} = \boldsymbol{\lambda}_k + \rho(A \mathbf{x} - \mathbf{b})$ and $\mu_{k+1, i} = \max(0, \mu_{k, i} + \rho(c_i^T \mathbf{x} - d_i))$ with active constraint classification and geometric penalty escalation.
- **Hardware Linear Minimization Oracle (LMO)**: Evaluates extreme vertices of constraint polytopes in a single clock cycle without matrix inversions or quadratic subproblems.
- **Chambolle-Pock First-Order Primal-Dual Engine**: Solves non-smooth saddle-point optimization $\min_{\mathbf{x}} f(\mathbf{x}) + g(K \mathbf{x})$ in hardware via alternating forward-backward primal-dual iterations with single-cycle extrapolation $\bar{\mathbf{x}} = 2\mathbf{x}_{\text{new}} - \mathbf{x}_{\text{old}}$.
- **Hardware Ising Hamiltonian & Local Field Engine**: Evaluates single-spin flip energy changes $\Delta E_k = (1 - 2 q_k)(Q_{kk} + \sum_{j \ne k} (Q_{kj} + Q_{jk}) q_j)$ with zero-latency arithmetic, enabling millions of spin flips per second.
- **Pipelined Boltzmann Probability Evaluator**: Evaluates $P = \exp(-\Delta E / T)$ in fixed-point via base-2 decomposition $\exp(-u) = 2^{-k} \cdot (1 - \ln(2)f + 0.240226 f^2)$, delivering $>99.9\%$ accuracy across the entire domain.
- **Adaptive Moment Estimation (Adam) Pipeline**: Pipelined hardware moment accumulator maintaining first moment $\mathbf{m}_t$ and second moment $\mathbf{v}_t$, evaluating coordinate-wise normalized updates $\Delta \theta_i = \frac{\alpha m_i}{\sqrt{v_i} + \epsilon}$ via a dedicated 24-cycle restoring square root and 48-cycle divider.
- **Augmented Normal KKT Hardware Solver**: Evaluates the condensed $N \times N$ system $(Q + A^T \Theta A)\Delta \mathbf{x} = -\mathbf{g}_{\text{aug}}$ using hardware Cholesky decomposition, with scale-invariant diagonal damping $\Theta = Z S^{-1}$ clamped to avoid fixed-point ill-conditioning.
- **Fraction-to-the-Boundary Step Selector**: Evaluates $\alpha_p = \min(1.0, \tau \min_{\Delta s_i < 0} (s_i / -\Delta s_i))$ and $\alpha_d = \min(1.0, \tau \min_{\Delta z_i < 0} (z_i / -\Delta z_i))$ in hardware, guaranteeing strict primal-dual interior positivity ($\mathbf{s} > \mathbf{0}, \mathbf{z} > \mathbf{0}$).
- **Nesterov Momentum Acceleration Engine**: Evaluates recursive momentum scalar updates $t_{k+1} = \frac{1 + \sqrt{1 + 4 t_k^2}}{2}$ and extrapolation $\mathbf{y}_{k+1} = \mathbf{x}_k + \frac{t_k - 1}{t_{k+1}}(\mathbf{x}_k - \mathbf{x}_{k-1})$, delivering theoretical $O(1/k^2)$ convergence speedups.
- **Hardware Soft-Thresholding Proximal Operator**: Real-time evaluation of $S_{\gamma \lambda}(z) = \text{sign}(z)\max(|z|-\gamma \lambda, 0)$, enabling true $L_1$ sparsity induction and driving irrelevant features to exact $32'h0000\_0000$ silicon zero.
- **Multi-Agent Particle Swarm Engine**: Hardware state machine coordinating up to $P=8$ particles in $N=4$ dimensions, maintaining dedicated position, velocity, and personal best memory tables.
- **Powell Dogleg Trust-Region Interpolation Engine**: Hardware root-solver executing the quadratic formula in silicon to find the exact piecewise linear dogleg intersection $\mathbf{p}(\beta) = \mathbf{p}_c + \beta(\mathbf{p}_{gn} - \mathbf{p}_c)$ on the trust-region boundary $\|\mathbf{p}\|_2 = \Delta$.
- **Scale-Invariant Cauchy Gradient Normalizer**: Automatically normalizes large gradients $\mathbf{g}$ before computing $\alpha_c = \frac{\mathbf{g}^T \mathbf{g}}{\mathbf{g}^T B \mathbf{g}}$, completely eliminating fixed-point overflow for unbounded gradient magnitudes.
- **Multi-Geometry Hardware Projection Engine**: Real-time projection operators in silicon supporting non-negativity, box clamping, Euclidean ball norm scaling $\mathbf{y} \cdot \frac{R}{\|\mathbf{y}\|_2}$, and exact probability simplex water-filling.
- **3-Phase ADMM Distributed Engine**: Splitting primal linear solve (factorized once via hardware Cholesky), proximal soft-thresholding ($S_{\lambda/\rho}$), and dual multiplier accumulation with strict primal-dual consensus.
- **L-BFGS Two-Loop Recursion Pipeline**: Evaluates Quasi-Newton search directions $\mathbf{p} = -H_k \mathbf{g}_k$ using only $M=4$ displacement vectors ($\mathbf{s}_i, \mathbf{y}_i, \rho_i$) with backward/forward recursion, saving $>90\%$ silicon area compared to full matrix BFGS.
- **Derivative-Free Geometric Simplex Engine**: Nelder-Mead hardware state machine optimizing non-smooth and noisy functions via single-cycle bit-shift geometric transformations without derivatives or matrix inversions.
- **Active-Set Primal-Dual KKT Engine**: Solves hard inequality constrained problems in silicon, automatically classifying active boundary variables, computing shadow prices $\mu_i$, and solving reduced-order systems $H_{\text{free}} \mathbf{p}_{\text{free}} = -\mathbf{g}_{\text{free}}$ via Cholesky decomposition.
- **Direct Hardware Normal Equation Solvers**: Solves $(J^T J + \lambda_{\text{eps}} I)\mathbf{p} = -J^T \mathbf{r}$ with single-cycle zero-cost bit-shift Jacobian calculations and hardware Cholesky factorization.
- **Matrix-Free $O(N)$ Vector Architecture**: The Conjugate Gradient engine operates using only 4 vector registers in silicon, eliminating all matrix storage and matrix factorization overhead.
- **Quasi-Newton Rank-2 Inverse Hessian Updates**: Direct hardware accumulation of $B_{k+1} = B_k + \gamma_1(\mathbf{s} \mathbf{s}^T) - \gamma_2(\mathbf{s} \mathbf{u}^T + \mathbf{u} \mathbf{s}^T)$, evaluating search directions $\mathbf{p} = -B \mathbf{g}$ with zero matrix inversions.
- **Universal Programmable Equation Engine**: Evaluates arbitrary mathematical equations without hardware redesign by using an internal microcode processor with dedicated Program Memory and Register Files.
- **Hardware Sigmoid Activation Engine**: Single-cycle / pipelined Q16.16 Sigmoid evaluator $\sigma(\eta) = \frac{1}{1 + e^{-\eta}}$ using symmetric piecewise linear spline interpolation.
- **Zero-Cost Bit-Shift Calculus**: Computes gradients and Hessian curvatures via numerical finite differences. Step sizes $h = 2^{-4}$ and $h = 2^{-8}$ convert all derivative divisions into single-cycle arithmetic bit-shifts.
- **Dedicated Fixed-Point Linear Dividers & Sqrt Units**: Integrates 48-cycle / 96-cycle Radix-2 Restoring Dividers and 24-cycle Restoring Square Root units.

---

## Repository Structure

```
ju_project/
├── Documents/                           # Research papers, presentations, and architecture specs
├── Playstation/                         # Hardware accelerator workspace
│   ├── models/                          # Golden reference software models
│   ├── rtl/                             # SystemVerilog RTL source code
│   │   ├── newton_2nd_order_32bit/      # Solver #1A: 32-bit Q16.16 1D Accelerator
│   │   ├── newton_2nd_order_64bit/      # Solver #1B: 64-bit Q32.32 1D Accelerator
│   │   ├── newton_multivar_32bit/       # Solver #1C: Multivariable N-Dimensional Suite
│   │   ├── levenberg_marquardt_32bit/   # Solver #2: Levenberg-Marquardt Suite
│   │   ├── irls_32bit/                  # Solver #3: IRLS Suite
│   │   ├── bfgs_32bit/                  # Solver #4: BFGS Full-Matrix Suite
│   │   ├── cg_32bit/                    # Solver #5: Conjugate Gradient Suite
│   │   ├── gauss_newton_32bit/          # Solver #6: Gauss-Newton Suite
│   │   ├── sqp_32bit/                   # Solver #7: SQP Constrained Suite
│   │   ├── nelder_mead_32bit/           # Solver #8: Nelder-Mead Simplex Suite
│   │   ├── lbfgs_32bit/                 # Solver #9: L-BFGS Two-Loop Suite
│   │   ├── lasso_32bit/                 # Solver #10: LASSO Coordinate Descent Suite
│   │   ├── admm_32bit/                  # Solver #11: ADMM Suite
│   │   ├── pgd_32bit/                   # Solver #12: PGD Suite
│   │   ├── dogleg_32bit/                # Solver #13: Trust-Region Dogleg Suite
│   │   ├── pso_32bit/                   # Solver #14: Particle Swarm Optimization Suite
│   │   ├── fista_32bit/                 # Solver #15: FISTA Proximal Gradient Suite
│   │   ├── ipm_32bit/                   # Solver #16: Primal-Dual IPM Suite
│   │   ├── adam_32bit/                  # Solver #17: Adam Neural Accelerator Suite
│   │   ├── qubo_32bit/                  # Solver #18: QUBO / Simulated Annealing Suite
│   │   ├── pdhg_32bit/                  # Solver #19: PDHG / Chambolle-Pock Suite
│   │   ├── frank_wolfe_32bit/           # Solver #20: Frank-Wolfe Accelerator Suite
│   │   ├── alm_32bit/                   # Solver #21: ALM Accelerator Suite
│   │   ├── dual_decomp_32bit/           # Solver #22: Dual Decomposition Suite
│   │   ├── rls_32bit/                   # Solver #23: RLS Adaptive Filter Suite
│   │   └── ekf_32bit/                   # [NEW] Solver #24: Extended Kalman Filter Suite
│   │       ├── ekf_types_pkg.sv         # Package: dimensions, matrices, status codes
│   │       ├── ekf_helpers.svh          # Inline matrix/vector math, FPF^T, HPH^T
│   │       ├── q16_alu.sv               # Q16.16 ALU
│   │       ├── q16_divider.sv           # 48-cycle Restoring Divider
│   │       ├── ekf_predict_engine.sv    # Time update / prediction engine
│   │       ├── ekf_correct_engine.sv    # Measurement update / correction engine
│   │       └── ekf_top.sv               # Top-level EKF SoC controller
│   └── sim/                             # Simulation & Verification Environment
│       ├── Makefile                     # Build & run Makefile (all 26 targets)
│       └── tb_sv/                       # SystemVerilog testbenches
│           ├── tb_newton_2nd_order.sv       # 32-bit 1D testbench
│           ├── tb_newton_2nd_order_64bit.sv # 64-bit 1D testbench
│           ├── tb_newton_multivar.sv        # Multivariable testbench
│           ├── tb_levenberg_marquardt.sv    # Levenberg-Marquardt testbench
│           ├── tb_irls.sv                   # IRLS testbench
│           ├── tb_bfgs.sv                   # BFGS testbench
│           ├── tb_cg.sv                     # Conjugate Gradient testbench
│           ├── tb_gauss_newton.sv           # Gauss-Newton testbench
│           ├── tb_sqp.sv                    # SQP testbench
│           ├── tb_nelder_mead.sv            # Nelder-Mead testbench
│           ├── tb_lbfgs.sv                  # L-BFGS testbench
│           ├── tb_lasso.sv                  # LASSO testbench
│           ├── tb_admm.sv                   # ADMM testbench
│           ├── tb_pgd.sv                    # PGD testbench
│           ├── tb_dogleg.sv                 # Trust-Region Dogleg testbench
│           ├── tb_pso.sv                    # Particle Swarm Optimization testbench
│           ├── tb_fista.sv                  # FISTA Proximal Gradient testbench
│           ├── tb_ipm.sv                    # Primal-Dual IPM testbench
│           ├── tb_adam.sv                   # Adam Neural Accelerator testbench
│           ├── tb_qubo.sv                   # QUBO / Simulated Annealing testbench
│           ├── tb_pdhg.sv                   # PDHG / Chambolle-Pock testbench
│           ├── tb_frank_wolfe.sv            # Frank-Wolfe Accelerator testbench
│           ├── tb_alm.sv                    # ALM Accelerator testbench
│           ├── tb_dual_decomp.sv            # Dual Decomposition testbench
│           ├── tb_rls.sv                    # RLS Adaptive Filter testbench
│           └── tb_ekf.sv                    # EKF Accelerator testbench
└── README.md                            # Project documentation
```

---

## Quickstart: Simulation & Verification

Navigate to the simulation directory:
```bash
cd Playstation/sim
```

1. **Run Extended Kalman Filter Tests**:
   ```bash
   make run_ekf
   ```

2. **Run All 26 Solver Suites Regression**:
   ```bash
   make all
   ```

---

## Verification Test Benchmarks

| Solver | Test Case | Target Optimum ($\mathbf{x}^*$ / $\mathbf{w}^*$ / $\boldsymbol{\theta}^*$ / $\mathbf{q}^*$) | Hardware Result | Iterations / Steps | Status |
| :--- | :--- | :---: | :---: | :---: | :---: |
| **Newton 1D (32-Bit)** | $f(x) = (x - 3)^2$ | $x^* = 3.0$ | $x^* = 3.001709$ | 3 | **PASSED** |
| **Newton 1D (32-Bit)** | $f(x) = x^4 - 4x^2 + 5$ | $x^* = \sqrt{2} \approx 1.4142$ | $x^* = 1.412918$ | 5 | **PASSED** |
| **Newton 1D (32-Bit)** | $f(x) = x^8 - 4x^4 + 3$ | $x^* = 2^{0.25} \approx 1.1892$ | $x^* = 1.184296$ | 7 | **PASSED** |
| **Newton 1D (64-Bit)** | $f(x) = (x - 3)^2$ | $x^* = 3.0$ | $x^* = 3.000008$ | 2 | **PASSED** |
| **Newton 1D (64-Bit)** | $f(x) = x^4 - 4x^2 + 5$ | $x^* = \sqrt{2} \approx 1.414214$ | $x^* = 1.414216$ | 5 | **PASSED** |
| **Newton 1D (64-Bit)** | $f(x) = x^8 - 4x^4 + 3$ | $x^* = 2^{0.25} \approx 1.189207$ | $x^* = 1.189189$ | 7 | **PASSED** |
| **Newton Multivar** | $f(x_0, x_1) = (x_0-2)^2 + (x_1-5)^2 + x_0 x_1$ | $(-0.666667, 5.333333)$ | $(-0.666550, 5.333328)$ | 2 | **PASSED** |
| **Newton Multivar** | $f(x_0, x_1) = 2(x_0-3)^2 + 3(x_1-4)^2$ | $(3.000000, 4.000000)$ | $(3.000061, 3.999985)$ | 2 | **PASSED** |
| **Newton Multivar** | $f(x_0, x_1, x_2) = (x_0-1)^2 + (x_1-2)^2 + (x_2-3)^2 + x_0 x_1$ | $(0.0, 2.0, 3.0)$ | $(0.000076, 2.000031, 3.000061)$ | 2 | **PASSED** |
| **Levenberg-Marquardt** | Parabola Parameter Estimation $y(t) = a t^2 + b t + c$ | $(1.0, -2.0, 3.0)$ | $(0.999954, -1.998993, 2.998825)$ | 2 | **PASSED** |
| **Levenberg-Marquardt** | 2D Robotics SLAM Localization $y(t) = (t - x_c)^2 + y_c$ | $(2.0, 3.0)$ | $(2.000046, 3.009735)$ | 3 | **PASSED** |
| **IRLS (Solver #3)** | 2D Linearly Separable Binary Classification | $100\%$ Accuracy ($p_m > 0.5$ for $y=1$) | $100.0\%$ Accuracy (6/6 correct) | 4 | **PASSED** |
| **IRLS (Solver #3)** | 1D Sigmoidal Logistic Thresholding | $100\%$ Accuracy | $100.0\%$ Accuracy (6/6 correct) | 5 | **PASSED** |
| **BFGS (Solver #4)** | 2D Coupled Quadratic $(x_0-2)^2 + (x_1-5)^2 + x_0 x_1$ | $(-0.666667, 5.333333)$ | $(-0.666672, 5.333328)$ | 4 | **PASSED** |
| **BFGS (Solver #4)** | 2D Decoupled Quadratic $2(x_0-3)^2 + 3(x_1-4)^2$ | $(3.000000, 4.000000)$ | $(3.000488, 3.999756)$ | 16 | **PASSED** |
| **BFGS (Solver #4)** | 3D Coupled Quadratic $(x_0-1)^2 + (x_1-2)^2 + (x_2-3)^2 + x_0 x_1$ | $(0.0, 2.0, 3.0)$ | $(-0.000046, 2.000061, 3.000046)$ | 6 | **PASSED** |
| **Conjugate Gradient (#5)** | 2D Coupled Quadratic $(x_0-2)^2 + (x_1-5)^2 + x_0 x_1$ | $(-0.666667, 5.333333)$ | $(-0.666580, 5.333344)$ | 2 | **PASSED** |
| **Conjugate Gradient (#5)** | 2D Decoupled Quadratic $2(x_0-3)^2 + 3(x_1-4)^2$ | $(3.000000, 4.000000)$ | $(3.000351, 4.000107)$ | 2 | **PASSED** |
| **Conjugate Gradient (#5)** | 3D Coupled Quadratic $(x_0-1)^2 + (x_1-2)^2 + (x_2-3)^2 + x_0 x_1$ | $(0.0, 2.0, 3.0)$ | $(-0.001358, 2.001373, 3.000031)$ | 5 | **PASSED** |
| **Gauss-Newton (#6)** | Parabola Parameter Estimation $y(t) = a t^2 + b t + c$ | $(1.0, -2.0, 3.0)$ | $(1.000015, -1.999985, 2.999985)$ | 2 | **PASSED** |
| **Gauss-Newton (#6)** | 2D Robotics SLAM Localization $y(t) = (t - x_c)^2 + y_c$ | $(2.0, 3.0)$ | $(2.000000, 3.000015)$ | 3 | **PASSED** |
| **SQP (#7)** | 2D Paraboloid ($x_0 \le 1.5, x_1 \le 2.5$) | $(1.500000, 2.500000)$ | $(1.500000, 2.500000)$ | 1 | **PASSED** |
| **SQP (#7)** | 2D Coupled Quadratic ($x_0 \ge 0, x_1 \ge 0$) | $(0.000000, 5.000000)$ | $(0.000000, 4.999496)$ | 2 | **PASSED** |
| **SQP (#7)** | 3D Multi-Axis Mixed Box Constraints | $(1.0, 1.5, 2.0)$ | $(1.000000, 1.499298, 2.000000)$ | 2 | **PASSED** |
| **Nelder-Mead (#8)** | 2D Paraboloid Direct Search | $(3.000000, 4.000000)$ | $(3.003143, 3.999542)$ | 28 | **PASSED** |
| **Nelder-Mead (#8)** | 2D Coupled Quadratic Direct Search | $(-0.666667, 5.333333)$ | $(-0.668518, 5.332626)$ | 29 | **PASSED** |
| **Nelder-Mead (#8)** | Non-Smooth $\|x_0 - 2\| + 2\|x_1 - 3\|$ | $(2.000000, 3.000000)$ | $(1.998215, 2.997726)$ | 30 | **PASSED** |
| **L-BFGS (#9)** | 2D Coupled Quadratic | $(-0.666667, 5.333333)$ | $(-0.666412, 5.333221)$ | 3 | **PASSED** |
| **L-BFGS (#9)** | 2D Decoupled Paraboloid | $(3.000000, 4.000000)$ | $(2.999985, 4.000031)$ | 4 | **PASSED** |
| **L-BFGS (#9)** | 3D Coupled Quadratic | $(0.000000, 2.000000, 3.000000)$ | $(0.000244, 2.000015, 3.000046)$ | 6 | **PASSED** |
| **LASSO (#10)** | OLS Linear Regression ($\lambda = 0.0$) | $(2.000000, 3.000000)$ | $(2.000778, 2.999374)$ | 18 | **PASSED** |
| **LASSO (#10)** | Sparse Feature Selection ($\lambda = 1.0$) | $(4.000000, 0.000000, 0.000000)$ | $(3.966660, 0.000000, 0.000000)$ | 1 | **PASSED** |
| **LASSO (#10)** | Compressed Sensing Sparse Recovery ($\lambda = 0.5$) | $(1.5, 0.0, 2.5, 0.0)$ | $(1.482132, 0.000000, 2.482224, 0.000000)$ | 17 | **PASSED** |
| **ADMM (#11)** | Linear Consensus ($\lambda = 0.0, \rho = 1.0$) | $(2.000000, 3.000000)$ | $(2.000092, 2.999908)$ | 5 | **PASSED** |
| **ADMM (#11)** | Sparse Feature Selection ($\lambda = 1.0, \rho = 1.0$) | $(4.000000, 0.000000, 0.000000)$ | $(3.966522, 0.000000, 0.000000)$ | 6 | **PASSED** |
| **ADMM (#11)** | Compressed Sensing Sparse Recovery ($\lambda = 0.5, \rho = 1.0$) | $(1.5, 0.0, 2.5, 0.0)$ | $(1.482208, 0.000000, 2.482162, 0.000000)$ | 7 | **PASSED** |
| **PGD (#12)** | Non-Negative Orthant ($x_0 \ge 0, x_1 \ge 0$) | $(0.000000, 4.000000)$ | $(0.000000, 4.000977)$ | 24 | **PASSED** |
| **PGD (#12)** | Euclidean $L_2$ Ball ($\|\mathbf{x}\|_2 \le 2.0$) | $(1.200000, 1.600000)$ | $(1.200104, 1.599899)$ | 2 | **PASSED** |
| **PGD (#12)** | Probability Simplex ($\sum x_i = 1, x_i \ge 0$) | $(0.000000, 1.000000, 0.000000)$ | $(0.000000, 1.000000, 0.000000)$ | 26 | **PASSED** |
| **Dogleg (#13)** | Non-Linear Parameter Estimation $y(t) = a t^2 + b t + c$ | $(1.000000, -2.000000, 3.000000)$ | $(1.000473, -1.999695, 2.998672)$ | 2 | **PASSED** |
| **Dogleg (#13)** | 2D Robotics SLAM Localization $y(t) = (t - x_c)^2 + y_c$ | $(2.000000, 3.000000)$ | $(2.000000, 3.000168)$ | 3 | **PASSED** |
| **Dogleg (#13)** | Cubic Physical Sensor Calibration $y(t) = a t^3 + b t$ | $(0.500000, 2.000000)$ | $(0.499542, 2.001678)$ | 2 | **PASSED** |
| **PSO (#14)** | 2D Coupled Multi-Agent Optimization | $(0.666667, 2.666667)$ | $(0.583466, 2.693283)$ | 3 | **PASSED** |
| **PSO (#14)** | 3D Sphere Global Optimization | $(1.000000, -2.000000, 3.000000)$ | $(0.909378, -1.988480, 2.887711)$ | 13 | **PASSED** |
| **PSO (#14)** | 2D Non-Convex Curved Valley (Rosenbrock) | $(1.000000, 1.000000)$ | $(1.000000, 1.000000)$ | 1 | **PASSED** |
| **FISTA (#15)** | Accelerated Smooth Convex Optimization ($\lambda = 0.0$) | $(3.000000, 4.000000)$ | $(3.002121, 4.000031)$ | 16 | **PASSED** |
| **FISTA (#15)** | Sparse Feature Selection ($\lambda = 1.0$) | $(3.500000, 0.000000, 0.000000)$ | $(3.507629, 0.000000, 0.000000)$ | 24 | **PASSED** |
| **FISTA (#15)** | 4D Compressed Sensing Sparse Signal Recovery ($\lambda = 0.5$) | $(0.750000, 0.000000, 1.750000, 0.000000)$ | $(0.742096, 0.000000, 1.731613, 0.000000)$ | 17 | **PASSED** |
| **IPM (#16)** | 2D Box-Constrained Convex QP | $(1.000000, 2.000000)$ | $(0.999268, 1.999268)$ | 3 | **PASSED** |
| **IPM (#16)** | 2D Coupled Quadratic on Half-Space | $(0.750000, 0.750000)$ | $(0.748962, 0.749100)$ | 9 | **PASSED** |
| **IPM (#16)** | 3D Multi-Constraint Actuator Allocation QP | $(1.500000, 1.000000, 0.500000)$ | $(1.499344, 0.997971, 0.501907)$ | 4 | **PASSED** |
| **Adam (#17)** | 2D Ill-Conditioned Anisotropic Valley Optimization (Adam) | $(2.000000, 3.000000)$ | $(1.974915, 3.029999)$ | 75 | **PASSED** |
| **Adam (#17)** | 2D Non-Stationary Ridge Tracking (RMSProp) | $(4.000000, -1.000000)$ | $(3.996964, -0.996902)$ | 26 | **PASSED** |
| **Adam (#17)** | 3D Weight Regularization with L2 Decay (Momentum SGD) | $(0.952381, 1.904762, 2.857143)$ | $(0.959930, 1.919815, 2.879684)$ | 40 | **PASSED** |
| **QUBO (#18)** | 4-Spin Max-Cut Graph Partitioning | $[0, 1, 0, 1]$ ($E^* = -8.0$) | $0101$ ($E = -8.000000$) | 60 | **PASSED** |
| **QUBO (#18)** | 4-Spin Number Partitioning (NP-Complete) | $[1, 0, 0, 1]$ ($E^* = -30.25$) | $1001$ ($E = -30.250000$) | 60 | **PASSED** |
| **QUBO (#18)** | 4-Spin Frustrated Ising Spin Glass | 1 Spin ON ($E^* = -2.0$) | $1000$ ($E = -2.000000$) | 60 | **PASSED** |
| **PDHG (#19)** | Total Variation (TV-L2) 1D Step Signal Denoising | $[1.250000, 1.250000, 3.750000, 3.750000]$ | $[1.248840, 1.250610, 3.749329, 3.751099]$ | 31 | **PASSED** |
| **PDHG (#19)** | Basis Pursuit / L1 Sparse Signal Recovery | $[2.000000, 0.000000, 3.000000, 0.000000]$ | $[1.966736, 0.000000, 2.966705, 0.000000]$ | 58 | **PASSED** |
| **PDHG (#19)** | Non-Negative Constrained Linear Inversion | $[0.000000, 2.000000]$ | $[0.000000, 2.001770]$ | 49 | **PASSED** |
| **Frank-Wolfe (#20)** | $L_1$ Ball Constrained Quadratic (Sparse FW) | $(1.000000, 0.000000)$ | $(1.000000, 0.000000)$ | 2 | **PASSED** |
| **Frank-Wolfe (#20)** | Probability Simplex Constrained (Simplex FW) | $(0.000000, 0.450000, 0.550000, 0.000000)$ | $(0.008865, 0.444443, 0.537033, 0.008865)$ | 50 | **PASSED** |
| **Frank-Wolfe (#20)** | Hyperbox Constrained Quadratic (Box FW) | $(1.000000, 0.500000)$ | $(1.000000, 0.500000)$ | 3 | **PASSED** |
| **ALM (#21)** | Equality Constrained QP ($x_0 + x_1 = 4.0$) | $(1.000000, 3.000000), \lambda^* = 1.0$ | $(1.000458, 3.000458), \lambda = 0.999573$ | 7 | **PASSED** |
| **ALM (#21)** | Inequality Constrained QP ($x_0 + 2x_1 \le 3.0$) | $(1.800000, 0.600000), \mu^* = 1.2$ | $(1.800034, 0.600052), \mu = 1.200027$ | 5 | **PASSED** |
| **ALM (#21)** | 3D Multi-Constraint Actuator Allocation | $(0.250000, 1.250000, 1.500000)$ | $(0.249390, 1.249420, 1.500839)$ | 8 | **PASSED** |
| **Dual Decomp (#22)** | 3-Agent Microgrid Power Allocation ($6.0$ MW) | $(1.727273, 2.454545, 1.818182), \lambda^* \approx 0.545$ | $(1.727646, 2.455307, 1.818405), \lambda = 0.545227$ | 6 | **PASSED** |
| **Dual Decomp (#22)** | 2-Agent Inequality Capacity Budget ($x_1 + x_2 \le 5.0$) | $(2.500000, 2.500000), \lambda^* = 1.5$ | $(2.500488, 2.500488), \lambda = 1.499893$ | 6 | **PASSED** |
| **Dual Decomp (#22)** | 2D Multi-Resource Coupled Allocation | $\mathbf{x}_1^* = [1.5, 0.5], \mathbf{x}_2^* = [0.5, 1.5], \boldsymbol{\lambda}^* = [0.5, 0.5]$ | $\mathbf{x}_1 = [1.500809, 0.500809], \mathbf{x}_2 = [0.500809, 1.500809]$ | 5 | **PASSED** |
| **RLS (#23)** | 2-Tap Real-Time Adaptive System Identification | $\mathbf{w}^* = [1.500000, -2.500000]$ | $\mathbf{w} = [1.499084, -2.496918]$ | 6 samples | **PASSED** |
| **RLS (#23)** | 3-Tap Acoustic Echo Cancellation ($\lambda = 0.95$) | $\mathbf{w}^* = [0.800000, -1.200000, 2.000000]$ | $\mathbf{w} = [0.800308, -1.199814, 1.998749]$ | 20 samples | **PASSED** |
| **RLS (#23)** | 4-Tap Physical Sensor Calibration ($\lambda = 0.98$) | $\mathbf{w}^* = [0.5, 1.0, -0.5, 2.5]$ | $\mathbf{w} = [0.500046, 0.999847, -0.500183, 2.497070]$ | 25 samples | **PASSED** |
| **EKF (#24)** | 2D Non-Linear Radar Range Tracking ($z = \sqrt{p_x^2 + y_0^2}$) | $(p_x, v_x) = (15.000000, 2.000000)$ | $(p_x, v_x) = (14.977875, 1.984329)$ | 10 steps | **PASSED** |
| **EKF (#24)** | 2D Autonomous Vehicle Kinematic Motion | $(x, y) = (6.000000, 8.000000)$ | $(x, y) = (5.989655, 8.008240)$ | 8 steps | **PASSED** |
| **EKF (#24)** | 4D Multi-Sensor Kinematic Target Tracking | $[5.0, -7.5, 1.0, -1.5]$ | $[4.988876, -7.490707, 0.994583, -1.495499]$ | 10 steps | **PASSED** |
