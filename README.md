# Physical AI Math Hardware Optimization Accelerator Suite

A high-performance, programmable **Hardware Optimization Accelerator Suite** implemented in SystemVerilog. Designed for embedded physical AI, robotics SLAM, trajectory optimization, nonlinear parameter estimation, classification, constrained optimal control, derivative-free black-box tuning, compressed sensing, distributed consensus, trust-region global non-linear optimization, and scientific computing on FPGA/ASIC platforms.

The suite includes fifteen specialized hardware architectures:
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

---

## Key Features & Highlights

- **Powell Dogleg Trust-Region Interpolation Engine**: Hardware root-solver executing the quadratic formula in silicon to find the exact piecewise linear dogleg intersection $\mathbf{p}(\beta) = \mathbf{p}_c + \beta(\mathbf{p}_{gn} - \mathbf{p}_c)$ on the trust-region boundary $\|\mathbf{p}\|_2 = \Delta$.
- **Scale-Invariant Cauchy Gradient Normalizer**: Automatically normalizes large gradients $\mathbf{g}$ before computing $\alpha_c = \frac{\mathbf{g}^T \mathbf{g}}{\mathbf{g}^T B \mathbf{g}}$, completely eliminating fixed-point overflow for unbounded gradient magnitudes.
- **Adaptive Trust Radius Controller**: Dynamically tunes trust radius $\Delta$ based on the gain ratio $\rho = \frac{\Delta F_{\text{act}}}{\Delta m_{\text{pred}}}$, expanding $\Delta \leftarrow \min(2\Delta, \Delta_{\max})$ on high model accuracy and contracting $\Delta \leftarrow \max(0.5\Delta, \Delta_{\min})$ on model mismatch.
- **Multi-Geometry Hardware Projection Engine**: Real-time projection operators in silicon supporting non-negativity, box clamping, Euclidean ball norm scaling $\mathbf{y} \cdot \frac{R}{\|\mathbf{y}\|_2}$, and exact probability simplex water-filling.
- **3-Phase ADMM Distributed Engine**: Splitting primal linear solve (factorized once via hardware Cholesky), proximal soft-thresholding ($S_{\lambda/\rho}$), and dual multiplier accumulation with strict primal-dual consensus.
- **Hardware Soft-Thresholding Operator**: Real-time evaluation of $S_\lambda(z) = \text{sign}(z)\max(|z|-\lambda, 0)$, enabling true $L_1$ sparsity induction and driving irrelevant features to exact $32'h0000\_0000$ silicon zero.
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
│   │   └── dogleg_32bit/                # [NEW] Solver #13: Trust-Region Dogleg Suite
│   │       ├── dogleg_types_pkg.sv      # Package: vector types, step classification
│   │       ├── dogleg_helpers.svh       # Inline vector and matrix helpers
│   │       ├── q16_alu.sv               # Q16.16 ALU
│   │       ├── q16_divider.sv           # 48-cycle Restoring Divider
│   │       ├── q16_sqrt.sv              # 24-cycle Square Root Unit
│   │       ├── dfg_dogleg_engine.sv     # Universal microcode objective evaluator f(t, x)
│   │       ├── dogleg_jacobian_engine.sv# Central finite-difference Jacobian & Hessian engine
│   │       ├── cholesky_solver_engine.sv# Hardware Cholesky normal solver (B · p_gn = -g)
│   │       ├── dogleg_step_engine.sv    # Powell Dogleg step interpolation unit
│   │       └── dogleg_top.sv            # Master Trust-Region SoC Controller
│   └── sim/                             # Simulation & Verification Environment
│       ├── Makefile                     # Build & run Makefile (all 15 targets)
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
│           └── tb_dogleg.sv                 # Trust-Region Dogleg testbench
└── README.md                            # Project documentation
```

---

## Quickstart: Simulation & Verification

Navigate to the simulation directory:
```bash
cd Playstation/sim
```

1. **Run 32-Bit 1D Newton Tests**:
   ```bash
   make run_32bit
   ```

2. **Run 64-Bit 1D High-Precision Tests**:
   ```bash
   make run_64bit
   ```

3. **Run Multivariable N-Dimensional Tests (Cholesky Solver)**:
   ```bash
   make run_multivar
   ```

4. **Run Levenberg-Marquardt (LM) Tests**:
   ```bash
   make run_lm
   ```

5. **Run Iteratively Reweighted Least Squares (IRLS) Tests**:
   ```bash
   make run_irls
   ```

6. **Run Quasi-Newton BFGS Tests**:
   ```bash
   make run_bfgs
   ```

7. **Run Non-Linear Conjugate Gradient (CG) Tests**:
   ```bash
   make run_cg
   ```

8. **Run Gauss-Newton Tests**:
   ```bash
   make run_gn
   ```

9. **Run SQP Constrained Tests**:
   ```bash
   make run_sqp
   ```

10. **Run Nelder-Mead Simplex Tests**:
    ```bash
    make run_nm
    ```

11. **Run L-BFGS Tests**:
    ```bash
    make run_lbfgs
    ```

12. **Run LASSO Coordinate Descent Tests**:
    ```bash
    make run_lasso
    ```

13. **Run ADMM Tests**:
    ```bash
    make run_admm
    ```

14. **Run PGD Tests**:
    ```bash
    make run_pgd
    ```

15. **Run Trust-Region Dogleg Tests**:
    ```bash
    make run_dogleg
    ```

16. **Run All 15 Solver Suites Regression**:
    ```bash
    make all
    ```

---

## Verification Test Benchmarks

| Solver | Test Case | Target Optimum ($\mathbf{x}^*$ / $\mathbf{w}^*$ / $\mathbf{z}^*$) | Hardware Result | Iterations | Status |
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
