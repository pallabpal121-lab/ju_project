# Physical AI Math Hardware Optimization Accelerator Suite

A high-performance, programmable **Hardware Optimization Accelerator Suite** implemented in SystemVerilog. Designed for embedded physical AI, robotics SLAM, trajectory optimization, nonlinear parameter estimation, classification, and scientific computing on FPGA/ASIC platforms.

The suite includes eight specialized hardware architectures:
1. **Solver #1A: 1D 32-Bit Newton Accelerator (`Q16.16`)**: Lightweight fixed-point architecture for scalar non-linear equations.
2. **Solver #1B: 1D 64-Bit Newton Accelerator (`Q32.32`)**: High-precision architecture delivering ultra-fine resolution (`2^-32 ≈ 2.328 × 10^-10`) for aerospace and scientific computing.
3. **Solver #1C: Multivariable N-Dimensional Newton Accelerator (`Q16.16`)**: Coupled multi-variable optimization engine integrating a hardware **Cholesky decomposition linear system solver** $(H + \lambda I)\mathbf{p} = -\mathbf{g}$ to solve coupled vector optimization problems without matrix inversion.
4. **Solver #2: Levenberg-Marquardt (LM) Non-Linear Least Squares Accelerator (`Q16.16`)**: Industry-standard optimizer for Robotics SLAM, sensor calibration, and curve fitting featuring an **adaptive Marquardt damping controller** $(J^T J + \lambda I)\mathbf{p} = -J^T \mathbf{r}$.
5. **Solver #3: Iteratively Reweighted Least Squares (IRLS) Accelerator (`Q16.16`)**: Physical machine learning & classification engine optimizing Logistic Regression and Generalized Linear Models (GLM) via $(X^T W X + \lambda I) \Delta \mathbf{w} = X^T (\mathbf{y} - \mathbf{p})$ with a hardware Sigmoid probability unit.
6. **Solver #4: Quasi-Newton BFGS Accelerator (`Q16.16`)**: High-dimensional optimizer directly computing and updating the **Inverse Hessian Matrix ($B_k \approx H_k^{-1}$)** in silicon via symmetric Rank-2 updates, eliminating matrix inversions and second-derivative computations entirely.
7. **Solver #5: Non-Linear Conjugate Gradient (CG) Accelerator (`Q16.16`)**: Ultra-low-area matrix-free $O(N)$ vector memory solver executing **Polak-Ribière conjugate direction updates with Powell restarts** and directional curvature line search.
8. **Solver #6: Gauss-Newton Non-Linear Least Squares Accelerator (`Q16.16`)**: Second-order non-linear least squares engine solving $(J^T J + \lambda_{\text{eps}} I)\mathbf{p} = -J^T \mathbf{r}$ via direct hardware Cholesky factorization with zero damping overhead.

---

## Key Features & Highlights

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
│   │   ├── bfgs_32bit/                  # Solver #4: BFGS Quasi-Newton Suite
│   │   ├── cg_32bit/                    # Solver #5: Conjugate Gradient Suite
│   │   └── gauss_newton_32bit/          # [NEW] Solver #6: Gauss-Newton Suite
│   │       ├── gn_types_pkg.sv          # Package: vector, observation data types
│   │       ├── gn_helpers.svh           # Inline vector/matrix helper functions
│   │       ├── q16_alu.sv               # Q16.16 ALU
│   │       ├── q16_divider.sv           # 48-cycle Restoring Divider
│   │       ├── q16_sqrt.sv              # 24-cycle Restoring Square Root Engine
│   │       ├── cholesky_solver_engine.sv# Hardware Cholesky linear solver
│   │       ├── dfg_gn_engine.sv         # DFG Model Evaluator f(t_m, x)
│   │       ├── gn_jacobian_engine.sv    # Jacobian J, JᵀJ accumulator, and Jᵀr MAC
│   │       └── gauss_newton_top.sv      # Master Gauss-Newton SoC Controller
│   └── sim/                             # Simulation & Verification Environment
│       ├── Makefile                     # Build & run Makefile (all 8 targets)
│       └── tb_sv/                       # SystemVerilog testbenches
│           ├── tb_newton_2nd_order.sv       # 32-bit 1D testbench
│           ├── tb_newton_2nd_order_64bit.sv # 64-bit 1D testbench
│           ├── tb_newton_multivar.sv        # Multivariable testbench
│           ├── tb_levenberg_marquardt.sv    # Levenberg-Marquardt testbench
│           ├── tb_irls.sv                   # IRLS testbench
│           ├── tb_bfgs.sv                   # BFGS testbench
│           ├── tb_cg.sv                     # Conjugate Gradient testbench
│           └── tb_gauss_newton.sv           # Gauss-Newton testbench
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

9. **Run All Solver Suites**:
   ```bash
   make all
   ```

---

## Verification Test Benchmarks

| Solver | Test Case | Target Optimum ($\mathbf{x}^*$) | Hardware Result | Iterations | Status |
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
