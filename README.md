# Physical AI Math Hardware Optimization Accelerator Suite

A high-performance, programmable **Hardware Optimization Accelerator Suite** implemented in SystemVerilog. Designed for embedded physical AI, robotics SLAM, trajectory optimization, nonlinear parameter estimation, classification, and scientific computing on FPGA/ASIC platforms.

The suite includes six specialized hardware architectures:
1. **Solver #1A: 1D 32-Bit Newton Accelerator (`Q16.16`)**: Lightweight fixed-point architecture for scalar non-linear equations.
2. **Solver #1B: 1D 64-Bit Newton Accelerator (`Q32.32`)**: High-precision architecture delivering ultra-fine resolution (`2^-32 ≈ 2.328 × 10^-10`) for aerospace and scientific computing.
3. **Solver #1C: Multivariable N-Dimensional Newton Accelerator (`Q16.16`)**: Coupled multi-variable optimization engine integrating a hardware **Cholesky decomposition linear system solver** $(H + \lambda I)\mathbf{p} = -\mathbf{g}$ to solve coupled vector optimization problems without matrix inversion.
4. **Solver #2: Levenberg-Marquardt (LM) Non-Linear Least Squares Accelerator (`Q16.16`)**: Industry-standard optimizer for Robotics SLAM, sensor calibration, and curve fitting featuring an **adaptive Marquardt damping controller** $(J^T J + \lambda I)\mathbf{p} = -J^T \mathbf{r}$.
5. **Solver #3: Iteratively Reweighted Least Squares (IRLS) Accelerator (`Q16.16`)**: Physical machine learning & classification engine optimizing Logistic Regression and Generalized Linear Models (GLM) via $(X^T W X + \lambda I) \Delta \mathbf{w} = X^T (\mathbf{y} - \mathbf{p})$ with a hardware Sigmoid probability unit.
6. **Solver #4: Quasi-Newton BFGS Accelerator (`Q16.16`)**: High-dimensional optimizer directly computing and updating the **Inverse Hessian Matrix ($B_k \approx H_k^{-1}$)** in silicon via symmetric Rank-2 updates, eliminating matrix inversions and second-derivative computations entirely.

---

## Key Features & Highlights

- **Universal Programmable Equation Engine**: Evaluates arbitrary mathematical equations without hardware redesign by using an internal microcode processor with dedicated Program Memory and Register Files.
- **Quasi-Newton Rank-2 Inverse Hessian Updates**: Direct hardware accumulation of $B_{k+1} = B_k + \gamma_1(\mathbf{s} \mathbf{s}^T) - \gamma_2(\mathbf{s} \mathbf{u}^T + \mathbf{u} \mathbf{s}^T)$, evaluating search directions $\mathbf{p} = -B \mathbf{g}$ with zero matrix inversions.
- **Hardware Sigmoid Activation Engine**: Single-cycle / pipelined Q16.16 Sigmoid evaluator $\sigma(\eta) = \frac{1}{1 + e^{-\eta}}$ using symmetric piecewise linear spline interpolation.
- **Hardware Cholesky Linear Solver**: Solves coupled multi-variable linear systems $(H + \lambda I)\mathbf{p} = -\mathbf{g}$, $(J^T J + \lambda I)\mathbf{p} = -J^T \mathbf{r}$, and $(X^T W X + \lambda I)\Delta \mathbf{w} = X^T (\mathbf{y} - \mathbf{p})$ directly in silicon using hardware Cholesky factorization ($A = L \cdot L^T$) with forward and backward substitution.
- **Zero-Cost Bit-Shift Calculus**: Computes gradients and Hessian curvatures via numerical finite differences. Step sizes $h = 2^{-4}$ and $h = 2^{-8}$ convert all derivative divisions into single-cycle arithmetic bit-shifts.
- **Dedicated Fixed-Point Linear Dividers & Sqrt Units**: Integrates 48-cycle / 96-cycle Radix-2 Restoring Dividers and 24-cycle Restoring Square Root units.
- **Bit-Accurate Python Model**: Includes a Python golden reference model (`Playstation/models/newton_model.py`) for cross-verifying simulation results.

---

## Repository Structure

```
ju_project/
├── Documents/                           # Research papers, presentations, and architecture specs
├── Playstation/                         # Hardware accelerator workspace
│   ├── models/                          # Golden reference software models
│   │   └── newton_model.py              # Bit-accurate Python simulation model
│   ├── rtl/                             # SystemVerilog RTL source code
│   │   ├── newton_2nd_order_32bit/      # Solver #1A: 32-bit Q16.16 1D Accelerator
│   │   │   ├── newton_types_pkg.sv      # Package: Q16.16 types, opcodes, constants
│   │   │   ├── q16_alu.sv               # Single-cycle Q16.16 fixed-point ALU
│   │   │   ├── q16_divider.sv           # 48-cycle Radix-2 restoring divider
│   │   │   ├── dfg_equation_engine.sv   # Programmable microcode DFG processor
│   │   │   ├── derivative_engine.sv     # Finite-difference derivative engine
│   │   │   └── newton_2nd_order_top.sv  # Top-level master optimization SoC
│   │   ├── newton_2nd_order_64bit/      # Solver #1B: 64-bit Q32.32 1D Accelerator
│   │   │   ├── newton_types_64bit_pkg.sv# Package: Q32.32 types, opcodes, constants
│   │   │   ├── q32_alu.sv               # Single-cycle Q32.32 fixed-point ALU
│   │   │   ├── q32_divider.sv           # 96-cycle Radix-2 restoring divider
│   │   │   ├── dfg_equation_engine_64bit.sv # 64-register microcode DFG processor
│   │   │   ├── derivative_engine_64bit.sv   # 64-bit finite-difference engine
│   │   │   └── newton_2nd_order_64bit_top.sv# Top-level 64-bit master SoC
│   │   ├── newton_multivar_32bit/       # Solver #1C: Multivariable N-Dimensional Suite
│   │   │   ├── newton_multivar_pkg.sv   # Package: packed 128-bit vector & 512-bit matrix
│   │   │   ├── multivar_helpers.svh     # Inline vector and matrix access functions
│   │   │   ├── q16_alu.sv               # Q16.16 ALU
│   │   │   ├── q16_divider.sv           # 48-cycle Restoring Divider
│   │   │   ├── q16_sqrt.sv              # 24-cycle Restoring Square Root Engine
│   │   │   ├── cholesky_solver_engine.sv# Hardware Cholesky linear solver (A = L·Lᵀ)
│   │   │   ├── dfg_multivar_engine.sv   # Multi-variable DFG equation evaluator
│   │   │   ├── multivar_derivative_engine.sv # Multi-point Gradient & Hessian sweeper
│   │   │   └── newton_multivar_top.sv   # Master Multivariable SoC Accelerator
│   │   ├── levenberg_marquardt_32bit/   # Solver #2: Levenberg-Marquardt Suite
│   │   │   ├── lm_types_pkg.sv          # Package: observation dataset & LM types
│   │   │   ├── lm_helpers.svh           # Inline vector/matrix helper functions
│   │   │   ├── q16_alu.sv               # Q16.16 ALU
│   │   │   ├── q16_divider.sv           # 48-cycle Restoring Divider
│   │   │   ├── q16_sqrt.sv              # 24-cycle Restoring Square Root Engine
│   │   │   ├── cholesky_solver_engine.sv# Hardware Cholesky linear solver
│   │   │   ├── dfg_lm_engine.sv         # DFG Model Evaluator f(t_m, x)
│   │   │   ├── lm_jacobian_engine.sv    # Jacobian J, JᵀJ outer-product, and Jᵀr MAC
│   │   │   └── levenberg_marquardt_top.sv # Master LM SoC with adaptive λ damping
│   │   ├── irls_32bit/                  # Solver #3: IRLS Suite
│   │   │   ├── irls_types_pkg.sv        # Package: feature matrix, labels, weight vector
│   │   │   ├── irls_helpers.svh         # Inline helper functions
│   │   │   ├── q16_alu.sv               # Q16.16 ALU
│   │   │   ├── q16_divider.sv           # 48-cycle Restoring Divider
│   │   │   ├── q16_sqrt.sv              # 24-cycle Restoring Square Root Engine
│   │   │   ├── q16_sigmoid.sv           # Hardware Q16.16 Sigmoid Activation Unit
│   │   │   ├── cholesky_solver_engine.sv# Hardware Cholesky linear solver
│   │   │   ├── irls_weight_grad_engine.sv # Computes η, p, W_mm, XᵀWX, and Xᵀ(y-p)
│   │   │   └── irls_top.sv              # Master IRLS SoC for classification
│   │   └── bfgs_32bit/                  # [NEW] Solver #4: BFGS Quasi-Newton Suite
│   │       ├── bfgs_types_pkg.sv        # Package: vector, matrix, microcode opcodes
│   │       ├── bfgs_helpers.svh         # Inline helper functions
│   │       ├── q16_alu.sv               # Q16.16 ALU
│   │       ├── q16_divider.sv           # 48-cycle Restoring Divider
│   │       ├── dfg_bfgs_engine.sv       # Programmable DFG Model Evaluator
│   │       ├── bfgs_gradient_engine.sv  # Finite-difference gradient sweeper ∇f(x)
│   │       ├── bfgs_matrix_update_engine.sv # Rank-2 Inverse Hessian update unit
│   │       └── bfgs_top.sv              # Master BFGS SoC Accelerator
│   └── sim/                             # Simulation & Verification Environment
│       ├── Makefile                     # Build & run Makefile (all 6 targets)
│       └── tb_sv/                       # SystemVerilog testbenches
│           ├── tb_newton_2nd_order.sv       # 32-bit 1D testbench
│           ├── tb_newton_2nd_order_64bit.sv # 64-bit 1D testbench
│           ├── tb_newton_multivar.sv        # Multivariable testbench
│           ├── tb_levenberg_marquardt.sv    # Levenberg-Marquardt testbench
│           ├── tb_irls.sv                   # IRLS testbench
│           └── tb_bfgs.sv                   # BFGS testbench
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

7. **Run All Solver Suites**:
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
