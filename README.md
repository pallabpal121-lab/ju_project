# Physical AI Math Hardware Optimization Accelerator Suite

A high-performance, programmable **Hardware Optimization Accelerator Suite** implemented in SystemVerilog. Designed for embedded physical AI, robotics SLAM, trajectory optimization, nonlinear parameter estimation, and scientific computing on FPGA/ASIC platforms.

The suite includes four specialized hardware architectures:
1. **Solver #1A: 1D 32-Bit Newton Accelerator (`Q16.16`)**: Lightweight fixed-point architecture for scalar non-linear equations.
2. **Solver #1B: 1D 64-Bit Newton Accelerator (`Q32.32`)**: High-precision architecture delivering ultra-fine resolution (`2^-32 ≈ 2.328 × 10^-10`) for aerospace and scientific computing.
3. **Solver #1C: Multivariable N-Dimensional Newton Accelerator (`Q16.16`)**: Coupled multi-variable optimization engine integrating a hardware **Cholesky decomposition linear system solver** $(H + \lambda I)\mathbf{p} = -\mathbf{g}$ to solve coupled vector optimization problems without matrix inversion.
4. **Solver #2: Levenberg-Marquardt (LM) Non-Linear Least Squares Accelerator (`Q16.16`)**: Industry-standard optimizer for Robotics SLAM, sensor calibration, and curve fitting featuring an **adaptive Marquardt damping controller** $(J^T J + \lambda I)\mathbf{p} = -J^T \mathbf{r}$.

---

## Key Features & Highlights

- **Universal Programmable Equation Engine**: Evaluates arbitrary mathematical equations without hardware redesign by using an internal microcode processor with dedicated Program Memory and Register Files.
- **Zero-Cost Bit-Shift Calculus**: Computes gradients and Hessian curvatures via numerical finite differences. Step sizes $h = 2^{-4}$ and $h = 2^{-8}$ convert all derivative divisions into single-cycle arithmetic bit-shifts.
- **Hardware Cholesky Linear Solver**: Solves coupled multi-variable linear systems $(H + \lambda I)\mathbf{p} = -\mathbf{g}$ and $(J^T J + \lambda I)\mathbf{p} = -J^T \mathbf{r}$ directly in silicon using hardware Cholesky factorization ($A = L \cdot L^T$) with forward and backward substitution.
- **Adaptive Marquardt Damping Schedule**: Dynamically transitions between fast second-order Gauss-Newton and robust first-order Gradient Descent depending on step success ($S(\mathbf{x}_{\text{trial}}) < S(\mathbf{x})$).
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
│   │   └── levenberg_marquardt_32bit/   # [NEW] Solver #2: Levenberg-Marquardt Suite
│   │       ├── lm_types_pkg.sv          # Package: observation dataset & LM types
│   │       ├── lm_helpers.svh           # Inline vector/matrix helper functions
│   │       ├── q16_alu.sv               # Q16.16 ALU
│   │       ├── q16_divider.sv           # 48-cycle Restoring Divider
│   │       ├── q16_sqrt.sv              # 24-cycle Restoring Square Root Engine
│   │       ├── cholesky_solver_engine.sv# Hardware Cholesky linear solver
│   │       ├── dfg_lm_engine.sv         # DFG Model Evaluator f(t_m, x)
│   │       ├── lm_jacobian_engine.sv    # Jacobian J, JᵀJ outer-product, and Jᵀr MAC
│   │       └── levenberg_marquardt_top.sv # Master LM SoC with adaptive λ damping
│   └── sim/                             # Simulation & Verification Environment
│       ├── Makefile                     # Build & run Makefile (all 4 targets)
│       └── tb_sv/                       # SystemVerilog testbenches
│           ├── tb_newton_2nd_order.sv       # 32-bit 1D testbench
│           ├── tb_newton_2nd_order_64bit.sv # 64-bit 1D testbench
│           ├── tb_newton_multivar.sv        # Multivariable testbench
│           └── tb_levenberg_marquardt.sv    # Levenberg-Marquardt testbench
└── README.md                            # Project documentation
```

---

## Mathematical Architecture & Optimization Flow

### 1. Levenberg-Marquardt (LM) Algorithm
The LM accelerator solves non-linear least squares problems: $\min_{\mathbf{x}} \frac{1}{2} \sum_{m=0}^{M-1} (f(t_m, \mathbf{x}) - y_m)^2$.

$$(J^T J + \lambda I) \mathbf{p} = -J^T \mathbf{r}$$

- $\mathbf{r} = [r_0, \dots, r_{M-1}]^T$: $M \times 1$ Residual vector ($r_m = f(t_m, \mathbf{x}) - y_m$).
- $J$: $M \times N$ Jacobian matrix ($J_{mn} = \frac{\partial r_m}{\partial x_n}$).
- $J^T J$: $N \times N$ approximated Hessian matrix.
- $J^T \mathbf{r}$: $N \times 1$ gradient vector.
- $\lambda$: Adaptive Marquardt damping parameter.

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

5. **Run All Solver Suites**:
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
