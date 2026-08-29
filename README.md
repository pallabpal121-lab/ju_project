# Universal Newton 2nd-Order Hardware Optimization Accelerator

A high-performance, programmable **Universal Newton 2nd-Order Hardware Optimization Accelerator Suite** implemented in SystemVerilog. Designed for embedded physical AI, robotics trajectory optimization, nonlinear parameter estimation, and scientific computing on FPGA/ASIC platforms.

The suite includes three dedicated silicon architectures:
- **1D 32-Bit Tier (`Q16.16`)**: Lightweight fixed-point architecture for scalar non-linear equations.
- **1D 64-Bit Tier (`Q32.32`)**: High-precision architecture delivering ultra-fine resolution (`2^-32 ≈ 2.328 × 10^-10`) for aerospace and scientific computing.
- **Multivariable N-Dimensional Tier (`Q16.16`)**: Coupled multi-variable optimization engine integrating a hardware **Cholesky decomposition linear system solver** $(H + \lambda I)\mathbf{p} = -\mathbf{g}$ to solve coupled vector optimization problems without matrix inversion.

---

## Key Features & Highlights

- **Universal Programmable Equation Engine**: Evaluates arbitrary mathematical equations without hardware redesign by using an internal microcode processor with dedicated Program Memory and Register Files.
- **Zero-Cost Bit-Shift Calculus**: Computes 1st derivatives (gradients) and 2nd derivatives (Hessian curvature) via numerical finite differences. Step sizes $h = 2^{-4}$ and $h = 2^{-8}$ convert all derivative divisions into single-cycle arithmetic bit-shifts.
- **Hardware Cholesky Linear Solver**: Solves coupled multi-variable linear systems $(H + \lambda I)\mathbf{p} = -\mathbf{g}$ directly in silicon using hardware Cholesky factorization ($A = L \cdot L^T$) with forward and backward substitution.
- **Dedicated Fixed-Point Linear Dividers & Sqrt Units**: Integrates 48-cycle / 96-cycle Radix-2 Restoring Dividers and 24-cycle Restoring Square Root units.
- **Levenberg-Marquardt Damping**: Adds dynamic regularization $\lambda$ to the curvature denominator to guarantee positive definiteness and prevent division by zero near flat or singular regions.
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
│   │   ├── newton_2nd_order_32bit/      # 32-bit Q16.16 1D Accelerator Suite
│   │   │   ├── newton_types_pkg.sv      # Package: Q16.16 types, opcodes, constants
│   │   │   ├── q16_alu.sv               # Single-cycle Q16.16 fixed-point ALU
│   │   │   ├── q16_divider.sv           # 48-cycle Radix-2 restoring divider
│   │   │   ├── dfg_equation_engine.sv   # Programmable microcode DFG processor
│   │   │   ├── derivative_engine.sv     # Finite-difference derivative engine
│   │   │   └── newton_2nd_order_top.sv  # Top-level master optimization SoC
│   │   ├── newton_2nd_order_64bit/      # 64-bit Q32.32 1D Accelerator Suite
│   │   │   ├── newton_types_64bit_pkg.sv# Package: Q32.32 types, opcodes, constants
│   │   │   ├── q32_alu.sv               # Single-cycle Q32.32 fixed-point ALU
│   │   │   ├── q32_divider.sv           # 96-cycle Radix-2 restoring divider
│   │   │   ├── dfg_equation_engine_64bit.sv # 64-register microcode DFG processor
│   │   │   ├── derivative_engine_64bit.sv   # 64-bit finite-difference engine
│   │   │   └── newton_2nd_order_64bit_top.sv# Top-level 64-bit master SoC
│   │   └── newton_multivar_32bit/       # [NEW] Multivariable N-Dimensional Suite
│   │       ├── newton_multivar_pkg.sv   # Package: packed 128-bit vector & 512-bit matrix
│   │       ├── multivar_helpers.svh     # Inline vector and matrix access functions
│   │       ├── q16_alu.sv               # Q16.16 ALU
│   │       ├── q16_divider.sv           # 48-cycle Restoring Divider
│   │       ├── q16_sqrt.sv              # 24-cycle Restoring Square Root Engine
│   │       ├── cholesky_solver_engine.sv# Hardware Cholesky linear solver (A = L·Lᵀ)
│   │       ├── dfg_multivar_engine.sv   # Multi-variable DFG equation evaluator
│   │       ├── multivar_derivative_engine.sv # Multi-point Gradient & Hessian sweeper
│   │       └── newton_multivar_top.sv   # Master Multivariable SoC Accelerator
│   └── sim/                             # Simulation & Verification Environment
│       ├── Makefile                     # Build & run Makefile (32-bit, 64-bit, multivar)
│       └── tb_sv/                       # SystemVerilog testbenches
│           ├── tb_newton_2nd_order.sv       # 32-bit 1D testbench
│           ├── tb_newton_2nd_order_64bit.sv # 64-bit 1D testbench
│           └── tb_newton_multivar.sv        # Multivariable testbench
└── README.md                            # Project documentation
```

---

## Mathematical Architecture & Optimization Flow

### 1. Multivariable Newton's Second-Order Method
The multivariable accelerator iteratively updates a vector of $N$ coupled parameters $\mathbf{x} = [x_0, x_1, \dots, x_{N-1}]^T$:

$$(H + \lambda I) \mathbf{p} = -\mathbf{g}$$

$$\mathbf{x}_{k+1} = \mathbf{x}_k + \alpha \mathbf{p}$$

- $\mathbf{g} = \nabla f(\mathbf{x})$: $N \times 1$ Gradient vector.
- $H = \nabla^2 f(\mathbf{x})$: $N \times N$ symmetric Hessian curvature matrix.
- $\mathbf{p}$: Newton search direction vector.
- $\alpha$: Step size / learning rate.

### 2. Multi-Point Finite-Difference Calculus in Hardware
- **Gradient Vector ($g_i$)**: $g_i = \frac{f(\mathbf{x} + h\mathbf{e}_i) - f(\mathbf{x} - h\mathbf{e}_i)}{2h} = (f_{+i} - f_{-i}) \ll 3$ (for $h = 2^{-4}$).
- **Diagonal Hessian ($H_{ii}$)**: $H_{ii} = \frac{f_{+i} - 2f_0 + f_{-i}}{h^2} = (f_{+i} - 2f_0 + f_{-i}) \ll 8$.
- **Off-Diagonal Hessian ($H_{ij}, i \neq j$)**:
  $$H_{ij} = \frac{f(\mathbf{x} + h\mathbf{e}_i + h\mathbf{e}_j) - f(\mathbf{x} + h\mathbf{e}_i - h\mathbf{e}_j) - f(\mathbf{x} - h\mathbf{e}_i + h\mathbf{e}_j) + f(\mathbf{x} - h\mathbf{e}_i - h\mathbf{e}_j)}{4h^2}$$
  Dividing by $4h^2 = 2^{-6}$ is an arithmetic left shift by 6 (`<<< 6`).

### 3. Hardware Cholesky Decomposition ($A = L \cdot L^T$)
Instead of numerically unstable matrix inversion, the hardware decomposes $A = (H + \lambda I)$ into lower-triangular matrix $L$:
1. **Diagonal**: $L_{ii} = \sqrt{A_{ii} - \sum_{k=0}^{i-1} L_{ik}^2}$ (computed via `q16_sqrt`)
2. **Off-Diagonal**: $L_{ji} = \frac{1}{L_{ii}} \left( A_{ji} - \sum_{k=0}^{i-1} L_{jk} L_{ik} \right)$ (computed via `q16_divider`)
3. **Forward Substitution**: Solves $L \mathbf{y} = -\mathbf{g}$
4. **Backward Substitution**: Solves $L^T \mathbf{p} = \mathbf{y}$

---

## Quickstart: Simulation & Verification

### Running the Simulations
Navigate to the simulation directory:
```bash
cd Playstation/sim
```

1. **Run 32-Bit 1D Optimization Tests**:
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

4. **Run All Suites**:
   ```bash
   make all
   ```

5. **Clean Simulation Artifacts**:
   ```bash
   make clean
   ```

---

## Verification Test Benchmarks

| Architecture | Test Case | Target Optimum ($\mathbf{x}^*$) | Hardware Result | Iterations | Status |
| :--- | :--- | :---: | :---: | :---: | :---: |
| **32-Bit 1D** | $f(x) = (x - 3)^2$ | $x^* = 3.0$ | $x^* = 3.001709$ | 3 | **PASSED** |
| **32-Bit 1D** | $f(x) = x^4 - 4x^2 + 5$ | $x^* = \sqrt{2} \approx 1.4142$ | $x^* = 1.412918$ | 5 | **PASSED** |
| **32-Bit 1D** | $f(x) = x^8 - 4x^4 + 3$ | $x^* = 2^{0.25} \approx 1.1892$ | $x^* = 1.184296$ | 7 | **PASSED** |
| **64-Bit 1D** | $f(x) = (x - 3)^2$ | $x^* = 3.0$ | $x^* = 3.000008$ | 2 | **PASSED** |
| **64-Bit 1D** | $f(x) = x^4 - 4x^2 + 5$ | $x^* = \sqrt{2} \approx 1.414214$ | $x^* = 1.414216$ | 5 | **PASSED** |
| **64-Bit 1D** | $f(x) = x^8 - 4x^4 + 3$ | $x^* = 2^{0.25} \approx 1.189207$ | $x^* = 1.189189$ | 7 | **PASSED** |
| **Multivariable** | $f(x_0, x_1) = (x_0-2)^2 + (x_1-5)^2 + x_0 x_1$ | $(-0.666667, 5.333333)$ | $(-0.666550, 5.333328)$ | 2 | **PASSED** |
| **Multivariable** | $f(x_0, x_1) = 2(x_0-3)^2 + 3(x_1-4)^2$ | $(3.000000, 4.000000)$ | $(3.000061, 3.999985)$ | 2 | **PASSED** |
| **Multivariable** | $f(x_0, x_1, x_2) = (x_0-1)^2 + (x_1-2)^2 + (x_2-3)^2 + x_0 x_1$ | $(0.0, 2.0, 3.0)$ | $(0.000076, 2.000031, 3.000061)$ | 2 | **PASSED** |
