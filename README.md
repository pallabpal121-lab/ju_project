# Universal Newton 2nd-Order Hardware Optimization Accelerator

A high-performance, programmable **Universal Newton 2nd-Order Hardware Optimization Accelerator** implemented in SystemVerilog. Designed for embedded physical AI, robotics trajectory optimization, nonlinear parameter estimation, and scientific computing on FPGA/ASIC platforms.

The accelerator is implemented in two precision tiers:
- **32-Bit Tier (`Q16.16`)**: Standard fixed-point architecture optimized for low-area, high-throughput embedded edge devices.
- **64-Bit Tier (`Q32.32`)**: High-precision architecture delivering ultra-fine resolution (`2^-32 ≈ 2.328 × 10^-10`) for industry-standard scientific and aerospace workloads.

---

## Key Features & Highlights

- **Universal Programmable Equation Engine**: Evaluates arbitrary mathematical equations without hardware redesign by using an internal microcode processor with dedicated Program Memory and Register Files.
- **Zero-Cost Bit-Shift Calculus**: Computes 1st derivatives (gradients) and 2nd derivatives (Hessian curvature) via 3-point numerical finite differences. By selecting step sizes $h = 2^{-4}$ (32-bit) and $h = 2^{-8}$ (64-bit), all derivative divisions are converted into single-cycle arithmetic bit-shifts.
- **Dedicated Fixed-Point Linear Solver**: Integrates 48-cycle (32-bit) and 96-cycle (64-bit) Radix-2 Restoring Dividers to compute the Newton step $\Delta x = -\frac{g(x)}{H(x)}$ in silicon.
- **Levenberg-Marquardt Damping**: Adds dynamic regularization $\lambda$ to the curvature denominator to prevent division by zero near flat or singular regions.
- **Bit-Accurate Python Model**: Includes a Python golden reference model (`Playstation/models/newton_model.py`) for compiling equations and cross-verifying simulation results.

---

## Repository Structure

```
ju_project/
├── Documents/                           # Research papers, presentations, and architecture specs
├── Playstation/                         # Hardware accelerator workspace
│   ├── models/                          # Golden reference software models
│   │   └── newton_model.py              # Bit-accurate Python simulation model
│   ├── rtl/                             # SystemVerilog RTL source code
│   │   ├── newton_2nd_order_32bit/      # 32-bit Q16.16 Accelerator Suite
│   │   │   ├── newton_types_pkg.sv      # Package: Q16.16 types, opcodes, constants
│   │   │   ├── q16_alu.sv               # Single-cycle Q16.16 fixed-point ALU
│   │   │   ├── q16_divider.sv           # 48-cycle Radix-2 restoring divider
│   │   │   ├── dfg_equation_engine.sv   # Programmable microcode DFG processor
│   │   │   ├── derivative_engine.sv     # Finite-difference derivative engine
│   │   │   └── newton_2nd_order_top.sv  # Top-level master optimization SoC
│   │   └── newton_2nd_order_64bit/      # 64-bit Q32.32 Accelerator Suite
│   │       ├── newton_types_64bit_pkg.sv# Package: Q32.32 types, opcodes, constants
│   │       ├── q32_alu.sv               # Single-cycle Q32.32 fixed-point ALU
│   │       ├── q32_divider.sv           # 96-cycle Radix-2 restoring divider
│   │       ├── dfg_equation_engine_64bit.sv # 64-register microcode DFG processor
│   │       ├── derivative_engine_64bit.sv   # 64-bit finite-difference engine
│   │       └── newton_2nd_order_64bit_top.sv# Top-level 64-bit master SoC
│   └── sim/                             # Simulation & Verification Environment
│       ├── Makefile                     # Build & run Makefile (32-bit & 64-bit targets)
│       └── tb_sv/                       # SystemVerilog testbenches
│           ├── tb_newton_2nd_order.sv       # 32-bit testbench
│           └── tb_newton_2nd_order_64bit.sv # 64-bit testbench
└── README.md                            # Project documentation
```

---

## Mathematical Architecture & Optimization Flow

### 1. Newton's Second-Order Method
The accelerator iteratively finds the local minimum / root $x^*$ of a function $f(x)$ using the update rule:

$$x_{k+1} = x_k - \alpha \cdot \frac{f'(x_k)}{f''(x_k)} = x_k - \alpha \cdot \frac{g(x_k)}{H(x_k)}$$

### 2. 3-Point Numerical Finite Differences
The derivative engine samples the function at three points:
1. $f_0 = f(x)$
2. $f_+ = f(x + h)$
3. $f_- = f(x - h)$

The difference terms are:
- $\Delta_1 = f_+ - f_-$
- $\Delta_2 = f_+ - 2f_0 + f_-$

From calculus:
- Gradient: $g(x) \approx \frac{\Delta_1}{2h}$
- Hessian: $H(x) \approx \frac{\Delta_2}{h^2}$

### 3. Simplified Hardware Newton Step
Combining the terms into the Newton update:

$$\Delta x = -\frac{g(x)}{H(x)} = -\frac{\Delta_1 / (2h)}{\Delta_2 / h^2} = -\frac{h \cdot \Delta_1}{2 \cdot \Delta_2}$$

| Computation | 32-Bit Hardware (`h = 2^-4 = 0.0625`) | 64-Bit Hardware (`h = 2^-8 = 0.00390625`) |
| :--- | :--- | :--- |
| **Numerator ($h \cdot \Delta_1$)** | `diff_1st >>> 4` (Shift right 4) | `diff_1st >>> 8` (Shift right 8) |
| **Denominator ($2 \cdot \Delta_2 \pm \lambda$)** | `(diff_2nd <<< 1) ± lambda` | `(diff_2nd <<< 1) ± lambda` |
| **Gradient ($g(x) = \Delta_1 / 2h$)** | `diff_1st <<< 3` (Multiply by 8) | `diff_1st <<< 7` (Multiply by 128) |

---

## Microcode Instruction Set Architecture (ISA)

Equations are encoded into 32-bit or 64-bit microcode words and executed by the internal DFG engine:

### Opcode Table:
| Opcode | Name | Operation | Description |
| :---: | :--- | :--- | :--- |
| `4'd0` | `OP_NOP` | No-op | Does nothing |
| `4'd1` | `OP_ADD` | `r[dst] = r[src_a] + r[src_b]` | Signed addition |
| `4'd2` | `OP_SUB` | `r[dst] = r[src_a] - r[src_b]` | Signed subtraction |
| `4'd3` | `OP_MUL` | `r[dst] = (r[src_a] * r[src_b]) >> FracBits` | Fixed-point multiplication |
| `4'd4` | `OP_DIV` | `r[dst] = (r[src_a] << FracBits) / r[src_b]` | Multi-cycle restoring division |
| `4'd5` | `OP_NEG` | `r[dst] = -r[src_a]` | Two's complement negation |
| `4'd6` | `OP_MOV` | `r[dst] = r[src_a]` | Register copy |
| `4'd7` | `OP_LOADC` | `r[dst] = {imm, 0...}` | Formats integer immediate to fixed-point |
| `4'd8` | `OP_END` | Output `r[REG_RESULT]` | Completes evaluation |

### Micro-Instruction Formats:

#### 32-Bit Instruction Word (`instr_t`):
```
 31        28 27        24 23        20 19        16 15                         0
+------------+------------+------------+------------+-------------------------------+
|  op (4b)   |  dst (4b)  | src_a (4b) | src_b (4b) |          imm (16b)            |
+------------+------------+------------+------------+-------------------------------+
```

#### 64-Bit Instruction Word (`instr64_t`):
```
 63        60 59        54 53        48 47        42 41        32 31                0
+------------+------------+------------+------------+------------+------------------+
|  op (4b)   |  dst (6b)  | src_a (6b) | src_b (6b) |  rsrv (10b)|     imm (32b)    |
+------------+------------+------------+------------+------------+------------------+
```

---

## Quickstart: Simulation & Verification

### Prerequisites
- **Icarus Verilog** (`iverilog` version 10.0+ / 12.0+)
- **vvp** runtime engine
- **Make**

### Running the Simulations
Navigate to the simulation directory:
```bash
cd Playstation/sim
```

1. **Run 32-Bit Optimization Tests**:
   ```bash
   make run_32bit
   ```

2. **Run 64-Bit High-Precision Tests**:
   ```bash
   make run_64bit
   ```

3. **Run All Suites**:
   ```bash
   make all
   ```

4. **Clean Simulation Artifacts**:
   ```bash
   make clean
   ```

---

## Verification Test Cases

Both accelerators are verified on non-linear benchmarks:

| Test Case | Target Minimum ($x^*$) | Expected $f(x^*)$ | 32-Bit Convergence | 64-Bit Convergence |
| :--- | :---: | :---: | :---: | :---: |
| **Test 1**: $f(x) = (x - 3)^2$ | $x^* = 3.0$ | $0.0$ | $x^* = 3.001709$ (3 iters) | $x^* = 3.000008$ (2 iters) |
| **Test 2**: $f(x) = x^4 - 4x^2 + 5$ | $x^* = \sqrt{2} \approx 1.4142$ | $1.0$ | $x^* = 1.412918$ (5 iters) | $x^* = 1.414216$ (5 iters) |
| **Test 3**: $f(x) = x^8 - 4x^4 + 3$ | $x^* = 2^{0.25} \approx 1.1892$ | $-1.0$ | $x^* = 1.184296$ (7 iters) | $x^* = 1.189189$ (7 iters) |

---

## Python Golden Reference Model

To run the Python reference solver:
```bash
python3 Playstation/models/newton_model.py
```
Outputs bit-accurate optimization trajectories and mathematical verification logs.
