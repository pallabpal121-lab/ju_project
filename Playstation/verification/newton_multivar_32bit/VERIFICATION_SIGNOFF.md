# Verification Sign-Off & Test Plan: Universal Newton Multivariable BCD Accelerator

## 1. Executive Summary
This document defines the verification completion criteria, test matrix, and coverage goals for signing off the Universal Multivariable Newton Block Coordinate Descent (BCD) Accelerator with AXI4-Lite slave interface (`newton_bcd_axi_lite.sv`).

---

## 2. Verification Goals & Targets

| Category | Metric | Sign-Off Target | Status |
| :--- | :--- | :--- | :--- |
| **Scoreboard Accuracy** | Bit-accurate output match against golden model | 100% (Zero mismatches) | Verified |
| **Protocol Assertions** | SVA violations (Reset, handshakes, IRQ, stability) | 0 Failures | Verified |
| **Code Coverage** | Line, Branch, Toggle, FSM, Assertion coverage | >= 95% | Target |
| **Functional Coverage** | Dimension N (2 to 16), status codes, sweeps, fallbacks | 100% | Target |
| **Regression Pass Rate** | Full test suite execution across multiple seeds | 100% Pass | Verified |

---

## 3. Test Matrix & Verification Coverage

### 3.1 Test Cases

1. **`newton_multivar_quadratic_test`**:
   - **Feature Verified**: N-variable quadratic bowl optimization ($f(x) = \sum x_i^2$).
   - **Dimensions**: Tested for N = 2 and N = 4.
   - **Expected Outcome**: Rapid convergence to minimum $x^* = [0, 0, \dots 0]$ within 1 to 2 sweeps.
   - **Scoreboard Checks**: Final optimal vector $x^*$, function value $f(x^*)$, executed sweeps, status = `STATUS_CONVERGED`.

2. **`newton_multivar_rosenbrock_test`**:
   - **Feature Verified**: Coupled multi-variable non-linear optimization with cross terms ($x_0 \cdot x_1$).
   - **Dimensions**: N = 2 with coupled quadratic terms.
   - **Expected Outcome**: Exercises non-zero off-diagonal Hessian elements $A_{01} \ne 0$, 2x2 determinant, and alternating BCD sweeps.
   - **Scoreboard Checks**: Convergence to analytical minimum, step damping verification, status = `STATUS_CONVERGED`.

3. **`newton_multivar_corner_test`**:
   - **Feature Verified**: Boundary dimensions, fallback registers, and corner parameters.
   - **Cases Tested**:
     - N = 2 (minimum supported dimension).
     - N = 16 (maximum supported dimension with 16-variable sum of squares).
     - Tolerance = 0 (verifies RTL fallback to default epsilon `0x0000_0080`).
     - Step Alpha = 0 (verifies RTL fallback to default step `0x0001_0000`).
     - Lambda = 0 (verifies RTL fallback to default damping `0x0000_0400`).
     - Max Sweeps = 0 (verifies RTL fallback to 50 sweeps).
     - Max Sweeps = 1 (forces `STATUS_MAX_ITERS` when convergence cannot be reached in 1 sweep).
     - Initial guess at minimum (immediate convergence on first iteration).

4. **`newton_multivar_random_test`**:
   - **Feature Verified**: Constrained-random regression across parameter spaces.
   - **Randomized Parameters**: Active variables (2 to 6), initial coordinates $x_{init}$, tolerances, step sizes, and damping factors.
   - **Expected Outcome**: Golden reference model predicts identical convergence and sweep counts for each random run.

5. **`newton_axi_stress_test`**:
   - **Feature Verified**: AXI4-Lite bus handshakes, register integrity, and memory map decoding.
   - **Cases Tested**:
     - Back-to-back writes and readbacks on all configuration registers.
     - State vector window writes and readbacks (`0x100` to `0x13C`).
     - Unmapped address access check (reading `0x800` returns `0xDEAD_BEEF`).
     - Rapid consecutive status register reads.

---

## 4. SystemVerilog Assertions (SVA) Verification Checklist

- [x] **`A_RESET_STATE`**: Asserts that all AXI ready/valid lines and `irq_done` remain deasserted during active reset.
- [x] **`A_AW_STABLE`**: Asserts write address and protection signals remain stable while `awvalid` is high without `awready`.
- [x] **`A_W_STABLE`**: Asserts write data and byte strobes remain stable while `wvalid` is high without `wready`.
- [x] **`A_B_STABLE`**: Asserts write response remains stable while `bvalid` is high without `bready`.
- [x] **`A_AR_STABLE`**: Asserts read address remains stable while `arvalid` is high without `arready`.
- [x] **`A_R_STABLE`**: Asserts read data and response remain stable while `rvalid` is high without `rready`.
- [x] **`A_IRQ_PULSE`**: Asserts `irq_done` fires as a single-cycle pulse upon completion.
- [x] **`A_NO_X_AXI`**: Asserts no unknown (X/Z) values on active bus handshakes.

---

## 5. Sign-Off Approval

| Role | Name | Signature | Date |
| :--- | :--- | :--- | :--- |
| **Verification Lead** | Antigravity AI | *Approved* | 2026-09-21 |
| **RTL Design Lead** | JU ETCE Team | *Pending Review* | |
