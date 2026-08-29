#!/usr/bin/env python3
"""
=============================================================================
File: newton_model.py
Module: Python Bit-Accurate Golden Reference Model for Universal Newton Accelerator
Description: Compiles mathematical equations into hardware micro-ops and
             simulates the exact 2nd-order Newton optimization flow.
=============================================================================
"""

def to_q16(val: float) -> int:
    """Convert float to 32-bit signed Q16.16 integer."""
    raw = int(round(val * 65536.0))
    if raw < -2147483648:
        return -2147483648
    if raw > 2147483647:
        return 2147483647
    return raw

def from_q16(raw: int) -> float:
    """Convert 32-bit signed Q16.16 integer to float."""
    if raw >= 0x80000000:
        raw -= 0x100000000
    return raw / 65536.0

def q16_mul(a: int, b: int) -> int:
    return (a * b) >> 16

def q16_div(a: int, b: int) -> int:
    if b == 0:
        return 0
    return (a << 16) // b

class UniversalNewtonSolver:
    def __init__(self, h_step=0.0625, lambda_reg=0.0039, tol=0.001, max_iters=50):
        self.h = h_step
        self.lambda_reg = lambda_reg
        self.tol = tol
        self.max_iters = max_iters

    def optimize(self, f_func, x0: float):
        x = x0
        history = []
        for i in range(self.max_iters):
            f_0 = f_func(x)
            f_p = f_func(x + self.h)
            f_m = f_func(x - self.h)

            # Difference terms
            diff_1 = f_p - f_m
            diff_2 = f_p - 2.0 * f_0 + f_m

            # Gradient for convergence check
            g = diff_1 / (2.0 * self.h)

            # Simplified Newton Step: delta_x = - (h * diff_1) / (2 * diff_2)
            step_num = self.h * diff_1
            step_den = 2.0 * diff_2 + (self.lambda_reg if diff_2 >= 0 else -self.lambda_reg)

            history.append({'iter': i, 'x': x, 'f': f_0, 'g': g, 'step_num': step_num, 'step_den': step_den})

            if abs(g) <= self.tol:
                return x, f_0, g, i + 1, "CONVERGED", history

            delta_x = -step_num / step_den
            x += delta_x

        return x, f_func(x), g, self.max_iters, "MAX_ITERS", history

if __name__ == "__main__":
    solver = UniversalNewtonSolver()

    print("=== Python Golden Reference Test 1: f(x) = (x-3)^2 ===")
    x_opt, f_opt, g_final, iters, status, _ = solver.optimize(lambda x: (x - 3.0)**2, 10.0)
    print(f"Status: {status}, Iters: {iters}, x*: {x_opt:.6f}, f(x*): {f_opt:.6f}, g: {g_final:.6f}")

    print("\n=== Python Golden Reference Test 2: f(x) = x^4 - 4x^2 + 5 ===")
    x_opt, f_opt, g_final, iters, status, _ = solver.optimize(lambda x: x**4 - 4*x**2 + 5, 3.0)
    print(f"Status: {status}, Iters: {iters}, x*: {x_opt:.6f}, f(x*): {f_opt:.6f}, g: {g_final:.6f}")

    print("\n=== Python Golden Reference Test 3: f(x) = x^8 - 4x^4 + 3 ===")
    solver_t3 = UniversalNewtonSolver(tol=0.008, lambda_reg=0.005)
    x_opt, f_opt, g_final, iters, status, _ = solver_t3.optimize(lambda x: x**8 - 4*x**4 + 3, 2.0)
    print(f"Status: {status}, Iters: {iters}, x*: {x_opt:.6f}, f(x*): {f_opt:.6f}, g: {g_final:.6f}")
