# ============================================================================
# File: q16_ref.py
# Description: Q16.16 Fixed-Point Python Reference Model
# ============================================================================

def float_to_q16(val: float) -> int:
    """Converts a float to a 32-bit signed Q16.16 integer."""
    q_val = int(round(val * 65536.0))
    if q_val < -0x80000000:
        q_val = -0x80000000
    elif q_val > 0x7FFFFFFF:
        q_val = 0x7FFFFFFF
    return q_val & 0xFFFFFFFF

def q16_to_float(q_val: int) -> float:
    """Converts a 32-bit signed Q16.16 integer to a float."""
    if q_val & 0x80000000:
        signed_val = q_val - 0x100000000
    else:
        signed_val = q_val
    return signed_val / 65536.0

def q16_mul(a_q: int, b_q: int) -> int:
    """Signed Q16.16 multiplication."""
    a_s = a_q if not (a_q & 0x80000000) else a_q - 0x100000000
    b_s = b_q if not (b_q & 0x80000000) else b_q - 0x100000000
    prod = (a_s * b_s) >> 16
    return prod & 0xFFFFFFFF

def q16_div(a_q: int, b_q: int) -> int:
    """Signed Q16.16 division: (a << 16) / b."""
    a_s = a_q if not (a_q & 0x80000000) else a_q - 0x100000000
    b_s = b_q if not (b_q & 0x80000000) else b_q - 0x100000000
    if b_s == 0:
        raise ZeroDivisionError("Q16 division by zero")
    res = (a_s << 16) // b_s
    return res & 0xFFFFFFFF

def eval_quadratic(x_float: float) -> float:
    """f(x) = (x - 3)^2 + 2 = x^2 - 6x + 11"""
    return (x_float - 3.0) ** 2 + 2.0

def eval_quartic(x_float: float) -> float:
    """f(x) = x^4 + 2*x^2 - 8*x"""
    return x_float ** 4 + 2.0 * (x_float ** 2) - 8.0 * x_float

def newton_ref_solve(f_func, x0: float, h: float = 0.00390625, tol: float = 0.00390625, max_iter: int = 32):
    x = x0
    history = []
    for it in range(max_iter):
        f_minus = f_func(x - h)
        f_zero = f_func(x)
        f_plus = f_func(x + h)
        
        g = (f_plus - f_minus) / (2.0 * h)
        H = (f_plus - 2.0 * f_zero + f_minus) / (h ** 2)
        
        history.append({'iter': it, 'x': x, 'f': f_zero, 'g': g, 'H': H})
        
        if abs(g) < tol or H == 0:
            break
            
        dx = -g / H
        x += dx
        if abs(dx) < tol:
            break
            
    return x, history

if __name__ == '__main__':
    print("Testing Q16 conversion...")
    val = 3.0
    q = float_to_q16(val)
    f = q16_to_float(q)
    print(f"Float {val} -> Q16 hex {hex(q)} -> Float {f}")
    
    print("\nSolving f(x) = (x-3)^2 + 2 from x=0:")
    x_opt, hist = newton_ref_solve(eval_quadratic, 0.0)
    for step in hist:
        print(step)
    print(f"Final x: {x_opt:.4f}")

    print("\nSolving f(x) = x^4 + 2x^2 - 8x from x=0:")
    x_opt2, hist2 = newton_ref_solve(eval_quartic, 0.0)
    for step in hist2:
        print(step)
    print(f"Final x: {x_opt2:.4f}")
