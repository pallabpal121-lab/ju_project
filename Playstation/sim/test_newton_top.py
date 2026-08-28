# ============================================================================
# File: test_newton_top.py
# Description: Cocotb Testbench for Q16.16 Newton Hardware Accelerator
# Compatible with 64-bit packed DFG instruction words
# ============================================================================

import cocotb
from cocotb.triggers import RisingEdge, ClockCycles, Timer
from cocotb.clock import Clock
import q16_ref

OP_NOP        = 0
OP_LOAD_CONST = 1
OP_LOAD_X     = 2
OP_ADD        = 3
OP_SUB        = 4
OP_MUL        = 5
OP_END        = 15

def encode_instr(opcode, dst=0, srcA=0, srcB=0, imm=0):
    val = (opcode & 0xF) << 60
    val |= (dst & 0x1F) << 55
    val |= (srcA & 0x1F) << 50
    val |= (srcB & 0x1F) << 45
    val |= (imm & 0xFFFFFFFF)
    return val

async def reset_dut(dut):
    dut.rst_n.value = 0
    dut.start.value = 0
    dut.prog_write_en.value = 0
    dut.prog_addr.value = 0
    dut.prog_instr.value = 0
    dut.prog_len_in.value = 0
    dut.x_init.value = 0
    dut.h_step.value = q16_ref.float_to_q16(0.00390625)
    dut.eps_tol.value = q16_ref.float_to_q16(0.00390625)
    dut.max_iter.value = 32
    await ClockCycles(dut.clk, 5)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)

async def load_dfg_program(dut, instructions):
    prog_len = len(instructions)
    for addr, instr in enumerate(instructions):
        dut.prog_write_en.value = 1
        dut.prog_addr.value = addr
        dut.prog_instr.value = instr
        dut.prog_len_in.value = prog_len
        await RisingEdge(dut.clk)
    dut.prog_write_en.value = 0
    await RisingEdge(dut.clk)

@cocotb.test()
async def test_quadratic_optimization(dut):
    """Test 1: f(x) = (x-3)^2 + 2 starting at x=0. Hardware should converge to x=3"""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    dut._log.info("--- Loading DFG Program 1: f(x) = (x-3)^2 + 2 ---")
    prog1 = [
        encode_instr(OP_LOAD_X, dst=0),
        encode_instr(OP_LOAD_CONST, dst=1, imm=q16_ref.float_to_q16(3.0)),
        encode_instr(OP_LOAD_CONST, dst=2, imm=q16_ref.float_to_q16(2.0)),
        encode_instr(OP_SUB, dst=3, srcA=0, srcB=1), # R3 = x - 3
        encode_instr(OP_MUL, dst=4, srcA=3, srcB=3), # R4 = (x-3)^2
        encode_instr(OP_ADD, dst=5, srcA=4, srcB=2), # R5 = (x-3)^2 + 2
        encode_instr(OP_END, srcA=5)
    ]
    await load_dfg_program(dut, prog1)

    dut.x_init.value = q16_ref.float_to_q16(0.0)
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    dut._log.info("Waiting for Newton hardware optimization to complete...")
    timeout_cycles = 5000
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if dut.done.value == 1:
            break
    
    assert dut.done.value == 1, "Hardware optimization timed out!"

    x_final_float = q16_ref.q16_to_float(int(dut.x_final.value))
    f_final_float = q16_ref.q16_to_float(int(dut.f_final.value))
    g_final_float = q16_ref.q16_to_float(int(dut.g_final.value))
    H_final_float = q16_ref.q16_to_float(int(dut.H_final.value))
    status_val    = int(dut.status.value)
    iter_val      = int(dut.iter_count.value)

    dut._log.info(f"Results for f(x) = (x-3)^2 + 2:")
    dut._log.info(f"  Status    : {status_val} (2 = SUCCESS)")
    dut._log.info(f"  Iterations: {iter_val}")
    dut._log.info(f"  x_final   : {x_final_float:.4f} (Expected: 3.0000)")
    dut._log.info(f"  f_final   : {f_final_float:.4f} (Expected: 2.0000)")
    dut._log.info(f"  g_final   : {g_final_float:.4f} (Expected: 0.0000)")
    dut._log.info(f"  H_final   : {H_final_float:.4f} (Expected: 2.0000)")

    assert status_val == 2, f"Expected STATUS_SUCCESS (2), got {status_val}"
    assert abs(x_final_float - 3.0) < 0.01, f"x_final {x_final_float} not close to 3.0"
    assert abs(f_final_float - 2.0) < 0.01, f"f_final {f_final_float} not close to 2.0"

@cocotb.test()
async def test_quartic_optimization(dut):
    """Test 2: f(x) = x^4 + 2x^2 - 8x starting at x=0. Hardware should converge to x=1.0"""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    dut._log.info("--- Loading DFG Program 2: f(x) = x^4 + 2x^2 - 8x ---")
    prog2 = [
        encode_instr(OP_LOAD_X, dst=0),
        encode_instr(OP_LOAD_CONST, dst=1, imm=q16_ref.float_to_q16(2.0)),
        encode_instr(OP_LOAD_CONST, dst=2, imm=q16_ref.float_to_q16(8.0)),
        encode_instr(OP_MUL, dst=3, srcA=0, srcB=0), # R3 = x^2
        encode_instr(OP_MUL, dst=4, srcA=3, srcB=3), # R4 = x^4
        encode_instr(OP_MUL, dst=5, srcA=1, srcB=3), # R5 = 2x^2
        encode_instr(OP_MUL, dst=6, srcA=2, srcB=0), # R6 = 8x
        encode_instr(OP_ADD, dst=7, srcA=4, srcB=5), # R7 = x^4 + 2x^2
        encode_instr(OP_SUB, dst=8, srcA=7, srcB=6), # R8 = x^4 + 2x^2 - 8x
        encode_instr(OP_END, srcA=8)
    ]
    await load_dfg_program(dut, prog2)

    dut.x_init.value = q16_ref.float_to_q16(0.0)
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    dut._log.info("Waiting for Newton hardware optimization to complete...")
    timeout_cycles = 10000
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if dut.done.value == 1:
            break
    
    assert dut.done.value == 1, "Hardware optimization timed out!"

    x_final_float = q16_ref.q16_to_float(int(dut.x_final.value))
    f_final_float = q16_ref.q16_to_float(int(dut.f_final.value))
    g_final_float = q16_ref.q16_to_float(int(dut.g_final.value))
    H_final_float = q16_ref.q16_to_float(int(dut.H_final.value))
    status_val    = int(dut.status.value)
    iter_val      = int(dut.iter_count.value)

    dut._log.info(f"Results for f(x) = x^4 + 2x^2 - 8x:")
    dut._log.info(f"  Status    : {status_val} (2 = SUCCESS)")
    dut._log.info(f"  Iterations: {iter_val}")
    dut._log.info(f"  x_final   : {x_final_float:.4f} (Expected: 1.0000)")
    dut._log.info(f"  f_final   : {f_final_float:.4f} (Expected: -5.0000)")
    dut._log.info(f"  g_final   : {g_final_float:.4f} (Expected: 0.0000)")
    dut._log.info(f"  H_final   : {H_final_float:.4f} (Expected: 16.0000)")

    assert status_val == 2, f"Expected STATUS_SUCCESS (2), got {status_val}"
    assert abs(x_final_float - 1.0) < 0.05, f"x_final {x_final_float} not close to 1.0"
    assert abs(f_final_float - (-5.0)) < 0.05, f"f_final {f_final_float} not close to -5.0"

@cocotb.test()
async def test_cubic_optimization(dut):
    """Test 3: f(x) = x^3 - x^2 + 1 starting at x=1.0. Hardware should converge to local min at x=2/3 (~0.6667)"""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    await reset_dut(dut)

    dut._log.info("--- Loading DFG Program 3: f(x) = x^3 - x^2 + 1 ---")
    prog3 = [
        encode_instr(OP_LOAD_X, dst=0),
        encode_instr(OP_LOAD_CONST, dst=1, imm=q16_ref.float_to_q16(1.0)),
        encode_instr(OP_MUL, dst=2, srcA=0, srcB=0), # R2 = x^2
        encode_instr(OP_MUL, dst=3, srcA=2, srcB=0), # R3 = x^3
        encode_instr(OP_SUB, dst=4, srcA=3, srcB=2), # R4 = x^3 - x^2
        encode_instr(OP_ADD, dst=5, srcA=4, srcB=1), # R5 = x^3 - x^2 + 1
        encode_instr(OP_END, srcA=5)
    ]
    await load_dfg_program(dut, prog3)

    dut.x_init.value = q16_ref.float_to_q16(1.0)
    dut.start.value = 1
    await RisingEdge(dut.clk)
    dut.start.value = 0

    dut._log.info("Waiting for Newton hardware optimization to complete...")
    timeout_cycles = 10000
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if dut.done.value == 1:
            break
    
    assert dut.done.value == 1, "Hardware optimization timed out!"

    x_final_float = q16_ref.q16_to_float(int(dut.x_final.value))
    f_final_float = q16_ref.q16_to_float(int(dut.f_final.value))
    g_final_float = q16_ref.q16_to_float(int(dut.g_final.value))
    H_final_float = q16_ref.q16_to_float(int(dut.H_final.value))
    status_val    = int(dut.status.value)
    iter_val      = int(dut.iter_count.value)

    dut._log.info(f"Results for f(x) = x^3 - x^2 + 1:")
    dut._log.info(f"  Status    : {status_val} (2 = SUCCESS)")
    dut._log.info(f"  Iterations: {iter_val}")
    dut._log.info(f"  x_final   : {x_final_float:.4f} (Expected: ~0.6667)")
    dut._log.info(f"  f_final   : {f_final_float:.4f} (Expected: ~0.8519)")
    dut._log.info(f"  g_final   : {g_final_float:.4f} (Expected: 0.0000)")
    dut._log.info(f"  H_final   : {H_final_float:.4f} (Expected: ~2.0000)")

    assert status_val == 2, f"Expected STATUS_SUCCESS (2), got {status_val}"
    assert abs(x_final_float - (2.0/3.0)) < 0.05, f"x_final {x_final_float} not close to 0.6667"
    assert abs(f_final_float - (23.0/27.0)) < 0.05, f"f_final {f_final_float} not close to 0.8519"
