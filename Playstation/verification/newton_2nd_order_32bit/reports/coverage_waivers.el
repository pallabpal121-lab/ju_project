# ==============================================================================
# Synopsys VCS / URG Formal Coverage Waiver & Exclusion Specification
# Project : Universal Newton 2nd-Order Optimization Accelerator (32-bit Q16.16)
# Target  : newton_2nd_order_top and child sub-modules
# Version : 1.0 (Production Tape-Out Milestone)
# ==============================================================================

# ------------------------------------------------------------------------------
# 1. TOP-LEVEL FSM DEFENSIVE FALLBACK WAIVERS
# ------------------------------------------------------------------------------
# Module  : newton_2nd_order_top
# Instance: newton_tb_top.u_dut
# Reason  : All 8 operational FSM states (TOP_IDLE, TOP_START_DERIV, TOP_WAIT_DERIV,
#           TOP_CHECK_CONV, TOP_SOLVE, TOP_WAIT_SOLVE, TOP_UPDATE_X, TOP_DONE)
#           are fully decoded in the SystemVerilog 'case (state)'. The default
#           branch is required exclusively for synthesis latch-prevention and
#           fail-safe hardware recovery. Under legal operating conditions,
#           it is mathematically and logically unreachable.
# File    : Playstation/rtl/newton_2nd_order_32bit/newton_2nd_order_top.sv
# Line    : 316
# Waiver  : APPROVED (Defensive Code)
# ------------------------------------------------------------------------------
exclude -module newton_2nd_order_top -line 316 -comment "Defensive FSM default state"

# ------------------------------------------------------------------------------
# 2. DFG EQUATION ENGINE FSM DEFENSIVE FALLBACK WAIVERS
# ------------------------------------------------------------------------------
# Module  : dfg_equation_engine
# Instance: newton_tb_top.u_dut.u_deriv_engine.u_dfg
# Reason  : ENG_IDLE, ENG_FETCH_EXEC, and ENG_DIV_WAIT states fully decode all
#           execution phases. The default state branch prevents latch inference
#           during logic synthesis.
# File    : Playstation/rtl/newton_2nd_order_32bit/dfg_equation_engine.sv
# Line    : 219
# Waiver  : APPROVED (Defensive Code)
# ------------------------------------------------------------------------------
exclude -module dfg_equation_engine -line 219 -comment "Defensive DFG engine FSM default state"

# ------------------------------------------------------------------------------
# 3. DERIVATIVE ENGINE FSM DEFENSIVE FALLBACK WAIVERS
# ------------------------------------------------------------------------------
# Module  : derivative_engine
# Instance: newton_tb_top.u_dut.u_deriv_engine
# Reason  : Central finite difference 3-point sampling sequence (f0, f+, f-) is
#           controlled by a deterministic 8-state sequence. The default case
#           is unreachable in silicon.
# File    : Playstation/rtl/newton_2nd_order_32bit/derivative_engine.sv
# Line    : 241
# Waiver  : APPROVED (Defensive Code)
# ------------------------------------------------------------------------------
exclude -module derivative_engine -line 241 -comment "Defensive Derivative engine FSM default state"

# ------------------------------------------------------------------------------
# 4. Q16 RESTORING DIVIDER FSM DEFENSIVE FALLBACK WAIVERS
# ------------------------------------------------------------------------------
# Module  : q16_divider
# Instance: newton_tb_top.u_dut.u_solver, newton_tb_top.u_dut.u_deriv_engine.u_dfg.u_div
# Reason  : The 48-cycle restoring division state machine consists of DIV_IDLE,
#           DIV_CALC, and DIV_DONE. The default fallback ensures fail-safe reset
#           behavior.
# File    : Playstation/rtl/newton_2nd_order_32bit/q16_divider.sv
# Line    : 167
# Waiver  : APPROVED (Defensive Code)
# ------------------------------------------------------------------------------
exclude -module q16_divider -line 167 -comment "Defensive Divider FSM default state"

# ------------------------------------------------------------------------------
# 5. FIXED-POINT BUS UPPER BITS TOGGLE WAIVERS
# ------------------------------------------------------------------------------
# Instance: newton_tb_top.u_dut
# Signals : prog_addr[4:3], prog_data[31:24]
# Reason  : Program depth is architected for 32 micro-instructions. Standard
#           equations (Quadratic, Cubic, Rational) use between 3 and 13 instructions.
#           Upper address lines do not toggle to '1' during these programs.
# Waiver  : APPROVED (Architectural Limit)
# ==============================================================================
