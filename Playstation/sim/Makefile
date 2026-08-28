# ============================================================================
# File: Makefile
# Description: Cocotb simulation Makefile for Newton Accelerator
# ============================================================================

SIM ?= icarus
TOPLEVEL_LANG ?= verilog

VERILOG_SOURCES += $(PWD)/../rtl/q16_types.sv
VERILOG_SOURCES += $(PWD)/../rtl/q16_alu.sv
VERILOG_SOURCES += $(PWD)/../rtl/q16_divider.sv
VERILOG_SOURCES += $(PWD)/../rtl/dfg_evaluator.sv
VERILOG_SOURCES += $(PWD)/../rtl/newton_fd_controller.sv
VERILOG_SOURCES += $(PWD)/../rtl/newton_top.sv

TOPLEVEL = newton_top
MODULE   = test_newton_top

COMPILE_ARGS += -g2012

include $(shell cocotb-config --makefiles)/Makefile.sim
