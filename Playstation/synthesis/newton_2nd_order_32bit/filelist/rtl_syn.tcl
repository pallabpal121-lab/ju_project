# ==============================================================================
# Tcl RTL Filelist for Synopsys Design Compiler (DC)
# Design   : Universal Newton 2nd-Order Accelerator (32-bit Q16.16)
# ==============================================================================

set RTL_SOURCE_FILES [list \
    "../../rtl/newton_2nd_order_32bit/newton_types_pkg.sv" \
    "../../rtl/newton_2nd_order_32bit/q16_alu.sv" \
    "../../rtl/newton_2nd_order_32bit/q16_divider.sv" \
    "../../rtl/newton_2nd_order_32bit/dfg_equation_engine.sv" \
    "../../rtl/newton_2nd_order_32bit/derivative_engine.sv" \
    "../../rtl/newton_2nd_order_32bit/newton_2nd_order_top.sv" \
]

set RTL_INCLUDE_DIRS [list \
    "../../rtl/newton_2nd_order_32bit" \
]
