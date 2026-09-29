# ==============================================================================
# Script 01: Design Compiler Tool & Foundry Library Setup
# File   : 01_dc_setup.tcl
# Role   : Step 0B - Target & Link Standard Cell Library Binding
# Design : Universal Newton 2nd-Order Accelerator (32-bit Q16.16)
# ==============================================================================
# 
# [BEGINNER GUIDE - WHY DO WE NEED THIS FILE?]
# Design Compiler cannot synthesize gates without knowing:
# 1. target_library    : The foundry standard cell database (.db) containing the
#                        actual silicon gates (NAND, NOR, DFF, Buffers, etc.) that
#                        your RTL logic will be converted into.
# 2. synthetic_library : High-level building blocks from Synopsys DesignWare
#                        (adders, multipliers, dividers) used for datapath optimization.
# 3. link_library      : The master lookup table DC consults when resolving any
#                        instantiated sub-module or standard cell. It must contain
#                        "*" (all designs already in DC memory), target_library,
#                        and synthetic_library.
# 4. set_svf           : Records all synthesis optimizations so Synopsys Formality
#                        can mathematically prove that the gate netlist is identical
#                        to your golden RTL.
# ==============================================================================

# ------------------------------------------------------------------------------
# 1. Search Paths Configuration
# ------------------------------------------------------------------------------
# Add PDK library path, RTL directory, and script folders to DC's internal lookup path.
set PDK_LIB_DIR "/home/user17/Synopsys/Design_Compiler/lib"
set RTL_DIR     "../../rtl/newton_2nd_order_32bit"

lappend search_path ${PDK_LIB_DIR} ${RTL_DIR} ${SCRIPTS_DIR} ${CONSTRAINTS_DIR} ${FILELIST_DIR}

# ------------------------------------------------------------------------------
# 2. Foundry Standard Cell Library Selection
# ------------------------------------------------------------------------------
if {${TECH_NODE} == "SCL180"} {
    # Semiconductor Complex Limited (SCL) 180nm Commercial Standard Cells
    if {${CORNER} == "slow"} {
        # Worst-case Slow (1.62 V, 125 C) - used for setup timing sign-off
        set TARGET_LIB "tsl18fs120_scl_ss.db"
    } elseif {${CORNER} == "fast"} {
        # Best-case Fast (1.98 V, -40 C) - used for hold timing sign-off
        set TARGET_LIB "tsl18fs120_scl_ff.db"
    } else {
        # Typical Nominal (1.80 V, 25 C) - default commercial corner
        set TARGET_LIB "tsl18fs120_typ.db"
    }
} elseif {${TECH_NODE} == "NANGATE45"} {
    # Nangate 45nm Open Cell Library (Educational/Research)
    set TARGET_LIB "NangateOpenCellLibrary_slow.db"
} else {
    set TARGET_LIB "tsl18fs120_typ.db"
}

# ------------------------------------------------------------------------------
# 3. Library Variable Assignments
# ------------------------------------------------------------------------------
# Target Library: Physical cells to which DC will map the logic
set target_library    [list ${TARGET_LIB}]

# Synthetic Library: Synopsys DesignWare building blocks (ALU, multipliers, adders)
set synthetic_library [list standard.sldb]

# Link Library: Master resolution list ("*" allows resolving internal RTL hierarchies)
set link_library      [list * ${target_library} ${synthetic_library}]

# Symbol Library: Graphic symbols for Synopsys Design Vision schematic viewer
set symbol_library    [list generic.sdb]

echo "INFO: Target Library selected -> ${target_library}"
echo "INFO: Link Library   selected -> ${link_library}"

# ------------------------------------------------------------------------------
# 4. ALIB Analysis Cache Setup
# ------------------------------------------------------------------------------
# ALIB generates pre-characterized library data to accelerate optimization runtime.
set alib_library_analysis_path "${WORK_DIR}/alib"
file mkdir ${alib_library_analysis_path}

# ------------------------------------------------------------------------------
# 5. Formality Setup Verification File (SVF)
# ------------------------------------------------------------------------------
# Records retiming, register merging, and state encoding transformations for
# downstream formal equivalence checking in Synopsys Formality.
set_svf "${OUTPUTS_DB_DIR}/${DESIGN_NAME}.svf"
