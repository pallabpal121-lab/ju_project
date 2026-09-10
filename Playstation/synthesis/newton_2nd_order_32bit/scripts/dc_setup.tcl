# ==============================================================================
# Design Compiler Specific Setup (dc_setup.tcl)
# ==============================================================================

# Search Paths
set PDK_LIB_DIR "/home/user17/Synopsys/Design_Compiler/lib"
set RTL_DIR     "../../rtl/newton_2nd_order_32bit"

lappend search_path ${PDK_LIB_DIR} ${RTL_DIR} ${SCRIPTS_DIR} ${CONSTRAINTS_DIR} ${FILELIST_DIR}

# Technology Library Selection
if {${TECH_NODE} == "SCL180"} {
    if {${CORNER} == "slow"} {
        set TARGET_LIB "tsl18fs120_scl_ss.db"
    } elseif {${CORNER} == "fast"} {
        set TARGET_LIB "tsl18fs120_scl_ff.db"
    } else {
        set TARGET_LIB "tsl18fs120_typ.db"
    }
} elseif {${TECH_NODE} == "NANGATE45"} {
    set TARGET_LIB "NangateOpenCellLibrary_slow.db"
} else {
    set TARGET_LIB "tsl18fs120_typ.db"
}

set target_library    [list ${TARGET_LIB}]
set synthetic_library [list standard.sldb]
set link_library      [list * ${target_library} ${synthetic_library}]
set symbol_library    [list generic.sdb]

echo "INFO: Target Library selected -> ${target_library}"
echo "INFO: Link Library   selected -> ${link_library}"

# Set up ALIB cache path
set alib_library_analysis_path "${WORK_DIR}/alib"
file mkdir ${alib_library_analysis_path}

# Setup Formality SVF record
set_svf "${OUTPUTS_DIR}/${DESIGN_NAME}.svf"
