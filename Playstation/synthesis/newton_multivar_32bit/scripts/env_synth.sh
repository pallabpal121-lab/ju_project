#!/bin/bash
# ==============================================================================
# Environment Configuration for Synopsys Design Compiler (DC)
# Design   : Universal Multivariable Newton BCD Accelerator
# Entity   : newton_bcd_axi_lite
# Process  : SCL 180nm Commercial CMOS (0.18 um)
# Voltage  : V_DD = 1.8 V (nominal core)
# ==============================================================================

# 1. Synopsys License Server Configuration
export SNPSLMD_LICENSE_FILE="27020@14.139.1.126"
export LM_LICENSE_FILE="5280@14.139.1.126"

# 2. Tool Binary Paths
export SYNOPSYS_ROOT="/home/user17/Synopsys"
export DC_ROOT="/home/user17/Synopsys/Design_Compiler/tool/Synthesis/syn/W-2024.09-SP1"

if [ -d "${DC_ROOT}/bin" ]; then
    export PATH="${DC_ROOT}/bin:${PATH}"
fi

# 3. SCL 180nm PDK Library Directory & Search Paths
export PDK_LIB_DIR="/home/user17/Synopsys/Design_Compiler/lib"
export SCL180_LIB_DIR="${PDK_LIB_DIR}"
export TARGET_LIB_NAME="tsl18fs120_typ.db"

# PVT Corner Mappings
export SCL180_LIB_TYP="tsl18fs120_typ.db"      # TT : 1.80 V,  25 C
export SCL180_LIB_SS="tsl18fs120_scl_ss.db"    # SS : 1.62 V, 125 C (Worst-Case Setup)
export SCL180_LIB_FF="tsl18fs120_scl_ff.db"    # FF : 1.98 V,   0 C (Best-Case Hold)

# 4. Project Root Directories
export SYNTH_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export RTL_ROOT="$(cd "${SYNTH_ROOT}/../../rtl/newton_multivar_32bit" && pwd)"

# 5. Default Synthesis Parameters
export DESIGN_NAME="newton_bcd_axi_lite"
export TECH_NODE="SCL180"
export CORNER="typical"
export SYNTH_CLK_PERIOD="20.0"

# 6. Interactive Command Aliases
alias dc="dc_shell -64bit"
alias dv="design_vision -64bit"

echo "======================================================================"
echo " Synopsys Design Compiler Synthesis Environment"
echo "  Design Name     : ${DESIGN_NAME}"
echo "  Synthesis Root  : ${SYNTH_ROOT}"
echo "  RTL Root        : ${RTL_ROOT}"
echo "  Technology Node : ${TECH_NODE} (Default Corner: ${CORNER})"
echo "  Target Library  : ${TARGET_LIB_NAME}"
echo "  PDK Directory   : ${PDK_LIB_DIR}"
echo "  dc_shell Path   : $(which dc_shell 2>/dev/null || echo 'Not found in PATH')"
echo "======================================================================"
