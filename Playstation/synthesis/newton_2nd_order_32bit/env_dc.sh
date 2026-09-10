#!/bin/bash
# ==============================================================================
# Environment Configuration for Synopsys Design Compiler (DC)
# Design   : Universal Newton 2nd-Order Accelerator (32-bit Q16.16)
# ==============================================================================

if [ -f "/home/user17/Synopsys/EDASetup.sh" ]; then
    source /home/user17/Synopsys/EDASetup.sh
fi

export SYNTH_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export DESIGN_NAME="newton_2nd_order_top"
export TARGET_LIB_DIR="/home/user17/Synopsys/Design_Compiler/lib"

# Setup Paths
export PATH="${PATH}:/home/user17/Synopsys/Design_Compiler/tool/Synthesis/syn/W-2024.09-SP1/bin"

# Aliases for Convenience
alias dc="dc_shell -topographical_mode"
alias dc_classic="dc_shell"
alias dv="design_vision"

echo "======================================================================"
echo " Synopsys Design Compiler Environment Loaded"
echo " Synthesis Root : ${SYNTH_ROOT}"
echo " Top Module     : ${DESIGN_NAME}"
echo " Tool Command   : $(which dc_shell 2>/dev/null || echo 'Not in PATH')"
echo "======================================================================"
